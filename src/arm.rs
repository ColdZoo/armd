//! 每个机械臂一个 worker 线程：独占一条串口总线 + 自己的命令/响应/发布队列。
//!
//! 队列设计（性能/实时性关键）：
//! - `urgent`（有界 64）：写寄存器、扭矩、sync_write 等"要立刻执行"的命令，优先处理
//! - `normal`（有界 256）：读快照、ping、改速率等，每轮最多消费 16 个，保证轮询不被饿死
//! - 轮询：按 `poll_hz` 合并为一次 sync_read，不排队；读到的位置进入状态缓存
//! - 发布：每个订阅者一个有界通道（容量 32），满了就丢最旧帧并计数（慢客户端不拖累总线）
//!
//! 安全层（v0.2.0，见 `docs/SAFETY-DESIGN.md`）：
//! - 堵转/限位保护：Goal-Present 持续偏差 + 速度为 0 → 该电机 torque OFF + goal 回退；
//! - 单电机故障隔离：硬件 error / 无响应只标记该电机，整臂仍 connected、继续轮询；
//! - 串口 fd 生命周期：`Link` 保证 open 前先 close，读失败先释放 fd 再重连；
//! - `arm.reboot` / `arm.clear_error` / `arm.torque_off` / `arm.sim.inject` RPC。

use crate::config::ArmConfig;
use crate::dxl;
use crate::port::{ArmIo, DxlPort, IoError, Link, SimPort, SyncReadResult};
use crate::profile::{joint_degrees, ProfileStore};
use crate::safety::{self, SoftLimitConfig, StallConfig, StallTracker, StallTrigger};
use serde_json::{json, Value};
use std::collections::{BTreeMap, HashSet};
use std::sync::atomic::{AtomicBool, AtomicI64, AtomicU64, Ordering};
use std::sync::mpsc::{sync_channel, Receiver, SyncSender, TryRecvError};
use std::sync::{Arc, Mutex, RwLock};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

pub fn now_ms() -> u64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_millis() as u64).unwrap_or(0)
}

// ---------------------------------------------------------------- 安全状态

/// 按电机上报的故障（硬件错误或连续无响应）。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MotorError {
    /// 硬件 error 字节；`None` 表示连续无响应
    pub code: Option<u8>,
    pub names: Vec<String>,
    pub message: String,
    pub ts_ms: u64,
    /// 首次出现时间（同一错误持续期间不变）
    pub since_ms: u64,
}

impl MotorError {
    pub fn hardware(id: u8, code: u8, ts_ms: u64) -> Self {
        Self {
            code: Some(code),
            names: safety::hardware_error_names(code).iter().map(|s| s.to_string()).collect(),
            message: format!("motor {id}: {}", safety::hardware_error_message(code)),
            ts_ms,
            since_ms: ts_ms,
        }
    }

    pub fn no_response(id: u8, ts_ms: u64) -> Self {
        Self {
            code: None,
            names: vec!["no_response".into()],
            message: format!("motor {id}: 连续无响应（总线上读不到该舵机，可能掉线/未上电）"),
            ts_ms,
            since_ms: ts_ms,
        }
    }

    pub fn to_json(&self) -> Value {
        json!({
            "code": self.code,
            "names": self.names,
            "message": self.message,
            "ts": self.ts_ms,
            "since_ms": self.since_ms,
        })
    }
}

/// 一次堵转保护事件（触发后保留到上层显式清除/重新使能）。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StallEvent {
    pub motor: u8,
    pub reason: String,
    pub position: i64,
    pub goal: i64,
    pub stall_ms: u64,
    pub ts_ms: u64,
    pub torque_off: bool,
    pub action: String,
    pub reverted_goal: i64,
}

impl StallEvent {
    pub fn to_json(&self) -> Value {
        json!({
            "motor": self.motor,
            "reason": self.reason,
            "position": self.position,
            "goal": self.goal,
            "stall_ms": self.stall_ms,
            "ts": self.ts_ms,
            "torque_off": self.torque_off,
            "action": self.action,
            "reverted_goal": self.reverted_goal,
        })
    }
}

/// 当前生效的安全配置摘要（随状态帧下发，方便客户端展示）。
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct SafetyInfo {
    pub stall_protection: bool,
    pub stall_timeout_ms: u64,
    pub stall_tolerance: i64,
    pub auto_torque_off: bool,
    pub soft_limits: bool,
    pub stall_exclude_motors: Vec<u8>,
}

impl SafetyInfo {
    pub fn to_json(&self) -> Value {
        json!({
            "stall_protection": self.stall_protection,
            "stall_timeout_ms": self.stall_timeout_ms,
            "stall_tolerance": self.stall_tolerance,
            "auto_torque_off": self.auto_torque_off,
            "soft_limits": self.soft_limits,
            "stall_exclude_motors": self.stall_exclude_motors,
        })
    }
}

#[derive(Debug, Clone, Default)]
pub struct ArmState {
    pub seq: u64,
    pub ts_ms: u64,
    pub connected: bool,
    pub joints: BTreeMap<u8, i64>,
    pub velocity: BTreeMap<u8, i64>,
    pub hz: f64,
    pub error: Option<String>,
    pub read_us: u64,
    /// 监控数据（每舵机：电压 V / 温度 ℃），`monitor: true` 时有效
    pub health: BTreeMap<u8, (f64, i64)>,
    /// 有 profile 时的关节角（度）
    pub deg: BTreeMap<String, f64>,
    /// 舵机 id -> 关节名
    pub joint_names: BTreeMap<String, String>,
    pub profile_saved_at: Option<i64>,
    // ------------------------------------------------------------ 安全层
    /// 按电机上报的故障（latched 硬件错误 / 连续无响应）
    pub motor_errors: BTreeMap<u8, MotorError>,
    /// 堵转保护事件（按电机，清除前一直保留）
    pub stalls: BTreeMap<u8, StallEvent>,
    /// 已知的 Torque_Enable 状态
    pub torque: BTreeMap<u8, bool>,
    /// EEPROM Min/Max Position Limit（读到才出现）
    pub limits: BTreeMap<u8, (i64, i64)>,
    /// armd 最后接受的目标（软限位后）
    pub goals: BTreeMap<u8, i64>,
    pub safety: SafetyInfo,
}

impl ArmState {
    pub fn to_json(&self) -> Value {
        let joints: serde_json::Map<String, Value> =
            self.joints.iter().map(|(k, v)| (k.to_string(), json!(v))).collect();
        json!({
            "seq": self.seq,
            "ts": self.ts_ms,
            "connected": self.connected,
            "joints": Value::Object(joints),
            "velocity": self.velocity.iter().map(|(k, v)| (k.to_string(), json!(v)))
                .collect::<serde_json::Map<String, Value>>(),
            "hz": (self.hz * 10.0).round() / 10.0,
            "error": self.error,
            "read_us": self.read_us,
            "health": self.health.iter().map(|(id, (v, t))| {
                (id.to_string(), json!({"voltage": v, "temp": t}))
            }).collect::<serde_json::Map<String, Value>>(),
            "joint_names": self.joint_names.iter().map(|(k, v)| (k.clone(), json!(v)))
                .collect::<serde_json::Map<String, Value>>(),
            "deg": self.deg.iter().map(|(k, v)| (k.clone(), json!((v * 100.0).round() / 100.0)))
                .collect::<serde_json::Map<String, Value>>(),
            "profile_saved_at": self.profile_saved_at,
            "motor_errors": self.motor_errors.iter().map(|(id, e)| (id.to_string(), e.to_json()))
                .collect::<serde_json::Map<String, Value>>(),
            "stalls": self.stalls.iter().map(|(id, e)| (id.to_string(), e.to_json()))
                .collect::<serde_json::Map<String, Value>>(),
            "torque": self.torque.iter().map(|(id, on)| (id.to_string(), json!(on)))
                .collect::<serde_json::Map<String, Value>>(),
            "limits": self.limits.iter().map(|(id, (lo, hi))| {
                (id.to_string(), json!({"min": lo, "max": hi}))
            }).collect::<serde_json::Map<String, Value>>(),
            "goals": self.goals.iter().map(|(id, g)| (id.to_string(), json!(g)))
                .collect::<serde_json::Map<String, Value>>(),
            "safety": self.safety.to_json(),
        })
    }
}

#[derive(Default)]
pub struct Stats {
    pub polls: AtomicU64,
    pub read_errors: AtomicU64,
    pub writes: AtomicU64,
    pub frames_published: AtomicU64,
    pub frames_dropped: AtomicU64,
    pub reconnects: AtomicU64,
    pub urgent_queued: AtomicI64,
    pub normal_queued: AtomicI64,
    // 安全层
    pub stalls: AtomicU64,
    pub torque_offs: AtomicU64,
    pub opens: AtomicU64,
    pub closes: AtomicU64,
}

impl Stats {
    pub fn to_json(&self) -> Value {
        json!({
            "polls": self.polls.load(Ordering::Relaxed),
            "read_errors": self.read_errors.load(Ordering::Relaxed),
            "writes": self.writes.load(Ordering::Relaxed),
            "frames_published": self.frames_published.load(Ordering::Relaxed),
            "frames_dropped": self.frames_dropped.load(Ordering::Relaxed),
            "reconnects": self.reconnects.load(Ordering::Relaxed),
            "urgent_queued": self.urgent_queued.load(Ordering::Relaxed),
            "normal_queued": self.normal_queued.load(Ordering::Relaxed),
            "stalls": self.stalls.load(Ordering::Relaxed),
            "torque_offs": self.torque_offs.load(Ordering::Relaxed),
            "opens": self.opens.load(Ordering::Relaxed),
            "closes": self.closes.load(Ordering::Relaxed),
        })
    }
}

/// 一个订阅者：有界发送队列 + 速率限制 + 关闭标记。
pub struct Sub {
    pub id: u64,
    pub arms: HashSet<String>,
    rate_mhz: AtomicU64, // Hz * 1000
    /// 订阅频率 >= 54Hz 时直通（不按时间丢帧），避免轮询抖动导致 60Hz 订阅掉到 50fps
    passthrough: AtomicBool,
    last: Mutex<Instant>,
    tx: SyncSender<String>,
    pub closed: AtomicBool,
    pub dropped: AtomicU64,
}

impl Sub {
    pub fn new(id: u64, arms: HashSet<String>, rate_hz: f64, tx: SyncSender<String>) -> Self {
        Self {
            id,
            arms,
            rate_mhz: AtomicU64::new((rate_hz.clamp(0.1, 1000.0) * 1000.0) as u64),
            passthrough: AtomicBool::new(rate_hz >= 54.0),
            last: Mutex::new(Instant::now() - Duration::from_secs(3600)),
            tx,
            closed: AtomicBool::new(false),
            dropped: AtomicU64::new(0),
        }
    }

    pub fn wants(&self, arm: &str) -> bool {
        !self.closed.load(Ordering::Relaxed) && self.arms.contains(arm)
    }

    pub fn set_rate(&self, rate_hz: f64) {
        self.rate_mhz.store((rate_hz.clamp(0.1, 1000.0) * 1000.0) as u64, Ordering::Relaxed);
        self.passthrough.store(rate_hz >= 54.0, Ordering::Relaxed);
    }

    pub fn due(&self) -> bool {
        if self.passthrough.load(Ordering::Relaxed) {
            return true;
        }
        let mut last = self.last.lock().unwrap();
        let rate = self.rate_mhz.load(Ordering::Relaxed) as f64 / 1000.0;
        // 留 3ms 容差：轮询抖动（比如某次晚 2ms、下次立刻补）时不至于隔帧丢
        let need = (1.0 / rate - 0.003).max(0.001);
        if last.elapsed().as_secs_f64() < need {
            return false;
        }
        *last = Instant::now();
        true
    }

    fn send(&self, frame: String) -> bool {
        match self.tx.try_send(frame) {
            Ok(()) => true,
            Err(_) => {
                self.dropped.fetch_add(1, Ordering::Relaxed);
                false
            }
        }
    }
}

pub enum Cmd {
    SyncRead { addr: u16, len: u16, ids: Vec<u8>, timeout_ms: u64 },
    ReadRegister { id: u8, addr: u16, len: u16 },
    WriteRegister { id: u8, addr: u16, data: Vec<u8> },
    SyncWrite { addr: u16, len: u16, values: Vec<(u8, Vec<u8>)> },
    Ping { id: u8 },
    Reconnect,
    SetRate { hz: f64 },
    /// Dynamixel Reboot(0x08)：清除 latched 硬件错误
    Reboot { ids: Vec<u8> },
    /// 清除 armd 侧的按电机故障/堵转标记（latched 硬件错误仍需 reboot）
    ClearError { ids: Vec<u8> },
    /// 仿真故障注入（仅 sim 臂）：stall / hardware_error / no_response / clear
    SimInject { kind: String, ids: Vec<u8>, enable: bool },
}

pub type Reply = SyncSender<Result<Value, IoError>>;

pub struct Request {
    pub cmd: Cmd,
    pub reply: Option<Reply>,
}

pub struct ArmHandle {
    pub cfg: ArmConfig,
    pub profile_store: ProfileStore,
    pub profile: Arc<Mutex<Option<Value>>>,
    pub urgent: SyncSender<Request>,
    pub normal: SyncSender<Request>,
    pub state: Arc<RwLock<ArmState>>,
    pub subs: Arc<Mutex<Vec<Arc<Sub>>>>,
    pub stats: Arc<Stats>,
    stop: Arc<AtomicBool>,
}

impl ArmHandle {
    pub fn spawn(cfg: ArmConfig) -> Arc<Self> {
        let (urgent_tx, urgent_rx) = sync_channel(64);
        let (normal_tx, normal_rx) = sync_channel(256);
        let state = Arc::new(RwLock::new(ArmState::default()));
        let subs: Arc<Mutex<Vec<Arc<Sub>>>> = Arc::new(Mutex::new(Vec::new()));
        let stats = Arc::new(Stats::default());
        let stop = Arc::new(AtomicBool::new(false));
        let profile_store = ProfileStore::new(
            cfg.profile.clone().map(Into::into)
                .unwrap_or_else(|| ProfileStore::default_path(&cfg.id)));
        let profile = Arc::new(Mutex::new(profile_store.load()));

        let safety_info = SafetyInfo {
            stall_protection: cfg.stall_protection,
            stall_timeout_ms: cfg.stall_timeout_ms,
            stall_tolerance: cfg.stall_tolerance,
            auto_torque_off: cfg.auto_torque_off,
            soft_limits: cfg.soft_limits,
            stall_exclude_motors: cfg.stall_exclude_motors.clone(),
        };
        {
            let mut st = state.write().unwrap();
            st.safety = safety_info.clone();
            st.joint_names = cfg.joint_names.clone();
        }

        let stall_cfg = StallConfig {
            enabled: cfg.stall_detection_active(),
            timeout: Duration::from_millis(cfg.stall_timeout_ms.max(50)),
            tolerance: cfg.stall_tolerance.max(1),
            velocity_epsilon: cfg.stall_velocity_epsilon.max(0),
        };
        let soft_limit_cfg =
            SoftLimitConfig { enabled: cfg.soft_limits, margin: cfg.soft_limit_margin };

        let worker = Worker {
            cfg: cfg.clone(),
            link: Link::new(),
            connected: false,
            hz: 0.0,
            last_monitor: Instant::now() - Duration::from_secs(3600),
            polls_in_window: 0,
            window_start: Instant::now(),
            next_poll: Instant::now(),
            last_poll_at: None,
            last_reconnect: Instant::now() - Duration::from_secs(3600),
            transport_errors_in_row: 0,
            state: state.clone(),
            subs: subs.clone(),
            stats: stats.clone(),
            profile: profile.clone(),
            goals: BTreeMap::new(),
            torque: BTreeMap::new(),
            modes: BTreeMap::new(),
            limits: BTreeMap::new(),
            motor_errors: BTreeMap::new(),
            stalls: BTreeMap::new(),
            read_fails: BTreeMap::new(),
            stall_tracker: StallTracker::new(stall_cfg),
            soft_limit_cfg,
            safety_info,
            warned_velocity: false,
            warned_limits: false,
            suppress_missing_until: None,
            last_refresh: Instant::now() - Duration::from_secs(3600),
        };
        if !cfg.sim && !cfg.port.is_empty() {
            // 启动时打一条便于排障的连接目标日志
            eprintln!("[armd] arm {} 串口 {} @ {}", cfg.id, cfg.port, cfg.baud);
        }

        let worker_cfg = cfg.clone();
        let stop_thread = stop.clone();
        std::thread::Builder::new()
            .name(format!("arm-{}", cfg.id))
            .spawn(move || worker.run(worker_cfg, urgent_rx, normal_rx, stop_thread))
            .expect("spawn arm worker");

        Arc::new(Self {
            cfg,
            profile_store,
            profile,
            urgent: urgent_tx,
            normal: normal_tx,
            state,
            subs,
            stats,
            stop,
        })
    }

    pub fn submit(&self, req: Request, urgent: bool) -> Result<(), &'static str> {
        let (tx, depth, name) = if urgent {
            (&self.urgent, &self.stats.urgent_queued, "urgent")
        } else {
            (&self.normal, &self.stats.normal_queued, "normal")
        };
        match tx.try_send(req) {
            Ok(()) => {
                depth.fetch_add(1, Ordering::Relaxed);
                Ok(())
            }
            Err(std::sync::mpsc::TrySendError::Full(_)) => Err(name),
            Err(std::sync::mpsc::TrySendError::Disconnected(_)) => Err(name),
        }
    }

    pub fn stop(&self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}

struct Worker {
    cfg: ArmConfig,
    link: Link,
    connected: bool,
    hz: f64,
    last_monitor: Instant,
    polls_in_window: u32,
    window_start: Instant,
    next_poll: Instant,
    last_poll_at: Option<Instant>,
    last_reconnect: Instant,
    transport_errors_in_row: u32,
    state: Arc<RwLock<ArmState>>,
    subs: Arc<Mutex<Vec<Arc<Sub>>>>,
    stats: Arc<Stats>,
    profile: Arc<Mutex<Option<Value>>>,
    // ------------------------------------------------------------ 安全层
    goals: BTreeMap<u8, i64>,
    torque: BTreeMap<u8, bool>,
    modes: BTreeMap<u8, u16>,
    limits: BTreeMap<u8, (i64, i64)>,
    motor_errors: BTreeMap<u8, MotorError>,
    stalls: BTreeMap<u8, StallEvent>,
    read_fails: BTreeMap<u8, u32>,
    stall_tracker: StallTracker,
    soft_limit_cfg: SoftLimitConfig,
    safety_info: SafetyInfo,
    warned_velocity: bool,
    warned_limits: bool,
    suppress_missing_until: Option<Instant>,
    last_refresh: Instant,
}

impl Worker {
    // ------------------------------------------------------------ 串口生命周期

    fn close_io(&mut self) -> bool {
        let closed = self.link.close();
        if closed {
            self.stats.closes.fetch_add(1, Ordering::Relaxed);
            self.connected = false;
        }
        closed
    }

    /// 打开串口。注意 [`Link::open_with`] 会先关闭旧句柄——这是修复 fd 泄漏的关键。
    fn open_io(&mut self) -> Result<(), IoError> {
        self.close_io(); // 防御性：确保进入 open 前没有旧 fd
        let sim = self.cfg.sim;
        let port = self.cfg.port.clone();
        let baud = self.cfg.baud;
        let motors = self.cfg.motors.clone();
        let res = self.link.open_with(|| {
            if sim {
                Ok(Box::new(SimPort::new(&motors)) as Box<dyn ArmIo>)
            } else {
                Ok(Box::new(DxlPort::open(&port, baud)?) as Box<dyn ArmIo>)
            }
        });
        if res.is_ok() {
            self.stats.opens.fetch_add(1, Ordering::Relaxed);
        }
        res
    }

    fn set_state_connected(&self, connected: bool, error: Option<String>) {
        let mut st = self.state.write().unwrap();
        st.connected = connected;
        st.error = error;
        st.ts_ms = now_ms();
    }

    fn sync_safety_state(&self) {
        let mut st = self.state.write().unwrap();
        st.motor_errors = self.motor_errors.clone();
        st.stalls = self.stalls.clone();
        st.torque = self.torque.clone();
        st.limits = self.limits.clone();
        st.goals = self.goals.clone();
        st.safety = self.safety_info.clone();
    }

    // ------------------------------------------------------------ 发布

    fn publish(&self, frame: Value) -> String {
        let line = frame.to_string();
        let subs = self.subs.lock().unwrap();
        for s in subs.iter() {
            if !s.wants(&self.cfg.id) || !s.due() {
                continue;
            }
            let mut f = frame.clone();
            if let Value::Object(map) = &mut f {
                map.insert("sub".into(), json!(s.id));
            }
            if s.send(f.to_string()) {
                self.stats.frames_published.fetch_add(1, Ordering::Relaxed);
            } else {
                self.stats.frames_dropped.fetch_add(1, Ordering::Relaxed);
            }
        }
        line
    }

    fn publish_state(&self) {
        let frame = {
            let st = self.state.read().unwrap();
            let mut v = st.to_json();
            if let Value::Object(map) = &mut v {
                map.insert("event".into(), json!("state"));
                map.insert("arm".into(), json!(self.cfg.id));
            }
            v
        };
        self.publish(frame);
    }

    fn publish_event(&self, connected: bool, error: Option<&str>) {
        let frame = json!({
            "event": "arm", "arm": self.cfg.id, "connected": connected,
            "error": error, "ts": now_ms(),
        });
        self.publish(frame);
    }

    fn publish_motor_error(&self, id: u8, err: &MotorError) {
        let frame = json!({
            "event": "motor_error", "arm": self.cfg.id, "motor": id,
            "code": err.code, "names": err.names, "message": err.message, "ts": err.ts_ms,
        });
        self.publish(frame);
    }

    fn publish_motor_recovered(&self, id: u8) {
        let frame = json!({
            "event": "motor_recovered", "arm": self.cfg.id, "motor": id, "ts": now_ms(),
        });
        self.publish(frame);
    }

    fn publish_stall(&self, ev: &StallEvent) {
        let mut frame = ev.to_json();
        if let Value::Object(map) = &mut frame {
            map.insert("event".into(), json!("stall"));
            map.insert("arm".into(), json!(self.cfg.id));
        }
        self.publish(frame);
    }

    // ------------------------------------------------------------ 按电机故障

    fn mark_motor_error(&mut self, id: u8, code: Option<u8>) {
        let now = now_ms();
        let next = match code {
            Some(code) => MotorError::hardware(id, code, now),
            None => MotorError::no_response(id, now),
        };
        let since = self
            .motor_errors
            .get(&id)
            .filter(|old| old.code == next.code && old.names == next.names)
            .map(|old| old.since_ms)
            .unwrap_or(now);
        let mut next = next;
        next.since_ms = since;
        if self.motor_errors.get(&id) == Some(&next) {
            return;
        }
        self.motor_errors.insert(id, next.clone());
        self.publish_motor_error(id, &next);
    }

    fn clear_motor_error(&mut self, id: u8) {
        if self.motor_errors.remove(&id).is_some() {
            self.publish_motor_recovered(id);
        }
    }

    fn clear_stall_marker(&mut self, id: u8) {
        self.stalls.remove(&id);
        self.stall_tracker.reset(id);
    }

    // ------------------------------------------------------------ 连接 / 握手

    fn reset_link_state(&mut self) {
        self.read_fails.clear();
        self.suppress_missing_until = None;
        self.transport_errors_in_row = 0;
        // 重连后不保留旧目标/堵转标记：舵机可能已经断电重启，绝不自动把旧目标再推一遍。
        self.goals.clear();
        self.stalls.clear();
        self.stall_tracker.reset_all();
        // torque / limits / motor_errors 由握手 + refresh 重新发现
        self.torque.clear();
        self.modes.clear();
        self.motor_errors.clear();
    }

    fn ensure_connected(&mut self, stop: &AtomicBool) {
        if self.connected {
            return;
        }
        if self.last_reconnect.elapsed().as_secs_f64() < self.cfg.reconnect_sec {
            return;
        }
        self.last_reconnect = Instant::now();
        if stop.load(Ordering::Relaxed) {
            return;
        }
        match self.open_io() {
            Ok(()) => self.handshake(),
            Err(e) => {
                self.connected = false;
                self.set_state_connected(false, Some(e.to_string()));
                self.publish_event(false, Some(&e.to_string()));
            }
        }
    }

    /// 握手：ping 所有电机。≥1 个响应即认为总线活着（单电机故障不拖垮整臂）。
    fn handshake(&mut self) {
        let motors = self.cfg.motors.clone();
        let mut responded = 0usize;
        let mut hw_errors: Vec<(u8, u8)> = Vec::new();
        let mut missing: Vec<u8> = Vec::new();
        let mut port_error: Option<IoError> = None;
        for id in &motors {
            let r = match self.link.io() {
                Some(io) => io.ping(*id),
                None => Err(IoError::Busy),
            };
            match r {
                Ok(_) => responded += 1,
                Err(IoError::Device { id, error }) => {
                    responded += 1;
                    hw_errors.push((id, error));
                }
                Err(e) if e.is_transport() => {
                    port_error = Some(e);
                    break;
                }
                Err(_) => missing.push(*id),
            }
        }
        if let Some(e) = port_error {
            self.close_io();
            self.set_state_connected(false, Some(e.to_string()));
            self.publish_event(false, Some(&e.to_string()));
            return;
        }
        if responded == 0 {
            self.close_io(); // 一个都没响应：释放 fd，等下一轮重连（避免占着串口）
            self.set_state_connected(
                false,
                Some(format!("握手失败：0/{} 个舵机响应", motors.len())),
            );
            self.publish_event(false, Some("握手失败：0 个舵机响应"));
            return;
        }
        self.reset_link_state();
        for (id, code) in hw_errors {
            self.mark_motor_error(id, Some(code));
        }
        for id in missing {
            self.mark_motor_error(id, None);
        }
        self.connected = true;
        self.last_poll_at = None;
        self.next_poll = Instant::now();
        self.stats.reconnects.fetch_add(1, Ordering::Relaxed);
        self.set_state_connected(true, None);
        self.publish_event(true, None);
        self.refresh_motor_state();
        self.sync_safety_state();
        eprintln!(
            "[armd] arm {} 已连接：{}/{} 个舵机响应，per-motor errors={}",
            self.cfg.id,
            responded,
            motors.len(),
            self.motor_errors.len()
        );
    }

    /// 是否还有电机的 torque/limits/mode 未知（需要重试读取）。
    fn motor_state_incomplete(&self) -> bool {
        self.cfg.motors.iter().any(|id| {
            !self.torque.contains_key(id)
                || !self.limits.contains_key(id)
                || !self.modes.contains_key(id)
        })
    }

    /// 读 Operating_Mode(11)、Torque_Enable(64) 与 EEPROM Min/Max Position Limit(48)，并落状态。
    fn refresh_motor_state(&mut self) {
        self.last_refresh = Instant::now();
        let motors = self.cfg.motors.clone();
        let timeout = Duration::from_millis(self.cfg.read_timeout_ms.max(20));
        let torque_res = match self.link.io() {
            Some(io) => io.sync_read(dxl::ADDR_TORQUE_ENABLE, 1, &motors, timeout),
            None => return,
        };
        if let Ok(res) = torque_res {
            for (id, data) in res.data {
                self.torque.insert(id, dxl::le_int(&data) != 0);
            }
        }
        let mode_res = match self.link.io() {
            Some(io) => io.sync_read(dxl::ADDR_OPERATING_MODE, 1, &motors, timeout),
            None => return,
        };
        if let Ok(res) = mode_res {
            for (id, data) in res.data {
                self.modes.insert(id, dxl::le_int(&data) as u16);
            }
        }
        let limits_res = match self.link.io() {
            Some(io) => io.sync_read(dxl::ADDR_MAX_POSITION_LIMIT, 4, &motors, timeout),
            None => return,
        };
        if let Ok(res) = limits_res {
            for (id, data) in res.data {
                if let Some((min, max)) = dxl::position_limits(&data) {
                    self.limits.insert(id, (min, max));
                }
            }
        }
        if self.cfg.soft_limits && !self.warned_limits {
            let unknown: Vec<u8> = motors
                .iter()
                .copied()
                .filter(|id| !self.limits.contains_key(id))
                .collect();
            if !unknown.is_empty() {
                self.warned_limits = true;
                eprintln!(
                    "[armd] arm {} 警告：motor {:?} 的 EEPROM Min/Max Position Limit 读取失败，软限位暂时只能做绝对范围（0..4095）夹紧",
                    self.cfg.id, unknown
                );
            }
        }
        self.sync_safety_state();
    }

    // ------------------------------------------------------------ 轮询

    fn poll(&mut self) {
        let (addr, len) = self.cfg.poll_window();
        let motors = self.cfg.motors.clone();
        let timeout = Duration::from_millis(self.cfg.read_timeout_ms);
        let t0 = Instant::now();
        let result = match self.link.io() {
            Some(io) => io.sync_read(addr, len, &motors, timeout),
            None => Err(IoError::Busy),
        };
        let elapsed_us = t0.elapsed().as_micros() as u64;
        match result {
            Ok(res) if !res.is_empty() => self.on_poll_ok(res, elapsed_us),
            Ok(_) => self.on_poll_transport_failure("全部舵机无响应（sync_read 空结果）"),
            Err(e) => self.on_poll_transport_failure(&e.to_string()),
        }
    }

    fn on_poll_ok(&mut self, res: SyncReadResult, elapsed_us: u64) {
        self.transport_errors_in_row = 0;
        self.stats.polls.fetch_add(1, Ordering::Relaxed);
        self.polls_in_window += 1;
        if self.polls_in_window >= 30 {
            let dt = self.window_start.elapsed().as_secs_f64();
            if dt > 0.0 {
                self.hz = self.polls_in_window as f64 / dt;
            }
            self.polls_in_window = 0;
            self.window_start = Instant::now();
        }
        self.last_poll_at = Some(Instant::now());

        let (addr, len) = self.cfg.poll_window();
        let mut joints: BTreeMap<u8, i64> = BTreeMap::new();
        let mut velocity: BTreeMap<u8, i64> = BTreeMap::new();
        for (id, data) in &res.data {
            if res.errors.contains_key(id) {
                continue; // 该电机处于硬件错误状态，位置数据不可信
            }
            if len >= 8 && addr <= dxl::ADDR_PRESENT_VELOCITY {
                let offset = (dxl::ADDR_PRESENT_VELOCITY.saturating_sub(addr)) as usize;
                if data.len() >= offset + 8 {
                    if let Some((v, p)) = dxl::velocity_position(&data[offset..]) {
                        velocity.insert(*id, v);
                        joints.insert(*id, p);
                        continue;
                    }
                }
            }
            joints.insert(*id, dxl::le_int(data));
        }

        // 按电机更新故障状态：有数据的恢复、有 error 的标记、缺失的累计
        let now = Instant::now();
        let suppress_missing =
            self.suppress_missing_until.map(|t| now < t).unwrap_or(false);
        let configured_motors = self.cfg.motors.clone();
        for id in &configured_motors {
            if res.data.contains_key(id) && !res.errors.contains_key(id) {
                self.read_fails.remove(id);
                self.clear_motor_error(*id);
            } else if let Some(code) = res.errors.get(id) {
                self.read_fails.remove(id);
                self.mark_motor_error(*id, Some(*code));
            } else if !suppress_missing {
                let fails = self.read_fails.entry(*id).or_insert(0);
                *fails += 1;
                if *fails >= 10 {
                    self.mark_motor_error(*id, None);
                }
            }
        }

        // 堵转保护（最后一道防线）
        if self.cfg.stall_detection_active() {
            self.run_stall_protection(&joints, &velocity, now);
        } else if !self.warned_velocity && self.cfg.stall_protection {
            self.warned_velocity = true;
            eprintln!(
                "[armd] arm {} 警告：显式配置了 poll_addr/poll_len 导致读不到 Present_Velocity，堵转保护自动跳过",
                self.cfg.id
            );
        }

        // 写状态缓存
        let profile = self.profile.lock().unwrap().clone();
        {
            let mut st = self.state.write().unwrap();
            st.seq += 1;
            st.ts_ms = now_ms();
            st.connected = true;
            st.error = None;
            st.joints = joints.clone();
            st.velocity = velocity.clone();
            st.hz = self.hz;
            st.read_us = elapsed_us;
            st.joint_names = self.cfg.joint_names.clone();
            st.motor_errors = self.motor_errors.clone();
            st.stalls = self.stalls.clone();
            st.torque = self.torque.clone();
            st.limits = self.limits.clone();
            st.goals = self.goals.clone();
            st.safety = self.safety_info.clone();
            if let Some(p) = &profile {
                st.deg = joint_degrees(&joints, &self.cfg.joint_names, p);
                st.profile_saved_at = p.get("savedAt").and_then(|v| v.as_i64());
            }
        }
        if !self.connected {
            self.connected = true;
            self.publish_event(true, None);
        }
        self.poll_health();
        // torque/limits/mode 正常在连接时读一次、之后由 armd 的写入跟踪维护；
        // 只有还没读到的电机才每秒重试一次（避免每秒额外 3 次 sync_read 占用总线）。
        if self.last_refresh.elapsed() >= Duration::from_secs(1) && self.motor_state_incomplete() {
            self.refresh_motor_state();
        }
        self.publish_state();
    }

    fn on_poll_transport_failure(&mut self, msg: &str) {
        self.stats.read_errors.fetch_add(1, Ordering::Relaxed);
        self.transport_errors_in_row += 1;
        {
            let mut st = self.state.write().unwrap();
            st.error = Some(msg.to_string());
        }
        if self.transport_errors_in_row >= 3 {
            // 关键：立即释放串口 fd。否则下一次 open 会因为旧 handle 仍占着而永远 EBUSY。
            self.close_io();
            self.last_reconnect = Instant::now();
            self.last_poll_at = None;
            self.set_state_connected(false, Some(msg.to_string()));
            self.publish_event(false, Some(msg));
        }
    }

    /// 堵转保护：对每个电机检测，触发时**只**处理该电机（torque OFF + goal 回退 + 事件）。
    fn run_stall_protection(
        &mut self,
        present: &BTreeMap<u8, i64>,
        velocity: &BTreeMap<u8, i64>,
        now: Instant,
    ) {
        let motors = self.cfg.motors.clone();
        let excluded: HashSet<u8> = self.cfg.stall_exclude_motors.iter().copied().collect();
        let mut triggers: Vec<StallTrigger> = Vec::new();
        for id in motors {
            if excluded.contains(&id) {
                continue;
            }
            let Some(&pos) = present.get(&id) else { continue };
            if let Some(t) = self.stall_tracker.observe(
                id,
                pos,
                velocity.get(&id).copied(),
                self.goals.get(&id).copied(),
                self.torque.get(&id).copied(),
                now,
            ) {
                triggers.push(t);
            }
        }
        for t in triggers {
            self.on_stall(t);
        }
    }

    fn on_stall(&mut self, t: StallTrigger) {
        // 回退目标：当前 Present 夹到限位内侧余量（未知限位时直接用 Present）
        let revert = match self.limits.get(&t.motor).copied() {
            Some((lo, hi)) if lo < hi => {
                let margin = self.cfg.stall_recover_margin.max(0);
                let (a, b) = (lo + margin, hi - margin);
                if a <= b {
                    t.position.clamp(a, b)
                } else {
                    t.position
                }
            }
            _ => t.position,
        }
        .clamp(safety::POSITION_MIN, safety::POSITION_MAX);

        let torque_off = self.cfg.auto_torque_off;
        {
            if let Some(io) = self.link.io() {
                if torque_off {
                    // 先卸力矩（sync_write 不等 ack，最快生效），只写这一个电机
                    let _ = io.sync_write(
                        dxl::ADDR_TORQUE_ENABLE,
                        &[(t.motor, dxl::le_bytes(0, 1))],
                    );
                }
                // 再回退目标，保证之后重新使能时不会飞向旧目标
                let _ = io.sync_write(
                    dxl::ADDR_GOAL_POSITION,
                    &[(t.motor, dxl::le_bytes(revert, 4))],
                );
            }
        }
        if torque_off {
            self.torque.insert(t.motor, false);
            self.stats.torque_offs.fetch_add(1, Ordering::Relaxed);
        }
        self.goals.insert(t.motor, revert);
        self.stats.stalls.fetch_add(1, Ordering::Relaxed);
        let ev = StallEvent {
            motor: t.motor,
            reason: "stall".into(),
            position: t.position,
            goal: t.goal,
            stall_ms: t.stall_ms,
            ts_ms: now_ms(),
            torque_off,
            action: if torque_off {
                "torque_off+goal_revert".into()
            } else {
                "goal_revert".into()
            },
            reverted_goal: revert,
        };
        self.stalls.insert(t.motor, ev.clone());
        self.stall_tracker.reset(t.motor);
        self.sync_safety_state();
        self.publish_stall(&ev);
        eprintln!(
            "[armd] arm {} 堵转保护：motor {} 卡在 {}（目标 {}，持续 {}ms）→ {}",
            self.cfg.id,
            t.motor,
            t.position,
            t.goal,
            t.stall_ms,
            ev.action
        );
    }

    /// 每秒读一次电压/温度（合成一次 sync_read：144 电压 2B + 146 温度 1B = 3B）
    fn poll_health(&mut self) {
        if !self.cfg.monitor {
            return;
        }
        if self.last_monitor.elapsed().as_secs_f64() < 1.0 {
            return;
        }
        self.last_monitor = Instant::now();
        let motors = self.cfg.motors.clone();
        let res = match self.link.io() {
            Some(io) => io.sync_read(
                dxl::ADDR_PRESENT_INPUT_VOLTAGE,
                3,
                &motors,
                Duration::from_millis(50),
            ),
            None => return,
        };
        let mut health = BTreeMap::new();
        if let Ok(res) = res {
            for (id, data) in res.data {
                if data.len() >= 3 {
                    let raw_v = (data[0] as u16) | ((data[1] as u16) << 8);
                    health.insert(id, (raw_v as f64 / 10.0, data[2] as i64));
                }
            }
        }
        if !health.is_empty() {
            self.state.write().unwrap().health = health;
        }
    }

    // ------------------------------------------------------------ 写命令（带安全校验）

    fn soft_limit_reject(&self) -> bool {
        self.cfg.soft_limit_mode.eq_ignore_ascii_case("reject")
    }

    fn write_register_checked(&mut self, id: u8, addr: u16, data: Vec<u8>) -> Result<Value, IoError> {
        let mut data = data;
        let mut warnings: Vec<String> = Vec::new();
        let mut applied: Option<i64> = None;
        let mut clamped: Option<Value> = None;
        if addr == dxl::ADDR_GOAL_POSITION && data.len() >= 4 {
            let requested = dxl::le_int(&data);
            let out = safety::apply_soft_limit(
                id,
                requested,
                self.limits.get(&id).copied(),
                self.modes.get(&id).copied(),
                &self.soft_limit_cfg,
            );
            if out.clamped && self.soft_limit_reject() {
                return Err(IoError::SoftLimit {
                    motor: id,
                    value: requested,
                    min: out.min,
                    max: out.max,
                });
            }
            warnings.extend(out.warnings.iter().cloned());
            if out.clamped {
                clamped = Some(json!({
                    "requested": out.requested, "applied": out.applied,
                    "min": out.min, "max": out.max,
                }));
            }
            applied = Some(out.applied);
            data = dxl::le_bytes(out.applied, data.len());
        }
        let res = match self.link.io() {
            Some(io) => io.write_register(id, addr, &data),
            None => Err(IoError::Busy),
        };
        match res {
            Ok(()) => {
                self.stats.writes.fetch_add(1, Ordering::Relaxed);
                if let Some(applied) = applied {
                    self.goals.insert(id, applied);
                    self.clear_stall_marker(id);
                }
                if addr == dxl::ADDR_TORQUE_ENABLE && data.len() == 1 {
                    let on = dxl::le_int(&data) != 0;
                    self.torque.insert(id, on);
                    if on {
                        self.clear_stall_marker(id);
                    }
                }
                if addr == dxl::ADDR_OPERATING_MODE && data.len() == 1 {
                    self.modes.insert(id, dxl::le_int(&data) as u16);
                }
                // teleop 启动时会通过 armd 修正 EEPROM 限位：跟踪写入，保持软限位用的限位不过期
                match (addr, data.len()) {
                    (dxl::ADDR_MAX_POSITION_LIMIT, 4) => {
                        if let Some((min, max)) = dxl::position_limits(&data) {
                            self.limits.insert(id, (min, max));
                        }
                    }
                    (dxl::ADDR_MAX_POSITION_LIMIT, 2) => {
                        let max = dxl::le_int(&data);
                        let min = self.limits.get(&id).map(|v| v.0).unwrap_or(0);
                        self.limits.insert(id, (min, max));
                    }
                    (dxl::ADDR_MIN_POSITION_LIMIT, 2) => {
                        let min = dxl::le_int(&data);
                        let max = self.limits.get(&id).map(|v| v.1).unwrap_or(4095);
                        self.limits.insert(id, (min, max));
                    }
                    _ => {}
                }
                self.sync_safety_state();
                Ok(json!({"ok": true, "applied": applied, "clamped": clamped, "warnings": warnings}))
            }
            Err(e) => {
                if let IoError::Device { id, error } = &e {
                    self.mark_motor_error(*id, Some(*error));
                    self.sync_safety_state();
                }
                Err(e)
            }
        }
    }

    fn sync_write_checked(
        &mut self,
        addr: u16,
        values: Vec<(u8, Vec<u8>)>,
    ) -> Result<Value, IoError> {
        let mut warnings: Vec<String> = Vec::new();
        let mut clamped = serde_json::Map::new();
        let mut goal_ids: Vec<u8> = Vec::new();
        let mut torque_updates: BTreeMap<u8, bool> = BTreeMap::new();
        let mut values = values;
        if addr == dxl::ADDR_GOAL_POSITION {
            for (id, data) in values.iter_mut() {
                if data.len() < 4 {
                    continue;
                }
                let requested = dxl::le_int(data);
                let out = safety::apply_soft_limit(
                    *id,
                    requested,
                    self.limits.get(id).copied(),
                    self.modes.get(id).copied(),
                    &self.soft_limit_cfg,
                );
                if out.clamped && self.soft_limit_reject() {
                    return Err(IoError::SoftLimit {
                        motor: *id,
                        value: requested,
                        min: out.min,
                        max: out.max,
                    });
                }
                warnings.extend(out.warnings.iter().cloned());
                if out.clamped {
                    clamped.insert(
                        id.to_string(),
                        json!({"requested": out.requested, "applied": out.applied,
                               "min": out.min, "max": out.max}),
                    );
                }
                *data = dxl::le_bytes(out.applied, data.len());
                goal_ids.push(*id);
            }
        } else if addr == dxl::ADDR_TORQUE_ENABLE {
            for (id, data) in values.iter() {
                if data.len() == 1 {
                    torque_updates.insert(*id, dxl::le_int(data) != 0);
                }
            }
        } else if addr == dxl::ADDR_OPERATING_MODE {
            for (id, data) in values.iter() {
                if data.len() == 1 {
                    self.modes.insert(*id, dxl::le_int(data) as u16);
                }
            }
        } else if addr == dxl::ADDR_MAX_POSITION_LIMIT || addr == dxl::ADDR_MIN_POSITION_LIMIT {
            for (id, data) in values.iter() {
                let (min0, max0) = self.limits.get(id).copied().unwrap_or((0, 4095));
                if addr == dxl::ADDR_MAX_POSITION_LIMIT && data.len() >= 4 {
                    if let Some((min, max)) = dxl::position_limits(data) {
                        self.limits.insert(*id, (min, max));
                    }
                } else if addr == dxl::ADDR_MAX_POSITION_LIMIT && data.len() == 2 {
                    self.limits.insert(*id, (min0, dxl::le_int(data)));
                } else if addr == dxl::ADDR_MIN_POSITION_LIMIT && data.len() == 2 {
                    self.limits.insert(*id, (dxl::le_int(data), max0));
                }
            }
        }
        let res = match self.link.io() {
            Some(io) => io.sync_write(addr, &values),
            None => Err(IoError::Busy),
        };
        match res {
            Ok(()) => {
                self.stats.writes.fetch_add(1, Ordering::Relaxed);
                for id in goal_ids {
                    if let Some((_, data)) = values.iter().find(|(vid, _)| *vid == id) {
                        self.goals.insert(id, dxl::le_int(data));
                    }
                    self.clear_stall_marker(id);
                }
                for (id, on) in &torque_updates {
                    self.torque.insert(*id, *on);
                    if *on {
                        self.clear_stall_marker(*id);
                    }
                }
                self.sync_safety_state();
                let mut out = json!({"ok": true, "clamped": clamped, "warnings": warnings});
                if !torque_updates.is_empty() {
                    if let Value::Object(map) = &mut out {
                        map.insert(
                            "torque".into(),
                            json!(torque_updates
                                .iter()
                                .map(|(k, v)| (k.to_string(), json!(v)))
                                .collect::<serde_json::Map<String, Value>>()),
                        );
                    }
                }
                Ok(out)
            }
            Err(e) => {
                if let IoError::Device { id, error } = &e {
                    self.mark_motor_error(*id, Some(*error));
                    self.sync_safety_state();
                }
                Err(e)
            }
        }
    }

    // ------------------------------------------------------------ reboot / clear_error

    fn handle_reboot(&mut self, ids: Vec<u8>) -> Result<Value, IoError> {
        let configured = self.cfg.motors.clone();
        let mut ids: Vec<u8> = if ids.is_empty() { configured.clone() } else { ids };
        ids.retain(|id| configured.contains(id));
        ids.sort_unstable();
        ids.dedup();
        if ids.is_empty() {
            return Err(IoError::Port("reboot: motors 为空或不在配置的 motors 里".into()));
        }
        let broadcast = ids.len() == configured.len();
        let mut errors: BTreeMap<u8, String> = BTreeMap::new();
        {
            let io = self.link.io().ok_or(IoError::Busy)?;
            if broadcast {
                io.reboot(dxl::BROADCAST_ID)?;
            } else {
                for id in &ids {
                    if let Err(e) = io.reboot(*id) {
                        if e.is_transport() {
                            return Err(e);
                        }
                        errors.insert(*id, e.to_string());
                    }
                }
            }
        }
        // 舵机重启需要时间；期间轮询缺包不要误判成 no_response
        self.suppress_missing_until = Some(Instant::now() + Duration::from_millis(2500));
        std::thread::sleep(Duration::from_millis(600));

        let mut rebooted: Vec<u8> = Vec::new();
        let mut failed: BTreeMap<u8, String> = BTreeMap::new();
        let mut pending: Vec<u8> = ids.clone();
        let deadline = Instant::now() + Duration::from_millis(2500);
        while !pending.is_empty() && Instant::now() < deadline {
            let mut still: Vec<u8> = Vec::new();
            for id in pending.drain(..) {
                let r = match self.link.io() {
                    Some(io) => io.ping(id),
                    None => Err(IoError::Busy),
                };
                match r {
                    Ok(_) => rebooted.push(id),
                    Err(IoError::Device { id, error }) => {
                        failed.insert(id, format!("reboot 后仍报硬件错误 0x{error:02X}"));
                    }
                    Err(e) if e.is_transport() => return Err(e),
                    Err(_) => still.push(id),
                }
            }
            if !still.is_empty() && Instant::now() < deadline {
                std::thread::sleep(Duration::from_millis(150));
            }
            pending = still;
        }
        for id in pending {
            failed.insert(id, "reboot 后无响应".into());
        }
        for id in &rebooted {
            self.motor_errors.remove(id);
            self.stalls.remove(id);
            self.stall_tracker.reset(*id);
            self.goals.remove(id);
            self.torque.insert(*id, false); // Reboot 后 RAM 复位：torque 默认 OFF
        }
        self.refresh_motor_state();
        self.sync_safety_state();
        let torque: BTreeMap<String, bool> =
            rebooted.iter().map(|id| (id.to_string(), false)).collect();
        let limits: serde_json::Map<String, Value> = rebooted
            .iter()
            .filter_map(|id| {
                self.limits.get(id).map(|(lo, hi)| (id.to_string(), json!({"min": lo, "max": hi})))
            })
            .collect();
        eprintln!(
            "[armd] arm {} reboot：成功 {:?}，失败 {:?}",
            self.cfg.id, rebooted, failed
        );
        Ok(json!({
            "ok": rebooted.len() == ids.len(),
            "rebooted": rebooted,
            "failed": failed.keys().copied().collect::<Vec<u8>>(),
            "errors": errors,
            "failures": failed,
            "torque": torque,
            "limits": limits,
            "warnings": if failed.is_empty() { Vec::<String>::new() } else { vec!["部分电机 reboot 失败，可能仍处于 latched 错误/掉线".to_string()] },
        }))
    }

    fn handle_clear_error(&mut self, ids: Vec<u8>) -> Result<Value, IoError> {
        let configured = self.cfg.motors.clone();
        let mut ids: Vec<u8> = if ids.is_empty() { configured.clone() } else { ids };
        ids.retain(|id| configured.contains(id));
        ids.sort_unstable();
        ids.dedup();
        let mut cleared: Vec<u8> = Vec::new();
        for id in &ids {
            let had_error = self.motor_errors.remove(id).is_some();
            let had_stall = self.stalls.remove(id).is_some();
            self.stall_tracker.reset(*id);
            if had_error || had_stall {
                cleared.push(*id);
            }
        }
        let mut statuses: BTreeMap<u8, u8> = BTreeMap::new();
        let mut warnings: Vec<String> = Vec::new();
        let res = match self.link.io() {
            Some(io) => io.sync_read(
                dxl::ADDR_HARDWARE_ERROR_STATUS,
                1,
                &ids,
                Duration::from_millis(self.cfg.read_timeout_ms.max(20)),
            ),
            None => Err(IoError::Busy),
        };
        if let Ok(res) = res {
            for (id, data) in &res.data {
                statuses.insert(*id, dxl::le_int(data) as u8);
            }
            for (id, code) in &res.errors {
                statuses.entry(*id).or_insert(*code);
            }
        }
        for (id, code) in &statuses {
            if *code != 0 {
                warnings.push(format!(
                    "motor {id} 仍报告 latched 硬件错误 0x{code:02X}，armd 标记已清除但舵机需要 arm.reboot（或断电）"
                ));
            }
        }
        self.sync_safety_state();
        Ok(json!({
            "ok": true,
            "cleared": cleared,
            "hardware_error_status": statuses
                .iter().map(|(k, v)| (k.to_string(), json!(v)))
                .collect::<serde_json::Map<String, Value>>(),
            "warnings": warnings,
        }))
    }

    // ------------------------------------------------------------ 命令分发

    fn dispatch(&mut self, req: Request) {
        let reply = req.reply;
        let respond = |res: Result<Value, IoError>| {
            if let Some(tx) = reply.as_ref() {
                let _ = tx.send(res);
            }
        };
        if !self.link.is_open() {
            if let Err(e) = self.open_io() {
                respond(Err(e));
                return;
            }
        }
        match req.cmd {
            Cmd::SyncRead { addr, len, ids, timeout_ms } => {
                let res = match self.link.io() {
                    Some(io) => io.sync_read(addr, len, &ids, Duration::from_millis(timeout_ms.max(1))),
                    None => Err(IoError::Busy),
                };
                respond(res.and_then(|out| {
                    let mut positions = serde_json::Map::new();
                    for (id, data) in &out.data {
                        positions.insert(id.to_string(), json!(dxl::le_int(data)));
                    }
                    Ok(json!({
                        "positions": Value::Object(positions),
                        "errors": out.errors.iter().map(|(k, v)| (k.to_string(), json!(v)))
                            .collect::<serde_json::Map<String, Value>>(),
                    }))
                }));
            }
            Cmd::ReadRegister { id, addr, len } => {
                let res = match self.link.io() {
                    Some(io) => io.sync_read(addr, len, &[id], Duration::from_millis(self.cfg.read_timeout_ms.max(20))),
                    None => Err(IoError::Busy),
                };
                let out = res.and_then(|out| {
                    if let Some(code) = out.errors.get(&id) {
                        return Err(IoError::Device { id, error: *code });
                    }
                    let data = out.data.get(&id).cloned().unwrap_or_default();
                    Ok(json!({ "model": Value::Null, "data": data, "value": dxl::le_int(&data) }))
                });
                respond(out);
            }
            Cmd::WriteRegister { id, addr, data } => {
                let res = self.write_register_checked(id, addr, data);
                respond(res);
            }
            Cmd::SyncWrite { addr, len, values } => {
                let _ = len;
                let res = self.sync_write_checked(addr, values);
                respond(res);
            }
            Cmd::Ping { id } => {
                let res = match self.link.io() {
                    Some(io) => io.ping(id),
                    None => Err(IoError::Busy),
                };
                match res {
                    Ok(model) => respond(Ok(json!({ "model": model }))),
                    Err(e) => {
                        if let IoError::Device { id, error } = &e {
                            self.mark_motor_error(*id, Some(*error));
                        }
                        respond(Err(e));
                    }
                }
            }
            Cmd::SetRate { hz } => {
                let hz = hz.clamp(0.5, 500.0);
                self.cfg.poll_hz = hz;
                self.next_poll = Instant::now();
                respond(Ok(json!({ "ok": true, "hz": hz })));
            }
            Cmd::Reconnect => {
                // 立刻释放 fd；下一轮 ensure_connected 会马上重开（last_reconnect 已过期）
                self.close_io();
                self.last_reconnect = Instant::now() - Duration::from_secs(3600);
                self.last_poll_at = None;
                self.set_state_connected(false, Some("手动重连中".into()));
                respond(Ok(json!({ "ok": true })));
            }
            Cmd::Reboot { ids } => {
                let res = self.handle_reboot(ids);
                respond(res);
            }
            Cmd::ClearError { ids } => {
                let res = self.handle_clear_error(ids);
                respond(res);
            }
            Cmd::SimInject { kind, ids, enable } => {
                let valid = matches!(
                    kind.as_str(),
                    "stall" | "hardware_error" | "no_response" | "transport_error" | "clear"
                );
                if !valid {
                    respond(Err(IoError::Port(format!("unknown sim fault '{kind}'"))));
                    return;
                }
                let ids = if ids.is_empty() { self.cfg.motors.clone() } else { ids };
                let res = match self.link.io() {
                    Some(io) => io.inject_fault(&kind, &ids, enable),
                    None => Err(IoError::Busy),
                };
                let out = res.map(|_| {
                    json!({"ok": true, "kind": kind, "motors": ids, "enable": enable, "sim": true})
                });
                respond(out);
            }
        }
    }

    // ------------------------------------------------------------ 主循环

    fn run(
        mut self,
        cfg: ArmConfig,
        urgent_rx: Receiver<Request>,
        normal_rx: Receiver<Request>,
        stop: Arc<AtomicBool>,
    ) {
        let _ = &cfg;
        self.ensure_connected(&stop);
        loop {
            if stop.load(Ordering::Relaxed) {
                break;
            }
            // 1) 急命令优先（写/扭矩/同步写），全清
            loop {
                match urgent_rx.try_recv() {
                    Ok(req) => {
                        self.stats.urgent_queued.fetch_sub(1, Ordering::Relaxed);
                        self.dispatch(req);
                    }
                    Err(TryRecvError::Empty) => break,
                    Err(TryRecvError::Disconnected) => return,
                }
            }
            self.ensure_connected(&stop);
            // 2) 普通命令，一轮最多 16 个
            for _ in 0..16 {
                match normal_rx.try_recv() {
                    Ok(req) => {
                        self.stats.normal_queued.fetch_sub(1, Ordering::Relaxed);
                        self.dispatch(req);
                    }
                    Err(_) => break,
                }
            }
            // 3) 轮询（按 poll_hz，合并为一次 sync_read）
            if self.connected && Instant::now() >= self.next_poll {
                self.poll();
                // 漂移无关的节拍：以上一次计划点为基准推进，避免睡眠误差累积
                let interval = self.cfg.poll_interval();
                self.next_poll += interval;
                if self.next_poll < Instant::now() {
                    self.next_poll = Instant::now() + interval;
                }
            }
            // 4) 睡到下一个轮询点附近；留出 2ms 内的响应窗口给写命令。
            //    断线时也至少睡 2ms：v0.1.0 里 next_poll 永不过期会导致 100% CPU 自旋。
            let wait = self.next_poll.saturating_duration_since(Instant::now());
            let cap = Duration::from_millis(2);
            let mut sleep_for = if wait > cap { cap } else { wait };
            if !self.connected {
                sleep_for = sleep_for.max(Duration::from_millis(2));
            }
            if sleep_for > Duration::from_micros(150) {
                std::thread::sleep(sleep_for);
            } else if self.connected {
                std::hint::spin_loop();
            } else {
                std::thread::sleep(Duration::from_millis(1));
            }
        }
        // 退出前释放串口
        self.close_io();
        self.set_state_connected(false, Some("worker stopped".into()));
    }
}
