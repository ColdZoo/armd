//! 每个机械臂一个 worker 线程：独占一条串口总线 + 自己的命令/响应/发布队列。
//!
//! 队列设计（性能/实时性关键）：
//! - `urgent`（有界 64）：写寄存器、扭矩、sync_write 等"要立刻执行"的命令，优先处理
//! - `normal`（有界 256）：读快照、ping、改速率等，每轮最多消费 16 个，保证轮询不被饿死
//! - 轮询：按 `poll_hz` 合并为一次 sync_read，不排队；读到的位置进入状态缓存
//! - 发布：每个订阅者一个有界通道（容量 32），满了就丢最旧帧并计数（慢客户端不拖累总线）

use crate::config::ArmConfig;
use crate::dxl;
use crate::port::{ArmIo, DxlPort, IoError, SimPort};
use crate::profile::{joint_degrees, ProfileStore};
use serde_json::{json, Value};
use std::collections::{BTreeMap, HashSet};
use std::sync::atomic::{AtomicBool, AtomicI64, AtomicU64, Ordering};
use std::sync::mpsc::{sync_channel, Receiver, SyncSender, TryRecvError};
use std::sync::{Arc, Mutex, RwLock};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

pub fn now_ms() -> u64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_millis() as u64).unwrap_or(0)
}

#[derive(Debug, Clone, Default)]
pub struct ArmState {
    pub seq: u64,
    pub ts_ms: u64,
    pub connected: bool,
    pub joints: BTreeMap<u8, i64>,
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

        let mut worker = Worker {
            cfg: cfg.clone(),
            io: None,
            connected: false,
            rate_hz: cfg.poll_hz,
            hz: 0.0,
            last_monitor: Instant::now() - Duration::from_secs(3600),
            polls_in_window: 0,
            window_start: Instant::now(),
            next_poll: Instant::now(),
            last_poll_at: None,
            last_reconnect: Instant::now() - Duration::from_secs(3600),
            errors_in_row: 0,
            state: state.clone(),
            subs: subs.clone(),
            stats: stats.clone(),
            profile: profile.clone(),
        };

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
    io: Option<Box<dyn ArmIo>>,
    connected: bool,
    rate_hz: f64,
    hz: f64,
    last_monitor: Instant,
    polls_in_window: u32,
    window_start: Instant,
    next_poll: Instant,
    last_poll_at: Option<Instant>,
    last_reconnect: Instant,
    errors_in_row: u32,
    state: Arc<RwLock<ArmState>>,
    subs: Arc<Mutex<Vec<Arc<Sub>>>>,
    stats: Arc<Stats>,
    profile: Arc<Mutex<Option<Value>>>,
}

impl Worker {
    fn open_io(&mut self) -> Result<(), IoError> {
        if self.cfg.sim {
            self.io = Some(Box::new(SimPort::new(&self.cfg.motors)));
            return Ok(());
        }
        let p = DxlPort::open(&self.cfg.port, self.cfg.baud)?;
        self.io = Some(Box::new(p));
        Ok(())
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
            Ok(()) => {
                // 握手：ping 所有电机，确认总线活着
                let io = self.io.as_mut().unwrap();
                let mut ok = 0;
                for id in self.cfg.motors.clone() {
                    if io.ping(id).is_ok() {
                        ok += 1;
                    }
                }
                if ok == self.cfg.motors.len() {
                    self.connected = true;
                    self.errors_in_row = 0;
                    self.last_poll_at = None;
                    self.next_poll = Instant::now();
                    self.stats.reconnects.fetch_add(1, Ordering::Relaxed);
                    self.set_state_connected(true, None);
                    self.publish_event(true, None);
                } else {
                    self.connected = false;
                    self.stats.read_errors.fetch_add(1, Ordering::Relaxed);
                    self.set_state_connected(false, Some(format!("握手失败：{ok}/{} 个舵机响应", self.cfg.motors.len())));
                    self.publish_event(false, None);
                }
            }
            Err(e) => {
                self.connected = false;
                self.set_state_connected(false, Some(e.to_string()));
                self.publish_event(false, Some(&e.to_string()));
            }
        }
    }

    fn set_state_connected(&self, connected: bool, error: Option<String>) {
        let mut st = self.state.write().unwrap();
        st.connected = connected;
        st.error = error;
        st.ts_ms = now_ms();
    }

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

    /// 每秒读一次电压/温度（两个寄存器一次 sync_read：144 电压 2B + 146 温度 1B = 3B）
    fn poll_health(&mut self) {
        if !self.cfg.monitor {
            return;
        }
        if self.last_monitor.elapsed().as_secs_f64() < 1.0 {
            return;
        }
        self.last_monitor = Instant::now();
        let io = match self.io.as_mut() {
            Some(io) => io,
            None => return,
        };
        let mut health = BTreeMap::new();
        if let Ok(map) = io.sync_read(144, 3, &self.cfg.motors, Duration::from_millis(50)) {
            for (id, data) in map {
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

    fn poll(&mut self) {
        let t0 = Instant::now();
        let result = match self.io.as_mut() {
            Some(io) => io.sync_read(
                self.cfg.poll_addr(),
                self.cfg.poll_len,
                &self.cfg.motors,
                Duration::from_millis(self.cfg.read_timeout_ms),
            ),
            None => Err(IoError::Busy),
        };
        let elapsed_us = t0.elapsed().as_micros() as u64;
        match result {
            Ok(map) => {
                self.errors_in_row = 0;
                self.stats.polls.fetch_add(1, Ordering::Relaxed);
                let mut joints = BTreeMap::new();
                for (id, data) in map {
                    joints.insert(id, dxl::le_int(&data));
                }
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
                let profile = self.profile.lock().unwrap().clone();
                {
                    let mut st = self.state.write().unwrap();
                    st.seq += 1;
                    st.ts_ms = now_ms();
                    st.connected = true;
                    st.joints = joints.clone();
                    st.hz = self.hz;
                    st.read_us = elapsed_us;
                    st.error = None;
                    st.joint_names = self.cfg.joint_names.clone();
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
                self.publish_state();
            }
            Err(e) => {
                self.stats.read_errors.fetch_add(1, Ordering::Relaxed);
                self.errors_in_row += 1;
                {
                    let mut st = self.state.write().unwrap();
                    st.error = Some(e.to_string());
                }
                if self.errors_in_row >= 3 && self.connected {
                    self.connected = false;
                    self.last_reconnect = Instant::now() - Duration::from_secs(3600);
                    self.last_poll_at = None;
                    self.set_state_connected(false, Some(e.to_string()));
                    self.publish_event(false, Some(&e.to_string()));
                }
            }
        }
    }

    fn dispatch(&mut self, req: Request) {
        let reply = req.reply;
        let respond = |res: Result<Value, IoError>| {
            if let Some(tx) = reply.as_ref() {
                let _ = tx.send(res);
            }
        };
        if self.io.is_none() {
            if let Err(e) = self.open_io() {
                respond(Err(e));
                return;
            }
        }
        let io = self.io.as_mut().unwrap();
        match req.cmd {
            Cmd::SyncRead { addr, len, ids, timeout_ms } => {
                let res = io.sync_read(addr, len, &ids, Duration::from_millis(timeout_ms.max(1)));
                respond(res.map(|map| {
                    let mut out = serde_json::Map::new();
                    for (id, data) in map {
                        out.insert(id.to_string(), json!(dxl::le_int(&data)));
                    }
                    json!({ "positions": Value::Object(out) })
                }));
            }
            Cmd::ReadRegister { id, addr, len } => {
                let res = io.sync_read(addr, len, &[id], Duration::from_millis(self.cfg.read_timeout_ms));
                respond(res.map(|map| {
                    let data = map.get(&id).cloned().unwrap_or_default();
                    json!({ "model": Value::Null, "data": data, "value": dxl::le_int(&data) })
                }));
            }
            Cmd::WriteRegister { id, addr, data } => {
                let res = io.write_register(id, addr, &data);
                if res.is_ok() {
                    self.stats.writes.fetch_add(1, Ordering::Relaxed);
                }
                respond(res.map(|_| json!({ "ok": true })));
            }
            Cmd::SyncWrite { addr, len, values } => {
                let res = io.sync_write(addr, &values);
                if res.is_ok() {
                    self.stats.writes.fetch_add(1, Ordering::Relaxed);
                }
                let _ = len;
                respond(res.map(|_| json!({ "ok": true })));
            }
            Cmd::Ping { id } => {
                let res = io.ping(id);
                respond(res.map(|model| json!({ "model": model })));
            }
            Cmd::SetRate { hz } => {
                let hz = hz.clamp(0.5, 500.0);
                self.cfg.poll_hz = hz;
                self.next_poll = Instant::now();
                respond(Ok(json!({ "ok": true, "hz": hz })));
            }
            Cmd::Reconnect => {
                self.connected = false;
                self.last_reconnect = Instant::now() - Duration::from_secs(3600);
                self.io = None;
                respond(Ok(json!({ "ok": true })));
            }
        }
    }

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
            // 4) 睡到下一个轮询点附近；留出 2ms 内的响应窗口给写命令
            let wait = self.next_poll.saturating_duration_since(Instant::now());
            let cap = Duration::from_millis(2);
            let sleep_for = if wait > cap { cap } else { wait };
            if sleep_for > Duration::from_micros(150) {
                std::thread::sleep(sleep_for);
            } else {
                std::hint::spin_loop();
            }
        }
    }
}
