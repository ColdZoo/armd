//! Unix socket RPC 服务：NDJSON 请求/响应 + 发布订阅流。
//!
//! 协议（换行分隔 JSON）：
//! - 请求：`{"id":1,"method":"arm.state","params":{"arm":"koch_v1_1:follower"}}`
//! - 响应：`{"id":1,"result":{...}}` 或 `{"id":1,"error":{"code":"...","message":"..."}}`
//! - 推送：`{"event":"state","sub":3,"arm":"...","seq":...,"joints":{...}}`
//!          `{"event":"arm","sub":3,"arm":"...","connected":...}`

use crate::arm::{now_ms, ArmHandle, Cmd, Request, Sub};
use crate::config::Config;
use crate::dxl;
use crate::port::IoError;
use crate::profile::{joint_degrees, validate as validate_profile};
use serde_json::{json, Value};
use std::collections::{HashMap, HashSet};
use std::io::{BufRead, BufReader, Write};
use std::net::TcpListener;
use std::os::unix::fs::PermissionsExt;
use std::os::unix::net::UnixListener;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{sync_channel, SyncSender};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// 一条客户端连接：Unix socket 或 TCP，对上层无差别。
pub trait Conn: std::io::Read + std::io::Write + Send + 'static {
    fn try_clone_conn(&self) -> std::io::Result<Box<dyn Conn>>;
}

impl Conn for std::os::unix::net::UnixStream {
    fn try_clone_conn(&self) -> std::io::Result<Box<dyn Conn>> {
        Ok(Box::new(self.try_clone()?))
    }
}

impl Conn for std::net::TcpStream {
    fn try_clone_conn(&self) -> std::io::Result<Box<dyn Conn>> {
        Ok(Box::new(self.try_clone()?))
    }
}

#[derive(Debug)]
struct RpcError {
    code: String,
    message: String,
}

impl RpcError {
    fn new(code: &str, message: impl Into<String>) -> Self {
        Self { code: code.into(), message: message.into() }
    }
    fn to_json(&self) -> Value {
        json!({ "code": self.code, "message": self.message })
    }
}

type RpcResult = Result<Value, RpcError>;

pub struct Daemon {
    pub socket_path: String,
    listen_specs: Vec<String>,
    token: Option<String>,
    identity_path: String,
    started: Instant,
    arms: HashMap<String, Arc<ArmHandle>>,
    subs: Mutex<HashMap<u64, Arc<Sub>>>,
    next_sub: AtomicU64,
    stop: AtomicBool,
}

impl Daemon {
    pub fn new(cfg: &Config) -> Self {
        let mut arms = HashMap::new();
        for arm_cfg in &cfg.arms {
            let handle = ArmHandle::spawn(arm_cfg.clone());
            arms.insert(arm_cfg.id.clone(), handle);
        }
        Self {
            socket_path: cfg.socket.clone(),
            listen_specs: cfg.listen_specs(),
            token: cfg.token.clone(),
            identity_path: cfg.identity.clone().unwrap_or_else(default_identity_path),
            started: Instant::now(),
            arms,
            subs: Mutex::new(HashMap::new()),
            next_sub: AtomicU64::new(1),
            stop: AtomicBool::new(false),
        }
    }

    pub fn arm_ids(&self) -> Vec<String> {
        let mut ids: Vec<String> = self.arms.keys().cloned().collect();
        ids.sort();
        ids
    }

    fn arm(&self, id: &str) -> Result<Arc<ArmHandle>, RpcError> {
        self.arms.get(id).cloned().ok_or_else(|| {
            RpcError::new("no_such_arm", format!("unknown arm '{id}'，可用：{:?}", self.arm_ids()))
        })
    }

    /// 启动所有监听（阻塞）。监听地址来自配置 `listen`（unix:<path> / tcp://host:port）。
    ///
    /// 借鉴 robotd（pollen-robotics/microduck）的做法：
    /// - 先把所有 listener **同步 bind 成功**再起线程，端口/路径被占用时立即失败；
    /// - Unix socket 已存在时先探测：活着就拒绝启动，死了才清掉（绝不抢占别的 daemon）；
    /// - 写 identity 文件（版本/pid/监听地址/机械臂），方便 `robotctl` 之类工具确认"谁在跑"。
    pub fn serve(self: Arc<Self>) -> std::io::Result<()> {
        let mut unix_listeners = Vec::new();
        let mut tcp_listeners = Vec::new();
        for spec in self.listen_specs.clone() {
            if let Some(path) = spec.strip_prefix("unix:") {
                if std::path::Path::new(path).exists() {
                    match std::os::unix::net::UnixStream::connect(path) {
                        Ok(_) => {
                            return Err(std::io::Error::new(
                                std::io::ErrorKind::AddrInUse,
                                format!("{path} 已经被另一个 armd 监听，拒绝抢占"),
                            ));
                        }
                        Err(_) => {
                            eprintln!("[armd] 清理上次崩溃遗留的 socket: {path}");
                            let _ = std::fs::remove_file(path);
                        }
                    }
                }
                let listener = UnixListener::bind(path)?;
                std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o600))?;
                eprintln!("[armd] listening on unix:{path} (arms: {:?})", self.arm_ids());
                unix_listeners.push(listener);
            } else if let Some(addr) = spec.strip_prefix("tcp://") {
                let listener = TcpListener::bind(addr)?;
                eprintln!("[armd] listening on tcp://{addr} (arms: {:?})", self.arm_ids());
                tcp_listeners.push(listener);
            } else {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::InvalidInput,
                    format!("非法监听地址 '{spec}'（支持 unix:<path> / tcp://host:port）"),
                ));
            }
        }
        if unix_listeners.is_empty() && tcp_listeners.is_empty() {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                "没有配置监听地址（listen / socket）",
            ));
        }
        if let Err(e) = self.write_identity() {
            eprintln!("[armd] identity 写入失败（不影响服务）: {e}");
        }

        let mut threads = Vec::new();
        for listener in unix_listeners {
            let daemon = self.clone();
            threads.push(std::thread::spawn(move || {
                let mut conn_id = 0u64;
                for stream in listener.incoming() {
                    if daemon.stop.load(Ordering::Relaxed) {
                        break;
                    }
                    match stream {
                        Ok(s) => {
                            conn_id += 1;
                            daemon.spawn_conn(Box::new(s), conn_id);
                        }
                        Err(e) => eprintln!("[armd] accept error (unix): {e}"),
                    }
                }
            }));
        }
        for listener in tcp_listeners {
            let daemon = self.clone();
            threads.push(std::thread::spawn(move || {
                let mut conn_id = 0u64;
                for stream in listener.incoming() {
                    if daemon.stop.load(Ordering::Relaxed) {
                        break;
                    }
                    match stream {
                        Ok(s) => {
                            let _ = s.set_nodelay(true);
                            conn_id += 1;
                            daemon.spawn_conn(Box::new(s), conn_id);
                        }
                        Err(e) => eprintln!("[armd] accept error (tcp): {e}"),
                    }
                }
            }));
        }
        for t in threads {
            let _ = t.join();
        }
        Ok(())
    }

    fn identity_json(&self) -> serde_json::Value {
        json!({
            "service": "armd",
            "version": VERSION,
            "pid": std::process::id(),
            "started_ms": now_ms(),
            "listen": self.listen_specs,
            "token_required": self.token.is_some(),
            "arms": self.arm_ids(),
        })
    }

    fn write_identity(&self) -> std::io::Result<()> {
        let body = self.identity_json().to_string();
        if let Some(parent) = std::path::Path::new(&self.identity_path).parent() {
            let _ = std::fs::create_dir_all(parent);
        }
        std::fs::write(&self.identity_path, body)
    }

    fn spawn_conn(self: &Arc<Self>, stream: Box<dyn Conn>, conn_id: u64) {
        let daemon = self.clone();
        std::thread::Builder::new()
            .name(format!("conn-{conn_id}"))
            .spawn(move || daemon.handle_conn(stream, conn_id))
            .ok();
    }

    fn handle_conn(&self, stream: Box<dyn Conn>, _conn_id: u64) {
        let writer = match stream.try_clone_conn() {
            Ok(s) => s,
            Err(_) => return,
        };
        // 单一写线程：响应和推送都走这条通道，保证不交织
        let (out_tx, out_rx): (SyncSender<String>, _) = sync_channel(256);
        let writer_thread = std::thread::spawn(move || {
            let mut w = std::io::BufWriter::new(writer);
            while let Ok(line) = out_rx.recv() {
                if w.write_all(line.as_bytes()).is_err()
                    || w.write_all(b"\n").is_err()
                    || w.flush().is_err()
                {
                    break;
                }
            }
        });

        let (frame_tx, frame_rx): (SyncSender<String>, _) = sync_channel(32);
        let out_for_frames = out_tx.clone();
        std::thread::spawn(move || {
            while let Ok(frame) = frame_rx.recv() {
                if out_for_frames.try_send(frame).is_err() {
                    // 客户端读得慢：直接丢推送，不影响总线
                }
            }
        });

        let mut conn_subs: Vec<u64> = Vec::new();
        let mut authed = self.token.is_none();
        let reader = BufReader::new(stream);
        for line in reader.lines() {
            let line = match line {
                Ok(l) => l,
                Err(_) => break,
            };
            if line.trim().is_empty() {
                continue;
            }
            let req: Value = match serde_json::from_str(&line) {
                Ok(v) => v,
                Err(e) => {
                    let _ = out_tx.try_send(json!({
                        "id": Value::Null,
                        "error": {"code":"bad_request","message": format!("invalid json: {e}")}
                    }).to_string());
                    continue;
                }
            };
            let id = req.get("id").cloned().unwrap_or(Value::Null);
            let is_jsonrpc = req.get("jsonrpc").is_some();
            let method = req.get("method").and_then(|m| m.as_str()).unwrap_or("").to_string();
            let params = req.get("params").cloned().unwrap_or_else(|| json!({}));
            let result = self.dispatch(&method, &params, &frame_tx, &mut conn_subs, &mut authed);
            let mut resp = match result {
                Ok(v) => json!({ "id": id, "result": v }),
                Err(e) => json!({ "id": id, "error": e.to_json() }),
            };
            if is_jsonrpc {
                if let Value::Object(map) = &mut resp {
                    map.insert("jsonrpc".into(), json!("2.0"));
                }
            }
            if out_tx.send(resp.to_string()).is_err() {
                break;
            }
        }

        // 连接断开：关闭它创建的所有订阅
        for sub_id in conn_subs {
            self.drop_sub(sub_id);
        }
        drop(out_tx);
        drop(writer_thread);
    }

    fn drop_sub(&self, sub_id: u64) {
        let sub = self.subs.lock().unwrap().remove(&sub_id);
        if let Some(sub) = sub {
            sub.closed.store(true, Ordering::Relaxed);
            for arm in self.arms.values() {
                arm.subs.lock().unwrap().retain(|s| s.id != sub_id);
            }
        }
    }

    fn dispatch(
        &self,
        method: &str,
        params: &Value,
        frame_tx: &SyncSender<String>,
        conn_subs: &mut Vec<u64>,
        authed: &mut bool,
    ) -> RpcResult {
        match method {
            // 握手：config.token 设置时必须先 hello
            "hello" => {
                let provided = params.get("token").and_then(|v| v.as_str());
                match &self.token {
                    None => {
                        *authed = true;
                        Ok(json!({ "ok": true, "auth_required": false, "version": VERSION }))
                    }
                    Some(expected) if provided == Some(expected.as_str()) => {
                        *authed = true;
                        Ok(json!({ "ok": true, "auth_required": true, "version": VERSION }))
                    }
                    Some(_) => Err(RpcError::new("unauthorized", "token 不正确")),
                }
            }
            _ if !*authed => Err(RpcError::new("unauthorized", "需要先发送 hello（已配置 token）")),
            "ping" => Ok(json!({
                "pong": true, "version": VERSION,
                "uptime_sec": self.started.elapsed().as_secs(),
            })),
            "info" => Ok(json!({
                "version": VERSION,
                "uptime_sec": self.started.elapsed().as_secs(),
                "socket": self.socket_path,
                "arms": self.arm_ids(),
            })),
            "arm.list" => Ok(Value::Array(self.list_arms())),
            "arm.state" => {
                let arm = self.arm(&str_param(params, "arm")?)?;
                let mut v = arm.state.read().unwrap().to_json();
                if let Value::Object(map) = &mut v {
                    map.insert("arm".into(), json!(arm.cfg.id));
                    map.insert("port".into(), json!(arm.cfg.port));
                    map.insert("sim".into(), json!(arm.cfg.sim));
                    map.insert("poll_addr".into(), json!(arm.cfg.poll_addr()));
                    map.insert("motors".into(), json!(arm.cfg.motors));
                }
                Ok(v)
            }
            "arm.profile.get" => {
                let arm = self.arm(&str_param(params, "arm")?)?;
                let profile = arm.profile.lock().unwrap().clone();
                Ok(json!({
                    "profile": profile,
                    "path": arm.profile_store.path().to_string_lossy(),
                    "has_profile": profile.is_some(),
                }))
            }
            "arm.profile.set" => {
                let arm = self.arm(&str_param(params, "arm")?)?;
                let payload = params
                    .get("profile")
                    .ok_or_else(|| RpcError::new("bad_request", "缺少 profile 对象"))?;
                let clean = validate_profile(payload, &arm.cfg.joint_names)
                    .map_err(|e| RpcError::new("bad_request", e))?;
                arm.profile_store
                    .save(&clean)
                    .map_err(|e| RpcError::new("io_error", e))?;
                *arm.profile.lock().unwrap() = Some(clean.clone());
                {
                    let mut st = arm.state.write().unwrap();
                    if !st.joints.is_empty() {
                        st.deg = joint_degrees(&st.joints, &arm.cfg.joint_names, &clean);
                    }
                    st.profile_saved_at = clean.get("savedAt").and_then(|v| v.as_i64());
                }
                Ok(json!({ "profile": clean, "path": arm.profile_store.path().to_string_lossy() }))
            }
            "arm.profile.history" => {
                let arm = self.arm(&str_param(params, "arm")?)?;
                let items: Vec<Value> = arm
                    .profile_store
                    .history()
                    .into_iter()
                    .map(|(name, saved_at)| json!({"name": name, "savedAt": saved_at}))
                    .collect();
                Ok(json!({"history": items, "dir": arm.profile_store.history_dir().to_string_lossy()}))
            }
            "arm.profile.restore" => {
                let arm = self.arm(&str_param(params, "arm")?)?;
                let name = str_param(params, "name")?;
                let payload = arm
                    .profile_store
                    .history_file(&name)
                    .map_err(|e| RpcError::new("bad_request", e))?;
                let clean = validate_profile(&payload, &arm.cfg.joint_names)
                    .map_err(|e| RpcError::new("bad_request", e))?;
                arm.profile_store
                    .save(&clean)
                    .map_err(|e| RpcError::new("io_error", e))?;
                *arm.profile.lock().unwrap() = Some(clean.clone());
                {
                    let mut st = arm.state.write().unwrap();
                    if !st.joints.is_empty() {
                        st.deg = joint_degrees(&st.joints, &arm.cfg.joint_names, &clean);
                    }
                    st.profile_saved_at = clean.get("savedAt").and_then(|v| v.as_i64());
                }
                Ok(json!({"restored": name, "profile": clean}))
            }
            "arm.profile.delete" => {
                let arm = self.arm(&str_param(params, "arm")?)?;
                let deleted = arm.profile_store.delete();
                *arm.profile.lock().unwrap() = None;
                {
                    let mut st = arm.state.write().unwrap();
                    st.deg.clear();
                    st.profile_saved_at = None;
                }
                Ok(json!({ "deleted": deleted }))
            }
            "arm.calibration.snapshot" => {
                let arm = self.arm(&str_param(params, "arm")?)?;
                let st = arm.state.read().unwrap();
                let names = &arm.cfg.joint_names;
                let raw: serde_json::Map<String, Value> = st
                    .joints
                    .iter()
                    .map(|(id, v)| {
                        let name = names.get(&id.to_string()).cloned().unwrap_or_else(|| id.to_string());
                        (name, json!(v))
                    })
                    .collect();
                Ok(json!({
                    "raw": raw,
                    "deg": st.deg,
                    "seq": st.seq,
                    "ts": st.ts_ms,
                    "connected": st.connected,
                    "health": st.health.iter().map(|(id, (v, t))| {
                        (id.to_string(), json!({"voltage": v, "temp": t}))
                    }).collect::<serde_json::Map<String, Value>>(),
                }))
            }
            "arm.read" => {
                let arm = self.arm(&str_param(params, "arm")?)?;
                let ids = optional_motors(params).unwrap_or_else(|| arm.cfg.motors.clone());
                let timeout = params.get("timeout_ms").and_then(|v| v.as_u64()).unwrap_or(50);
                call_arm(
                    &arm,
                    Cmd::SyncRead {
                        addr: dxl::ADDR_PRESENT_POSITION,
                        len: 4,
                        ids,
                        timeout_ms: timeout,
                    },
                    false,
                    1000,
                )
            }
            "arm.read_register" => {
                let arm = self.arm(&str_param(params, "arm")?)?;
                let motor = u8_param(params, "motor")?;
                let addr = u16_param(params, "address")?;
                let len = u16_param(params, "length")?;
                call_arm(&arm, Cmd::ReadRegister { id: motor, addr, len }, false, 1000)
            }
            "arm.write_register" => {
                let arm = self.arm(&str_param(params, "arm")?)?;
                let motor = u8_param(params, "motor")?;
                let addr = u16_param(params, "address")?;
                let len = u16_param(params, "length")? as usize;
                let value = params
                    .get("value")
                    .and_then(|v| v.as_i64())
                    .ok_or_else(|| RpcError::new("bad_request", "value 必须是整数"))?;
                let data = dxl::le_bytes(value, len);
                call_arm(&arm, Cmd::WriteRegister { id: motor, addr, data }, true, 1000)
            }
            "arm.sync_write" => {
                let arm = self.arm(&str_param(params, "arm")?)?;
                let addr = u16_param(params, "address")?;
                let len = u16_param(params, "length")? as usize;
                let values = params
                    .get("values")
                    .and_then(|v| v.as_object())
                    .ok_or_else(|| RpcError::new("bad_request", "values 必须是 {motor: value} 对象"))?;
                let mut vals = Vec::with_capacity(values.len());
                for (k, v) in values {
                    let id: u8 = k.parse().map_err(|_| RpcError::new("bad_request", format!("非法 motor id '{k}'")))?;
                    let value = v.as_i64().ok_or_else(|| RpcError::new("bad_request", format!("motor {k} 的值必须是整数")))?;
                    vals.push((id, dxl::le_bytes(value, len)));
                }
                call_arm(&arm, Cmd::SyncWrite { addr, len: len as u16, values: vals }, true, 1000)
            }
            "arm.torque" => {
                let arm = self.arm(&str_param(params, "arm")?)?;
                let enable = params.get("enable").and_then(|v| v.as_bool()).unwrap_or(true);
                let ids = optional_motors(params).unwrap_or_else(|| arm.cfg.motors.clone());
                let value = if enable { 1 } else { 0 };
                let vals: Vec<(u8, Vec<u8>)> = ids.into_iter().map(|id| (id, dxl::le_bytes(value, 1))).collect();
                call_arm(
                    &arm,
                    Cmd::SyncWrite { addr: dxl::ADDR_TORQUE_ENABLE, len: 1, values: vals },
                    true,
                    1000,
                )
            }
            "arm.torque_off" => {
                // 便捷方法：等价 arm.torque enable=false（逐电机 torque OFF）
                let arm = self.arm(&str_param(params, "arm")?)?;
                let ids = optional_motors(params).unwrap_or_else(|| arm.cfg.motors.clone());
                let vals: Vec<(u8, Vec<u8>)> =
                    ids.into_iter().map(|id| (id, dxl::le_bytes(0, 1))).collect();
                call_arm(
                    &arm,
                    Cmd::SyncWrite { addr: dxl::ADDR_TORQUE_ENABLE, len: 1, values: vals },
                    true,
                    1000,
                )
            }
            "arm.reboot" => {
                // Dynamixel Reboot(0x08)：清除 latched 硬件错误；worker 内等 ~600ms + ping，最长 5s
                let arm = self.arm(&str_param(params, "arm")?)?;
                let ids = optional_motors(params).unwrap_or_else(|| arm.cfg.motors.clone());
                call_arm(&arm, Cmd::Reboot { ids }, true, 5000)
            }
            "arm.clear_error" => {
                // 只清 armd 侧标记 + 读 Hardware_Error_Status(70)；latched 错误仍需 reboot
                let arm = self.arm(&str_param(params, "arm")?)?;
                let ids = optional_motors(params).unwrap_or_else(|| arm.cfg.motors.clone());
                call_arm(&arm, Cmd::ClearError { ids }, true, 2000)
            }
            "arm.sim.inject" => {
                // 仅仿真臂：注入/清除故障
                // （stall / hardware_error / no_response / transport_error / clear）
                let arm = self.arm(&str_param(params, "arm")?)?;
                if !arm.cfg.sim {
                    return Err(RpcError::new(
                        "not_sim",
                        format!(
                            "机械臂 '{}' 不是仿真臂（sim=false），不支持故障注入",
                            arm.cfg.id
                        ),
                    ));
                }
                let kind = str_param(params, "kind")?;
                if !matches!(
                    kind.as_str(),
                    "stall" | "hardware_error" | "no_response" | "transport_error" | "clear"
                ) {
                    return Err(RpcError::new(
                        "bad_request",
                        format!(
                            "kind 必须是 stall/hardware_error/no_response/transport_error/clear，收到 '{kind}'"
                        ),
                    ));
                }
                let enable = params.get("enable").and_then(|v| v.as_bool()).unwrap_or(true);
                let ids = optional_motors(params).unwrap_or_else(|| arm.cfg.motors.clone());
                call_arm(&arm, Cmd::SimInject { kind, ids, enable }, true, 1000)
            }
            "arm.ping" => {
                let arm = self.arm(&str_param(params, "arm")?)?;
                let motor = u8_param(params, "motor")?;
                call_arm(&arm, Cmd::Ping { id: motor }, false, 1000)
            }
            "arm.reconnect" => {
                let arm = self.arm(&str_param(params, "arm")?)?;
                call_arm(&arm, Cmd::Reconnect, true, 1000)
            }
            "arm.set_rate" => {
                let arm = self.arm(&str_param(params, "arm")?)?;
                let hz = params
                    .get("hz")
                    .and_then(|v| v.as_f64())
                    .ok_or_else(|| RpcError::new("bad_request", "hz 必须是数字"))?;
                if !(0.5..=500.0).contains(&hz) {
                    return Err(RpcError::new("bad_request", "hz 需在 0.5~500 之间"));
                }
                call_arm(&arm, Cmd::SetRate { hz }, true, 500)
            }
            "arm.stats" => {
                let arm = self.arm(&str_param(params, "arm")?)?;
                Ok(arm.stats.to_json())
            }
            "sub.subscribe" => {
                let arms = match params.get("arms") {
                    Some(Value::Array(arr)) => arr
                        .iter()
                        .filter_map(|v| v.as_str().map(|s| s.to_string()))
                        .collect::<Vec<_>>(),
                    _ => self.arm_ids(),
                };
                let rate_hz = params.get("rate_hz").and_then(|v| v.as_f64()).unwrap_or(60.0);
                let sub_id = self.next_sub.fetch_add(1, Ordering::Relaxed);
                let sub = Arc::new(Sub::new(
                    sub_id,
                    arms.iter().cloned().collect::<HashSet<_>>(),
                    rate_hz,
                    frame_tx.clone(),
                ));
                self.subs.lock().unwrap().insert(sub_id, sub.clone());
                for arm_id in &arms {
                    if let Ok(arm) = self.arm(arm_id) {
                        arm.subs.lock().unwrap().push(sub.clone());
                    }
                }
                conn_subs.push(sub_id);
                Ok(json!({ "sub_id": sub_id, "rate_hz": rate_hz, "arms": arms }))
            }
            "sub.unsubscribe" => {
                let sub_id = params
                    .get("sub_id")
                    .and_then(|v| v.as_u64())
                    .ok_or_else(|| RpcError::new("bad_request", "sub_id 必填"))?;
                self.drop_sub(sub_id);
                conn_subs.retain(|id| *id != sub_id);
                Ok(json!({ "ok": true }))
            }
            "sub.rate" => {
                let sub_id = params
                    .get("sub_id")
                    .and_then(|v| v.as_u64())
                    .ok_or_else(|| RpcError::new("bad_request", "sub_id 必填"))?;
                let rate_hz = params
                    .get("rate_hz")
                    .and_then(|v| v.as_f64())
                    .ok_or_else(|| RpcError::new("bad_request", "rate_hz 必填"))?;
                let sub = self.subs.lock().unwrap().get(&sub_id).cloned();
                match sub {
                    Some(s) => {
                        s.set_rate(rate_hz);
                        Ok(json!({ "ok": true }))
                    }
                    None => Err(RpcError::new("no_such_sub", "订阅不存在")),
                }
            }
            _ => Err(RpcError::new("unknown_method", format!("未知方法 '{method}'"))),
        }
    }

    fn list_arms(&self) -> Vec<Value> {
        let mut out = Vec::new();
        for (id, arm) in &self.arms {
            let st = arm.state.read().unwrap();
            // 安全层摘要与 arm.state 同源：直接取 state JSON 里的 motor_errors/stalls 对象
            let state_json = st.to_json();
            let motor_errors =
                state_json.get("motor_errors").cloned().unwrap_or_else(|| json!({}));
            let stalls = state_json.get("stalls").cloned().unwrap_or_else(|| json!({}));
            out.push(json!({
                "id": id,
                "label": arm.cfg.label,
                "port": arm.cfg.port,
                "sim": arm.cfg.sim,
                "motors": arm.cfg.motors,
                "connected": st.connected,
                "hz": (st.hz * 10.0).round() / 10.0,
                "seq": st.seq,
                "error": st.error,
                "profile_saved_at": st.profile_saved_at,
                "motor_errors": motor_errors,
                "stalls": stalls,
                "stats": arm.stats.to_json(),
            }));
        }
        out.sort_by(|a, b| a["id"].as_str().unwrap_or("").cmp(b["id"].as_str().unwrap_or("")));
        out
    }
}

/// 单实例锁：`<socket>.lock` 上的 flock（File::try_lock）。
///
/// 借鉴 robotd：串口独占（serialport 已设 TIOCEXCL）只挡住非 root 的二次打开，
/// 挡不住"另一个 armd 用 root 启动"。所以 daemon 必须在打开串口前先拿到锁。
/// 锁文件**永不删除**（删掉会留下两个不同 inode 的竞争者），进程退出内核自动释放。
pub fn acquire_instance_lock(socket: &str) -> std::io::Result<std::fs::File> {
    let path = format!("{socket}.lock");
    let file = std::fs::OpenOptions::new()
        .create(true)
        .read(true)
        .write(true)
        .open(&path)?;
    file.try_lock().map_err(|e| {
        std::io::Error::new(
            std::io::ErrorKind::AddrInUse,
            format!("另一个 armd 已经持有 {path}（若确认没有在跑，检查进程）: {e}"),
        )
    })?;
    Ok(file)
}

fn default_identity_path() -> String {
    // Linux（ARM 板）优先 /run/armd，macOS 退到 /tmp
    let run_dir = std::path::Path::new("/run/armd");
    if std::fs::metadata("/run").map(|m| m.is_dir()).unwrap_or(false)
        && std::fs::create_dir_all(run_dir).is_ok()
        && std::fs::metadata(run_dir).map(|m| m.permissions().readonly()).unwrap_or(true) == false
    {
        return "/run/armd/identity.json".to_string();
    }
    "/tmp/armd-identity.json".to_string()
}

fn call_arm(arm: &Arc<ArmHandle>, cmd: Cmd, urgent: bool, wait_ms: u64) -> RpcResult {
    let (tx, rx) = sync_channel(1);
    arm.submit(Request { cmd, reply: Some(tx) }, urgent)
        .map_err(|q| RpcError::new("queue_full", format!("{q} 队列已满")))?;
    match rx.recv_timeout(Duration::from_millis(wait_ms)) {
        Ok(Ok(v)) => Ok(v),
        Ok(Err(e)) => Err(io_error_to_rpc(e)),
        Err(_) => Err(RpcError::new("timeout", "机械臂 worker 响应超时")),
    }
}

/// worker 返回的 IO/安全层错误 → RPC 错误码。
///
/// - `SoftLimit`（`soft_limit_mode="reject"` 时写 Goal 越界）→ `soft_limit`
/// - `Unsupported`（真机臂不支持的操作，如 `arm.sim.inject`）→ `not_sim`
/// - `Device`（舵机硬件 error 位）→ `serial_error`（保持 v0.1.0 语义）
/// - 其余传输层错误 → `serial_error`
fn io_error_to_rpc(e: IoError) -> RpcError {
    match &e {
        IoError::SoftLimit { .. } => RpcError::new("soft_limit", e.to_string()),
        IoError::Unsupported => RpcError::new("not_sim", e.to_string()),
        _ => RpcError::new("serial_error", e.to_string()),
    }
}

fn str_param(params: &Value, key: &str) -> Result<String, RpcError> {
    params
        .get(key)
        .and_then(|v| v.as_str())
        .map(|s| s.to_string())
        .ok_or_else(|| RpcError::new("bad_request", format!("缺少字符串参数 '{key}'")))
}

fn u8_param(params: &Value, key: &str) -> Result<u8, RpcError> {
    params
        .get(key)
        .and_then(|v| v.as_u64())
        .filter(|v| *v <= 253)
        .map(|v| v as u8)
        .ok_or_else(|| RpcError::new("bad_request", format!("缺少/非法参数 '{key}' (0~253)")))
}

fn u16_param(params: &Value, key: &str) -> Result<u16, RpcError> {
    params
        .get(key)
        .and_then(|v| v.as_u64())
        .filter(|v| *v <= u16::MAX as u64)
        .map(|v| v as u16)
        .ok_or_else(|| RpcError::new("bad_request", format!("缺少/非法参数 '{key}' (0~65535)")))
}

fn optional_motors(params: &Value) -> Option<Vec<u8>> {
    params.get("motors").and_then(|v| v.as_array()).map(|arr| {
        arr.iter().filter_map(|v| v.as_u64()).filter(|v| *v <= 253).map(|v| v as u8).collect()
    })
}
