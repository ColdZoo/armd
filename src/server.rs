//! Unix socket RPC 服务：NDJSON 请求/响应 + 发布订阅流。
//!
//! 协议（换行分隔 JSON）：
//! - 请求：`{"id":1,"method":"arm.state","params":{"arm":"koch_v1_1:follower"}}`
//! - 响应：`{"id":1,"result":{...}}` 或 `{"id":1,"error":{"code":"...","message":"..."}}`
//! - 推送：`{"event":"state","sub":3,"arm":"...","seq":...,"joints":{...}}`
//!          `{"event":"arm","sub":3,"arm":"...","connected":...}`

use crate::arm::{ArmHandle, Cmd, Request, Sub};
use crate::config::Config;
use crate::dxl;
use serde_json::{json, Value};
use std::collections::{HashMap, HashSet};
use std::io::{BufRead, BufReader, Write};
use std::os::unix::fs::PermissionsExt;
use std::os::unix::net::{UnixListener, UnixStream};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{sync_channel, SyncSender};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

pub const VERSION: &str = env!("CARGO_PKG_VERSION");

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

    /// 启动监听（阻塞）。
    pub fn serve(self: Arc<Self>) -> std::io::Result<()> {
        let _ = std::fs::remove_file(&self.socket_path);
        let listener = UnixListener::bind(&self.socket_path)?;
        std::fs::set_permissions(&self.socket_path, std::fs::Permissions::from_mode(0o600))?;
        eprintln!("[armd] listening on {} (arms: {:?})", self.socket_path, self.arm_ids());
        let mut conn_id = 0u64;
        for stream in listener.incoming() {
            if self.stop.load(Ordering::Relaxed) {
                break;
            }
            match stream {
                Ok(stream) => {
                    conn_id += 1;
                    self.spawn_conn(stream, conn_id);
                }
                Err(e) => eprintln!("[armd] accept error: {e}"),
            }
        }
        Ok(())
    }

    fn spawn_conn(self: &Arc<Self>, stream: UnixStream, conn_id: u64) {
        let daemon = self.clone();
        std::thread::Builder::new()
            .name(format!("conn-{conn_id}"))
            .spawn(move || daemon.handle_conn(stream, conn_id))
            .ok();
    }

    fn handle_conn(&self, stream: UnixStream, _conn_id: u64) {
        let writer = match stream.try_clone() {
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
            let method = req.get("method").and_then(|m| m.as_str()).unwrap_or("").to_string();
            let params = req.get("params").cloned().unwrap_or_else(|| json!({}));
            let result = self.dispatch(&method, &params, &frame_tx, &mut conn_subs);
            let resp = match result {
                Ok(v) => json!({ "id": id, "result": v }),
                Err(e) => json!({ "id": id, "error": e.to_json() }),
            };
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
    ) -> RpcResult {
        match method {
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
                "stats": arm.stats.to_json(),
            }));
        }
        out.sort_by(|a, b| a["id"].as_str().unwrap_or("").cmp(b["id"].as_str().unwrap_or("")));
        out
    }
}

fn call_arm(arm: &Arc<ArmHandle>, cmd: Cmd, urgent: bool, wait_ms: u64) -> RpcResult {
    let (tx, rx) = sync_channel(1);
    arm.submit(Request { cmd, reply: Some(tx) }, urgent)
        .map_err(|q| RpcError::new("queue_full", format!("{q} 队列已满")))?;
    match rx.recv_timeout(Duration::from_millis(wait_ms)) {
        Ok(Ok(v)) => Ok(v),
        Ok(Err(e)) => Err(RpcError::new("serial_error", e.to_string())),
        Err(_) => Err(RpcError::new("timeout", "机械臂 worker 响应超时")),
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
