#![cfg(unix)]
//! armd v0.2.0 安全层集成测试：起**真实 armd 进程**（配置里的 sim 臂），走真实
//! Unix socket NDJSON RPC + 订阅，验证 `docs/SAFETY-DESIGN.md` §7 的验收场景。
//!
//! - 只用 std（不新增任何 crate）：自带一个小 JSON 解析器 + 手写 NDJSON 客户端；
//! - 每个测试用唯一临时目录（socket / identity / `<socket>.lock` / profile 都在里面），
//!   绝不碰 `/tmp/armd.sock`、`/tmp/armd-identity.json` 或生产 daemon；
//! - 运行：`cargo test --test safety_integration`（cargo 会先构建 `armd` 二进制，
//!   路径由 `env!("CARGO_BIN_EXE_armd")` 提供）；
//! - 调试：`ARMD_TEST_KEEP=1 cargo test --test safety_integration -- --nocapture`
//!   会保留临时目录（含 `daemon.log`）。
//!
//! 场景：
//! 1. `stall_protection_torques_off_and_reverts` — 堵转 → 该电机 torque OFF + goal 回退，
//!    事件/状态/寄存器一致，其他电机与状态流不受影响；
//! 2. `single_motor_hardware_error_isolation` — 单电机 0x80 不拖垮整臂，`arm.reboot` 清除；
//! 3. `soft_limits_clamp_and_reject` — Goal=5000 在 clamp 模式夹紧到 4095-margin，reject 模式报 `soft_limit`；
//! 4. `reconnect_cycles_keep_stream_and_balance_fds` — `arm.reconnect` ×20，订阅不断流、
//!    无错误响应、`opens`/`closes` 稳态不泄漏。

use std::collections::BTreeMap;
use std::fs::{self, File};
use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{channel, Receiver, RecvTimeoutError};
use std::time::{Duration, Instant};

// ---------------------------------------------------------------- 极简 JSON

/// 只保留测试需要的 JSON 子集；数字统一用 f64（本测试涉及的数字都 < 2^53）。
#[derive(Debug, Clone, PartialEq)]
enum Json {
    Null,
    Bool(bool),
    Num(f64),
    Str(String),
    Arr(Vec<Json>),
    Obj(BTreeMap<String, Json>),
}

impl Json {
    fn get(&self, key: &str) -> Option<&Json> {
        match self {
            Json::Obj(map) => map.get(key),
            _ => None,
        }
    }

    fn as_bool(&self) -> Option<bool> {
        match self {
            Json::Bool(b) => Some(*b),
            _ => None,
        }
    }

    fn as_f64(&self) -> Option<f64> {
        match self {
            Json::Num(n) => Some(*n),
            _ => None,
        }
    }

    fn as_i64(&self) -> Option<i64> {
        self.as_f64().map(|n| n as i64)
    }

    fn as_u64(&self) -> Option<u64> {
        self.as_f64().filter(|n| *n >= 0.0).map(|n| n as u64)
    }

    fn as_str(&self) -> Option<&str> {
        match self {
            Json::Str(s) => Some(s),
            _ => None,
        }
    }

    fn as_array(&self) -> Option<&Vec<Json>> {
        match self {
            Json::Arr(v) => Some(v),
            _ => None,
        }
    }

    fn as_object(&self) -> Option<&BTreeMap<String, Json>> {
        match self {
            Json::Obj(m) => Some(m),
            _ => None,
        }
    }

    fn is_empty_object(&self) -> bool {
        matches!(self, Json::Obj(m) if m.is_empty())
    }

    fn str_field(&self, key: &str) -> Option<&str> {
        self.get(key).and_then(Json::as_str)
    }

    fn i64_field(&self, key: &str) -> Option<i64> {
        self.get(key).and_then(Json::as_i64)
    }

    fn u64_field(&self, key: &str) -> Option<u64> {
        self.get(key).and_then(Json::as_u64)
    }

    fn bool_field(&self, key: &str) -> Option<bool> {
        self.get(key).and_then(Json::as_bool)
    }

    fn array_field(&self, key: &str) -> Option<&Vec<Json>> {
        self.get(key).and_then(Json::as_array)
    }
}

struct JsonParser<'a> {
    bytes: &'a [u8],
    pos: usize,
}

impl<'a> JsonParser<'a> {
    fn new(text: &'a str) -> Self {
        Self {
            bytes: text.as_bytes(),
            pos: 0,
        }
    }

    fn parse(mut self) -> Result<Json, String> {
        let value = self.value()?;
        self.skip_ws();
        if self.pos != self.bytes.len() {
            return Err(format!("JSON 尾部有多余内容（位置 {}）", self.pos));
        }
        Ok(value)
    }

    fn peek(&self) -> Option<u8> {
        self.bytes.get(self.pos).copied()
    }

    fn skip_ws(&mut self) {
        while let Some(b' ' | b'\t' | b'\r' | b'\n') = self.peek() {
            self.pos += 1;
        }
    }

    fn expect(&mut self, byte: u8) -> Result<(), String> {
        if self.peek() == Some(byte) {
            self.pos += 1;
            Ok(())
        } else {
            Err(format!(
                "期望字节 {:?}（位置 {}），实际 {:?}",
                byte as char,
                self.pos,
                self.peek().map(|b| b as char)
            ))
        }
    }

    fn value(&mut self) -> Result<Json, String> {
        self.skip_ws();
        match self.peek() {
            Some(b'{') => self.object(),
            Some(b'[') => self.array(),
            Some(b'"') => Ok(Json::Str(self.string()?)),
            Some(b't') => {
                self.literal("true")?;
                Ok(Json::Bool(true))
            }
            Some(b'f') => {
                self.literal("false")?;
                Ok(Json::Bool(false))
            }
            Some(b'n') => {
                self.literal("null")?;
                Ok(Json::Null)
            }
            Some(b'-' | b'0'..=b'9') => self.number(),
            other => Err(format!(
                "非法 JSON 起始字节 {:?}（位置 {}）",
                other.map(|b| b as char),
                self.pos
            )),
        }
    }

    fn literal(&mut self, word: &str) -> Result<(), String> {
        let end = self.pos + word.len();
        if self.bytes.get(self.pos..end) == Some(word.as_bytes()) {
            self.pos = end;
            Ok(())
        } else {
            Err(format!("期望字面量 {word:?}（位置 {}）", self.pos))
        }
    }

    fn object(&mut self) -> Result<Json, String> {
        self.expect(b'{')?;
        let mut map = BTreeMap::new();
        self.skip_ws();
        if self.peek() == Some(b'}') {
            self.pos += 1;
            return Ok(Json::Obj(map));
        }
        loop {
            self.skip_ws();
            let key = self.string()?;
            self.skip_ws();
            self.expect(b':')?;
            let value = self.value()?;
            map.insert(key, value);
            self.skip_ws();
            match self.peek() {
                Some(b',') => {
                    self.pos += 1;
                }
                Some(b'}') => {
                    self.pos += 1;
                    return Ok(Json::Obj(map));
                }
                other => {
                    return Err(format!(
                        "对象里期望 ',' 或 '}}'（位置 {}），实际 {:?}",
                        self.pos,
                        other.map(|b| b as char)
                    ))
                }
            }
        }
    }

    fn array(&mut self) -> Result<Json, String> {
        self.expect(b'[')?;
        let mut items = Vec::new();
        self.skip_ws();
        if self.peek() == Some(b']') {
            self.pos += 1;
            return Ok(Json::Arr(items));
        }
        loop {
            items.push(self.value()?);
            self.skip_ws();
            match self.peek() {
                Some(b',') => {
                    self.pos += 1;
                }
                Some(b']') => {
                    self.pos += 1;
                    return Ok(Json::Arr(items));
                }
                other => {
                    return Err(format!(
                        "数组里期望 ',' 或 ']'（位置 {}），实际 {:?}",
                        self.pos,
                        other.map(|b| b as char)
                    ))
                }
            }
        }
    }

    fn string(&mut self) -> Result<String, String> {
        self.expect(b'"')?;
        let mut out: Vec<u8> = Vec::new();
        loop {
            let Some(byte) = self.peek() else {
                return Err("字符串未闭合".into());
            };
            self.pos += 1;
            match byte {
                b'"' => {
                    return String::from_utf8(out).map_err(|e| format!("非法 UTF-8 字符串: {e}"))
                }
                b'\\' => {
                    let Some(esc) = self.peek() else {
                        return Err("字符串转义未完成".into());
                    };
                    self.pos += 1;
                    match esc {
                        b'"' => out.push(b'"'),
                        b'\\' => out.push(b'\\'),
                        b'/' => out.push(b'/'),
                        b'b' => out.push(0x08),
                        b'f' => out.push(0x0c),
                        b'n' => out.push(b'\n'),
                        b'r' => out.push(b'\r'),
                        b't' => out.push(b'\t'),
                        b'u' => {
                            let cp = self.hex4()?;
                            let ch = if (0xD800..0xDC00).contains(&cp) {
                                // 高代理项：尝试配一个低位
                                if self.peek() == Some(b'\\')
                                    && self.bytes.get(self.pos + 1) == Some(&b'u')
                                {
                                    self.pos += 2;
                                    let low = self.hex4()?;
                                    if (0xDC00..0xE000).contains(&low) {
                                        let combined =
                                            0x10000 + ((cp - 0xD800) << 10) + (low - 0xDC00);
                                        char::from_u32(combined).unwrap_or('\u{FFFD}')
                                    } else {
                                        '\u{FFFD}'
                                    }
                                } else {
                                    '\u{FFFD}'
                                }
                            } else {
                                char::from_u32(cp).unwrap_or('\u{FFFD}')
                            };
                            let mut buf = [0u8; 4];
                            out.extend_from_slice(ch.encode_utf8(&mut buf).as_bytes());
                        }
                        other => return Err(format!("未知转义 \\{}", other as char)),
                    }
                }
                0x00..=0x1f => return Err(format!("字符串里有裸控制字符 0x{byte:02x}")),
                _ => out.push(byte),
            }
        }
    }

    fn hex4(&mut self) -> Result<u32, String> {
        let Some(slice) = self.bytes.get(self.pos..self.pos + 4) else {
            return Err("\\u 转义不完整".into());
        };
        let text = std::str::from_utf8(slice).map_err(|_| "\\u 转义非 UTF-8".to_string())?;
        let n = u32::from_str_radix(text, 16).map_err(|e| format!("\\u{text} 非法: {e}"))?;
        self.pos += 4;
        Ok(n)
    }

    fn number(&mut self) -> Result<Json, String> {
        let start = self.pos;
        if self.peek() == Some(b'-') {
            self.pos += 1;
        }
        let mut has_digit = false;
        while let Some(b'0'..=b'9') = self.peek() {
            has_digit = true;
            self.pos += 1;
        }
        if !has_digit {
            return Err(format!("数字缺少整数部分（位置 {start}）"));
        }
        if self.peek() == Some(b'.') {
            self.pos += 1;
            let mut frac_digits = false;
            while let Some(b'0'..=b'9') = self.peek() {
                frac_digits = true;
                self.pos += 1;
            }
            if !frac_digits {
                return Err(format!("数字缺少小数位（位置 {start}）"));
            }
        }
        if let Some(b'e' | b'E') = self.peek() {
            self.pos += 1;
            if let Some(b'+' | b'-') = self.peek() {
                self.pos += 1;
            }
            let mut exp_digits = false;
            while let Some(b'0'..=b'9') = self.peek() {
                exp_digits = true;
                self.pos += 1;
            }
            if !exp_digits {
                return Err(format!("数字缺少指数位（位置 {start}）"));
            }
        }
        let text = std::str::from_utf8(&self.bytes[start..self.pos]).unwrap_or_default();
        let n: f64 = text
            .parse()
            .map_err(|e| format!("数字 {text:?} 解析失败: {e}"))?;
        Ok(Json::Num(n))
    }
}

fn parse_json(text: &str) -> Result<Json, String> {
    JsonParser::new(text).parse()
}

/// 把字符串转义成 JSON 字符串字面量（配置里的临时路径用）。
fn json_escape(text: &str) -> String {
    let mut out = String::with_capacity(text.len() + 2);
    for ch in text.chars() {
        match ch {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out
}

fn motor_field<'a>(root: &'a Json, field: &str, motor: u8) -> Option<&'a Json> {
    root.get(field)?.get(&motor.to_string())
}

/// `applied` 可能是数字（单电机 write_register）也可能是
/// `{"1": {"requested":..,"applied":..}}`（sync_write 的 clamped 明细），都兼容。
fn applied_for_motor(result: &Json, motor: u8) -> Option<i64> {
    if let Some(applied) = result.get("applied") {
        match applied {
            Json::Num(_) => return applied.as_i64(),
            Json::Obj(_) => {
                if let Some(v) = applied.get(&motor.to_string()).and_then(Json::as_i64) {
                    return Some(v);
                }
            }
            _ => {}
        }
    }
    result
        .get("clamped")?
        .get(&motor.to_string())?
        .get("applied")
        .and_then(Json::as_i64)
}

fn clamped_for_motor(result: &Json, motor: u8) -> Option<bool> {
    let clamped = result.get("clamped")?;
    match clamped {
        Json::Bool(b) => Some(*b),
        Json::Obj(map) => map.get(&motor.to_string()).map(|_| true),
        _ => None,
    }
}

// ---------------------------------------------------------------- NDJSON RPC 客户端

#[derive(Debug, Clone)]
struct RpcError {
    code: String,
    message: String,
}

impl std::fmt::Display for RpcError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}: {}", self.code, self.message)
    }
}

impl RpcError {
    fn from_json(value: &Json) -> Self {
        Self {
            code: value.str_field("code").unwrap_or("<missing>").to_string(),
            message: value
                .str_field("message")
                .unwrap_or("<missing>")
                .to_string(),
        }
    }
}

struct Client {
    reader: BufReader<UnixStream>,
    next_id: u64,
    socket: PathBuf,
}

impl Client {
    fn connect(socket: &Path) -> Client {
        let mut last_err = None;
        for _ in 0..250 {
            match UnixStream::connect(socket) {
                Ok(stream) => {
                    stream.set_read_timeout(Some(Duration::from_secs(10))).ok();
                    stream.set_write_timeout(Some(Duration::from_secs(10))).ok();
                    return Client {
                        reader: BufReader::new(stream),
                        next_id: 1,
                        socket: socket.to_path_buf(),
                    };
                }
                Err(e) => {
                    last_err = Some(e);
                    std::thread::sleep(Duration::from_millis(20));
                }
            }
        }
        panic!("无法连接 armd socket {}: {last_err:?}", socket.display());
    }

    fn call(&mut self, method: &str, params_json: &str) -> Result<Json, RpcError> {
        let id = self.next_id;
        self.next_id += 1;
        let request = format!(r#"{{"id":{id},"method":"{method}","params":{params_json}}}"#);
        {
            let stream = self.reader.get_mut();
            stream.write_all(request.as_bytes()).unwrap_or_else(|e| {
                panic!("写请求 {method} 到 {} 失败: {e}", self.socket.display())
            });
            stream.write_all(b"\n").unwrap_or_else(|e| {
                panic!("写请求 {method} 到 {} 失败: {e}", self.socket.display())
            });
            stream.flush().unwrap_or_else(|e| {
                panic!("flush 请求 {method} 到 {} 失败: {e}", self.socket.display())
            });
        }
        for _ in 0..16 {
            let mut line = String::new();
            match self.reader.read_line(&mut line) {
                Ok(0) => panic!(
                    "等待 {method} 响应时 armd 关闭了连接（{}）",
                    self.socket.display()
                ),
                Ok(_) => {}
                Err(e) => panic!("等待 {method} 响应时读 socket 失败: {e}"),
            }
            let text = line.trim();
            if text.is_empty() {
                continue;
            }
            let frame = parse_json(text)
                .unwrap_or_else(|e| panic!("{method} 响应不是合法 JSON: {e}\n原文: {text}"));
            if frame.u64_field("id") != Some(id) {
                // 理论上 RPC 连接上不会有推送帧；出现了就跳过。
                continue;
            }
            if let Some(err) = frame.get("error") {
                return Err(RpcError::from_json(err));
            }
            return Ok(frame.get("result").cloned().unwrap_or(Json::Null));
        }
        panic!("{method}: 收到 16 个非本请求 id 的帧，仍未等到 id={id} 的响应");
    }

    fn call_ok(&mut self, method: &str, params_json: &str) -> Json {
        match self.call(method, params_json) {
            Ok(result) => result,
            Err(e) => panic!("{method} 失败: {e}\nparams: {params_json}"),
        }
    }

    fn call_err(&mut self, method: &str, params_json: &str) -> RpcError {
        match self.call(method, params_json) {
            Ok(result) => panic!("{method} 期望失败，实际成功: {result:?}\nparams: {params_json}"),
            Err(e) => e,
        }
    }

    fn state(&mut self) -> Json {
        self.call_ok("arm.state", &format!(r#"{{"arm":"{ARM}"}}"#))
    }

    fn stats(&mut self) -> Json {
        self.call_ok("arm.stats", &format!(r#"{{"arm":"{ARM}"}}"#))
    }

    fn reconnect(&mut self) -> Json {
        self.call_ok("arm.reconnect", &format!(r#"{{"arm":"{ARM}"}}"#))
    }
}

/// 等到状态满足条件；失败时把最后一次状态打进 panic，便于排查。
fn wait_state(
    client: &mut Client,
    timeout: Duration,
    what: &str,
    pred: impl Fn(&Json) -> bool,
) -> Json {
    let deadline = Instant::now() + timeout;
    let mut last = Json::Null;
    while Instant::now() < deadline {
        last = client.state();
        if pred(&last) {
            return last;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    panic!(
        "等待状态条件超时（{what}，{:?}）；最后一次 arm.state: {last:?}",
        timeout
    );
}

fn wait_connected(client: &mut Client, timeout: Duration) -> Json {
    wait_state(client, timeout, "connected=true 且已有 joints", |s| {
        s.bool_field("connected") == Some(true) && s.as_object_field_non_empty("joints")
    })
}

/// `Json` 上补一个辅助：字段是非空对象（joints/motor_errors 之类）。
impl Json {
    fn as_object_field_non_empty(&self, field: &str) -> bool {
        self.get(field)
            .and_then(Json::as_object)
            .map(|o| !o.is_empty())
            .unwrap_or(false)
    }

    fn field_is_empty_object(&self, field: &str) -> bool {
        match self.get(field) {
            None => true,
            Some(value) => value.is_empty_object(),
        }
    }
}

// ---------------------------------------------------------------- 订阅端

struct Subscriber {
    // 仅用于排查问题/日志（订阅 id 由服务端分配）
    #[allow(dead_code)]
    sub_id: u64,
    rx: Receiver<Json>,
}

impl Subscriber {
    fn wait_for<F: Fn(&Json) -> bool>(&self, timeout: Duration, pred: F) -> Option<Json> {
        let deadline = Instant::now() + timeout;
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return None;
            }
            match self.rx.recv_timeout(left) {
                Ok(frame) => {
                    if pred(&frame) {
                        return Some(frame);
                    }
                }
                Err(RecvTimeoutError::Timeout) | Err(RecvTimeoutError::Disconnected) => {
                    return None
                }
            }
        }
    }

    fn drain(&self) -> Vec<Json> {
        let mut frames = Vec::new();
        while let Ok(frame) = self.rx.try_recv() {
            frames.push(frame);
        }
        frames
    }
}

fn is_state_event(frame: &Json) -> bool {
    frame.str_field("event") == Some("state")
}

fn is_stall_event_for(frame: &Json, motor: u8) -> bool {
    frame.str_field("event") == Some("stall") && frame.i64_field("motor") == Some(motor as i64)
}

fn is_motor_error_event_for(frame: &Json, motor: u8) -> bool {
    frame.str_field("event") == Some("motor_error")
        && frame.i64_field("motor") == Some(motor as i64)
}

// ---------------------------------------------------------------- armd 测试进程

const ARM: &str = "sim";

static NEXT_TEST_DAEMON: AtomicU64 = AtomicU64::new(0);

struct TestDaemon {
    label: String,
    dir: PathBuf,
    socket: PathBuf,
    log: PathBuf,
    child: Option<Child>,
}

impl TestDaemon {
    /// 用给定 arm 配置片段（json 对象，允许 `{PROFILE}` 占位符）起一个独立 armd。
    fn spawn(label: &str, arm_template: &str) -> TestDaemon {
        let seq = NEXT_TEST_DAEMON.fetch_add(1, Ordering::Relaxed);
        let dir = std::env::temp_dir().join(format!(
            "armd-test-{}-{}-{}",
            std::process::id(),
            seq,
            label
        ));
        let _ = fs::remove_dir_all(&dir);
        fs::create_dir_all(&dir)
            .unwrap_or_else(|e| panic!("创建临时目录 {} 失败: {e}", dir.display()));

        let socket = dir.join("armd.sock");
        let socket_text = socket.to_string_lossy().to_string();
        // macOS 的 sockaddr_un.sun_path 只有 104 字节，路径过长时 bind 会失败。
        assert!(
            socket_text.len() < 100,
            "临时 socket 路径太长（macOS 上限 104 字节）: {socket_text}"
        );
        let identity = dir.join("identity.json");
        let profile = dir.join("profile.json");
        let config = dir.join("config.json");
        let log = dir.join("daemon.log");

        let arm = arm_template.replace("{PROFILE}", &json_escape(&profile.to_string_lossy()));
        let config_text = format!(
            r#"{{"socket":"{socket}","identity":"{identity}","arms":[{arm}]}}"#,
            socket = json_escape(&socket_text),
            identity = json_escape(&identity.to_string_lossy()),
        );
        fs::write(&config, config_text)
            .unwrap_or_else(|e| panic!("写测试配置 {} 失败: {e}", config.display()));

        let log_file = File::create(&log)
            .unwrap_or_else(|e| panic!("创建 daemon 日志 {} 失败: {e}", log.display()));
        let child = Command::new(env!("CARGO_BIN_EXE_armd"))
            .arg("--config")
            .arg(&config)
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(Stdio::from(log_file))
            .spawn()
            .unwrap_or_else(|e| {
                panic!(
                    "无法启动 armd（{}）: {e}\n（cargo test 会自动构建并使用 CARGO_BIN_EXE_armd）",
                    env!("CARGO_BIN_EXE_armd")
                )
            });

        let mut daemon = TestDaemon {
            label: label.to_string(),
            dir,
            socket,
            log,
            child: Some(child),
        };
        daemon.wait_until_socket_ready();
        daemon
    }

    fn wait_until_socket_ready(&mut self) {
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            if let Some(status) = self
                .child
                .as_mut()
                .expect("child")
                .try_wait()
                .unwrap_or_else(|e| panic!("waitpid armd 失败: {e}"))
            {
                panic!(
                    "armd[{}] 启动即退出（{status}）：\n{}",
                    self.label,
                    self.log_tail()
                );
            }
            if UnixStream::connect(&self.socket).is_ok() {
                return;
            }
            if Instant::now() >= deadline {
                panic!(
                    "armd[{}] socket {} 5s 内未就绪：\n{}",
                    self.label,
                    self.socket.display(),
                    self.log_tail()
                );
            }
            std::thread::sleep(Duration::from_millis(20));
        }
    }

    fn connect(&self) -> Client {
        Client::connect(&self.socket)
    }

    fn subscribe(&self, arms: &[&str], rate_hz: f64) -> Subscriber {
        let mut stream = UnixStream::connect(&self.socket).unwrap_or_else(|e| {
            panic!(
                "订阅连接 armd[{}]（{}）失败: {e}",
                self.label,
                self.socket.display()
            )
        });
        stream
            .set_read_timeout(Some(Duration::from_secs(5)))
            .expect("设置订阅连接超时");
        let arms_json: Vec<String> = arms.iter().map(|a| format!("\"{a}\"")).collect();
        let request = format!(
            r#"{{"id":1,"method":"sub.subscribe","params":{{"arms":[{}],"rate_hz":{rate_hz}}}}}"#,
            arms_json.join(",")
        );
        stream
            .write_all(request.as_bytes())
            .expect("写 sub.subscribe");
        stream.write_all(b"\n").expect("写 sub.subscribe");
        stream.flush().expect("flush sub.subscribe");

        let mut reader = BufReader::new(stream);
        let mut line = String::new();
        reader.read_line(&mut line).expect("读 sub.subscribe 响应");
        let frame = parse_json(line.trim()).unwrap_or_else(|e| {
            panic!(
                "sub.subscribe 响应不是合法 JSON: {e}\n原文: {line}\narmd 日志:\n{}",
                self.log_tail()
            )
        });
        if let Some(err) = frame.get("error") {
            panic!(
                "sub.subscribe 失败: {}\narmd 日志:\n{}",
                RpcError::from_json(err),
                self.log_tail()
            );
        }
        let sub_id = frame
            .get("result")
            .and_then(|r| r.u64_field("sub_id"))
            .unwrap_or_else(|| panic!("sub.subscribe 结果缺少 sub_id: {frame:?}"));

        let (tx, rx) = channel::<Json>();
        std::thread::Builder::new()
            .name(format!("sub-{}", self.label))
            .spawn(move || {
                reader
                    .get_mut()
                    .set_read_timeout(Some(Duration::from_millis(250)))
                    .ok();
                loop {
                    let mut line = String::new();
                    match reader.read_line(&mut line) {
                        Ok(0) => break,
                        Ok(_) => {
                            let text = line.trim();
                            if text.is_empty() {
                                continue;
                            }
                            if let Ok(frame) = parse_json(text) {
                                if tx.send(frame).is_err() {
                                    break;
                                }
                            }
                        }
                        Err(e) => {
                            let is_timeout = matches!(
                                e.kind(),
                                std::io::ErrorKind::WouldBlock
                                    | std::io::ErrorKind::TimedOut
                                    | std::io::ErrorKind::Interrupted
                            );
                            if is_timeout {
                                continue;
                            }
                            break;
                        }
                    }
                }
            })
            .expect("spawn 订阅读线程");

        Subscriber { sub_id, rx }
    }

    fn log_tail(&self) -> String {
        let text = fs::read_to_string(&self.log).unwrap_or_default();
        let lines: Vec<&str> = text.lines().collect();
        let start = lines.len().saturating_sub(40);
        lines[start..].join("\n")
    }
}

impl Drop for TestDaemon {
    fn drop(&mut self) {
        if let Some(mut child) = self.child.take() {
            let _ = child.kill();
            let _ = child.wait();
        }
        if std::env::var_os("ARMD_TEST_KEEP").is_some() {
            eprintln!("[safety_integration] 保留测试目录: {}", self.dir.display());
        } else {
            let _ = fs::remove_dir_all(&self.dir);
        }
    }
}

// ---------------------------------------------------------------- 配置构造

/// 与生产 `koch_v1_1` 默认值一致的 sim 臂（便于复现真机行为）。
fn sim_arm_json(soft_limit_mode: &str, reconnect_sec: f64) -> String {
    let template = r#"{
        "id": "sim",
        "label": "sim arm",
        "sim": true,
        "port": "",
        "poll_hz": 60,
        "read_timeout_ms": 30,
        "motors": [1, 2, 3, 4, 5, 6],
        "reconnect_sec": {RECONNECT_SEC},
        "stall_timeout_ms": 250,
        "stall_tolerance": 60,
        "stall_velocity_epsilon": 2,
        "stall_protection": true,
        "stall_exclude_motors": [],
        "auto_torque_off": true,
        "stall_recover_margin": 20,
        "soft_limits": true,
        "soft_limit_mode": "{SOFT_LIMIT_MODE}",
        "soft_limit_margin": 10,
        "joint_names": {
            "1": "shoulder_pan",
            "2": "shoulder_lift",
            "3": "elbow_flex",
            "4": "wrist_flex",
            "5": "wrist_roll",
            "6": "gripper"
        },
        "profile": "{PROFILE}"
    }"#;
    template
        .replace("{SOFT_LIMIT_MODE}", soft_limit_mode)
        .replace("{RECONNECT_SEC}", &format!("{reconnect_sec}"))
}

/// 指定 `stall_exclude_motors` 的 sim 臂（用于夹爪排除测试）。
fn sim_arm_json_excluding(soft_limit_mode: &str, reconnect_sec: f64, excluded: &[u8]) -> String {
    let excluded: Vec<String> = excluded.iter().map(|id| id.to_string()).collect();
    sim_arm_json(soft_limit_mode, reconnect_sec).replace(
        "\"stall_exclude_motors\": []",
        &format!("\"stall_exclude_motors\": [{}]", excluded.join(",")),
    )
}

fn joint_pos(state: &Json, motor: u8) -> i64 {
    motor_field(state, "joints", motor)
        .and_then(Json::as_i64)
        .unwrap_or_else(|| panic!("arm.state.joints 缺少 motor {motor}: {state:?}"))
}

fn empty_motor_errors(state: &Json) -> bool {
    // §5：正常时应为 {}；为兼容"字段暂缺"的早期实现，缺字段也当空。
    state.field_is_empty_object("motor_errors")
}

// ---------------------------------------------------------------- 测试 1：堵转保护

#[test]
fn stall_protection_torques_off_and_reverts() {
    let daemon = TestDaemon::spawn("stall", &sim_arm_json("clamp", 0.1));
    let mut client = daemon.connect();
    wait_connected(&mut client, Duration::from_secs(5));

    // torque ON（仿真默认已 ON，这里显式置位，保证 armd 的 torque 缓存已知）
    client.call_ok("arm.torque", r#"{"arm":"sim","enable":true,"motors":[3]}"#);
    wait_state(&mut client, Duration::from_secs(2), "torque[3]=true", |s| {
        motor_field(s, "torque", 3).and_then(Json::as_bool) == Some(true)
    });

    // 订阅要在注入之前建立（验收要求：事件能推到已有订阅）
    let sub = daemon.subscribe(&[ARM], 60.0);
    std::thread::sleep(Duration::from_millis(100));
    let _ = sub.drain();

    let state = client.state();
    let present = joint_pos(&state, 3);
    let goal = present + 800;
    // 先给一个明确目标，否则 goal==present 不满足"偏差 > tolerance"的堵转条件；
    // 目标写下后立刻注入 stall（仿真的位置会被冻结）。
    let write_ack = client.call_ok(
        "arm.sync_write",
        &format!(r#"{{"arm":"sim","address":116,"length":4,"values":{{"3":{goal}}}}}"#),
    );
    assert_eq!(
        write_ack.bool_field("ok"),
        Some(true),
        "sync_write goal 应成功: {write_ack:?}"
    );
    let started = Instant::now();
    client.call_ok(
        "arm.sim.inject",
        r#"{"arm":"sim","kind":"stall","motors":[3]}"#,
    );

    // 事件：1500ms 内到达，字段齐全（§4）
    let event = sub
        .wait_for(Duration::from_millis(1500), |f| is_stall_event_for(f, 3))
        .unwrap_or_else(|| {
            panic!(
                "注入 stall 后 1500ms 内没收到 event:\"stall\"(motor=3)；armd 日志:\n{}",
                daemon.log_tail()
            )
        });
    let event_latency = started.elapsed();
    for key in ["reason", "position", "goal", "stall_ms", "torque_off"] {
        assert!(
            event.get(key).is_some(),
            "stall 事件缺少 {key} 字段: {event:?}"
        );
    }
    assert_eq!(
        event.bool_field("torque_off"),
        Some(true),
        "stall 事件应符合 auto_torque_off: {event:?}"
    );
    assert_eq!(
        event.str_field("reason"),
        Some("stall"),
        "stall 事件 reason 期望 \"stall\": {event:?}"
    );
    assert!(
        event.u64_field("stall_ms").unwrap_or(0) >= 200,
        "stall_ms 应覆盖 stall_timeout_ms(250) 的量级: {event:?}"
    );
    eprintln!("[stall] 事件延迟 {event_latency:?}: {event:?}");

    // 状态：stalls、connected、torque OFF、无硬件错误
    let stalled_state = wait_state(
        &mut client,
        Duration::from_secs(2),
        "arm.state.stalls[\"3\"] 出现且 torque[3]=false",
        |s| {
            motor_field(s, "stalls", 3).is_some()
                && motor_field(s, "torque", 3).and_then(Json::as_bool) == Some(false)
        },
    );
    assert_eq!(
        stalled_state.bool_field("connected"),
        Some(true),
        "单电机堵转不应让整臂 disconnected: {stalled_state:?}"
    );
    assert!(
        empty_motor_errors(&stalled_state),
        "堵转不应产生硬件错误 motor_errors: {stalled_state:?}"
    );
    let stall_state = motor_field(&stalled_state, "stalls", 3).expect("stalls[3]");
    assert_eq!(
        stall_state.i64_field("motor"),
        Some(3),
        "stalls[3].motor 应为 3: {stall_state:?}"
    );
    assert_eq!(
        stall_state.bool_field("torque_off"),
        Some(true),
        "stalls[3].torque_off 应为 true: {stall_state:?}"
    );
    assert!(
        stall_state.i64_field("position").is_some() && stall_state.i64_field("goal").is_some(),
        "stalls[3] 应带 position/goal: {stall_state:?}"
    );

    // 其他电机：状态流继续推进、joints 仍在更新
    let before = client.state();
    std::thread::sleep(Duration::from_millis(150));
    let after = client.state();
    assert!(
        after.u64_field("seq").unwrap_or(0) > before.u64_field("seq").unwrap_or(0),
        "堵转后状态 seq 应继续增长: before={:?} after={:?}",
        before.u64_field("seq"),
        after.u64_field("seq")
    );
    for motor in [1u8, 2, 4, 5, 6] {
        assert!(
            motor_field(&after, "joints", motor)
                .and_then(Json::as_i64)
                .is_some(),
            "电机 {motor} 的 joints 不应受电机 3 堵转影响: {after:?}"
        );
    }
    // 订阅状态帧仍在推
    let seq_before = before.u64_field("seq").unwrap_or(0);
    assert!(
        sub.wait_for(Duration::from_secs(1), |f| {
            is_state_event(f) && f.u64_field("seq").unwrap_or(0) > seq_before
        })
        .is_some(),
        "堵转后订阅状态流不应中断"
    );

    // 寄存器回读：Torque_Enable(64) 必须真的是 0
    let torque_reg = client.call_ok(
        "arm.read_register",
        r#"{"arm":"sim","motor":3,"address":64,"length":1}"#,
    );
    assert_eq!(
        torque_reg.i64_field("value"),
        Some(0),
        "stall 后 read_register(64, motor 3) 应为 0: {torque_reg:?}"
    );

    // 清除注入 → 无硬件错误
    client.call_ok(
        "arm.sim.inject",
        r#"{"arm":"sim","kind":"clear","motors":[3]}"#,
    );
    let cleared = wait_state(
        &mut client,
        Duration::from_secs(1),
        "clear 后 motor_errors 为空 + connected",
        |s| empty_motor_errors(s) && s.bool_field("connected") == Some(true),
    );
    assert_eq!(
        cleared.bool_field("connected"),
        Some(true),
        "clear 后应保持 connected: {cleared:?}"
    );
    let hw_reg = client.call_ok(
        "arm.read_register",
        r#"{"arm":"sim","motor":3,"address":70,"length":1}"#,
    );
    assert_eq!(
        hw_reg.i64_field("value"),
        Some(0),
        "stall 期间不应留下 Hardware_Error_Status(70): {hw_reg:?}"
    );
    let _ = daemon; // Drop: kill + wait + 清理临时目录
}

// ---------------------------------------------------------------- 测试 2：单电机硬件错误隔离

#[test]
fn single_motor_hardware_error_isolation() {
    let daemon = TestDaemon::spawn("hardware-error", &sim_arm_json("clamp", 0.1));
    let mut client = daemon.connect();
    wait_connected(&mut client, Duration::from_secs(5));

    let sub = daemon.subscribe(&[ARM], 60.0);
    std::thread::sleep(Duration::from_millis(100));
    let _ = sub.drain();

    let started = Instant::now();
    client.call_ok(
        "arm.sim.inject",
        r#"{"arm":"sim","kind":"hardware_error","motors":[3]}"#,
    );

    // 事件：1s 内收到 motor_error，code=0x80
    let event = sub
        .wait_for(Duration::from_millis(1000), |f| {
            is_motor_error_event_for(f, 3)
        })
        .unwrap_or_else(|| {
            panic!(
                "注入 hardware_error 后 1s 内没收到 event:\"motor_error\"(motor=3)；armd 日志:\n{}",
                daemon.log_tail()
            )
        });
    eprintln!(
        "[hardware_error] 事件延迟 {:?}: {event:?}",
        started.elapsed()
    );
    assert_eq!(
        event.i64_field("code"),
        Some(128),
        "motor_error 事件 code 应为 128(0x80): {event:?}"
    );
    let names = event
        .array_field("names")
        .map(|arr| {
            arr.iter()
                .filter_map(Json::as_str)
                .map(|s| s.to_string())
                .collect::<Vec<_>>()
        })
        .unwrap_or_default();
    assert!(
        names.iter().any(|n| n == "hardware_alert"),
        "motor_error 事件 names 应含 hardware_alert: {event:?}"
    );

    // 状态：只有电机 3 有 error，整臂仍 connected，其他电机正常
    let err_state = wait_state(
        &mut client,
        Duration::from_secs(1),
        "arm.state.motor_errors[\"3\"].code==128",
        |s| motor_field(s, "motor_errors", 3).and_then(|e| e.i64_field("code")) == Some(128),
    );
    assert_eq!(
        err_state.bool_field("connected"),
        Some(true),
        "单电机硬件错误不应让整臂 disconnected: {err_state:?}"
    );
    assert!(
        err_state
            .get("error")
            .map(|e| matches!(e, Json::Null))
            .unwrap_or(true),
        "单电机硬件错误不应写整臂 error 字段: error={:?}",
        err_state.get("error")
    );
    let errors_obj = err_state
        .get("motor_errors")
        .and_then(Json::as_object)
        .expect("arm.state 应有 motor_errors 对象");
    assert_eq!(
        errors_obj.keys().collect::<Vec<_>>(),
        vec!["3"],
        "motor_errors 只应包含电机 3: {errors_obj:?}"
    );
    for motor in [1u8, 2, 4, 5, 6] {
        assert!(
            motor_field(&err_state, "joints", motor)
                .and_then(Json::as_i64)
                .is_some(),
            "电机 {motor} 的 joints 应继续更新: {err_state:?}"
        );
        assert!(
            motor_field(&err_state, "velocity", motor)
                .and_then(Json::as_i64)
                .is_some(),
            "电机 {motor} 的 velocity 应继续更新: {err_state:?}"
        );
    }

    // arm.read 也要按电机返回 errors（§6）：hardware_error 电机没有 data、只有 errors
    let read = client.call_ok(
        "arm.read",
        r#"{"arm":"sim","motors":[1,2,3,4,5,6],"timeout_ms":100}"#,
    );
    let read_positions = read
        .get("positions")
        .and_then(Json::as_object)
        .expect("arm.read 应有 positions 对象");
    let read_errors = read
        .get("errors")
        .and_then(Json::as_object)
        .expect("arm.read 应带 errors 对象（按电机返回硬件错误）");
    assert!(
        !read_positions.contains_key("3"),
        "带 hardware_error 的电机 3 不应出现在 arm.read.positions: {read:?}"
    );
    assert_eq!(
        read_errors.get("3").and_then(Json::as_i64),
        Some(128),
        "arm.read.errors[\"3\"] 应为 128: {read:?}"
    );
    for motor in ["1", "2", "4", "5", "6"] {
        assert!(
            read_positions.contains_key(motor),
            "正常电机 {motor} 应出现在 arm.read.positions: {read:?}"
        );
    }

    // 状态流不断（seq 继续增长）
    let s1 = client.state();
    std::thread::sleep(Duration::from_millis(200));
    let s2 = client.state();
    assert!(
        s2.u64_field("seq").unwrap_or(0) > s1.u64_field("seq").unwrap_or(0),
        "单电机错误期间状态流不应停止: s1={s1:?} s2={s2:?}"
    );

    // arm.reboot 清除 latched 错误
    let reboot = client.call_ok("arm.reboot", r#"{"arm":"sim","motors":[3]}"#);
    assert_eq!(
        reboot.bool_field("ok"),
        Some(true),
        "arm.reboot 应成功: {reboot:?}"
    );
    let rebooted: Vec<i64> = reboot
        .array_field("rebooted")
        .map(|arr| arr.iter().filter_map(Json::as_i64).collect())
        .unwrap_or_default();
    assert!(
        rebooted.contains(&3),
        "arm.reboot.rebooted 应包含 3: {reboot:?}"
    );

    // reboot 后：motor_errors 清空、torque[3]=false、connected=true
    let after_reboot = wait_state(
        &mut client,
        Duration::from_secs(3),
        "reboot 后 motor_errors 清空 + torque[3]=false + connected",
        |s| {
            empty_motor_errors(s)
                && motor_field(s, "torque", 3).and_then(Json::as_bool) == Some(false)
                && s.bool_field("connected") == Some(true)
        },
    );
    assert!(
        after_reboot.bool_field("connected") == Some(true),
        "reboot 后应 connected: {after_reboot:?}"
    );

    // 之后 torque 能重新使能
    client.call_ok("arm.torque", r#"{"arm":"sim","enable":true,"motors":[3]}"#);
    let enabled = wait_state(
        &mut client,
        Duration::from_secs(2),
        "torque[3]=true（reboot 后可重新使能）",
        |s| motor_field(s, "torque", 3).and_then(Json::as_bool) == Some(true),
    );
    assert_eq!(
        motor_field(&enabled, "torque", 3).and_then(Json::as_bool),
        Some(true),
        "arm.torque enable 后 torque[3] 应为 true: {enabled:?}"
    );
    let _ = daemon;
}

// ---------------------------------------------------------------- 测试 3：软限位

#[test]
fn soft_limits_clamp_and_reject() {
    // ---- clamp（默认）
    let daemon = TestDaemon::spawn("soft-limit-clamp", &sim_arm_json("clamp", 0.1));
    let mut client = daemon.connect();
    wait_connected(&mut client, Duration::from_secs(5));
    wait_state(
        &mut client,
        Duration::from_secs(3),
        "limits[\"1\"] 已从 EEPROM 读到",
        |s| motor_field(s, "limits", 1).is_some(),
    );

    let result = client.call_ok(
        "arm.sync_write",
        r#"{"arm":"sim","address":116,"length":4,"values":{"1":5000}}"#,
    );
    assert_eq!(
        result.bool_field("ok"),
        Some(true),
        "clamp 模式写 Goal=5000 应成功: {result:?}"
    );
    let applied = applied_for_motor(&result, 1);
    if let Some(applied) = applied {
        assert!(
            applied <= 4085,
            "clamp 后 applied 应 <= 4095-soft_limit_margin(10)=4085: {result:?}"
        );
        assert!(applied >= 0, "clamp 后 applied 应 >= 0: {result:?}");
    }
    let clamped = clamped_for_motor(&result, 1);
    if let Some(clamped) = clamped {
        assert!(clamped, "clamped 标记应为 true: {result:?}");
    }
    let warnings: Vec<String> = result
        .array_field("warnings")
        .map(|arr| {
            arr.iter()
                .filter_map(Json::as_str)
                .map(|s| s.to_string())
                .collect()
        })
        .unwrap_or_default();
    assert!(
        applied.is_some() || !warnings.is_empty(),
        "软限位 clamp 应返回 applied 或 warnings（§6）: {result:?}"
    );
    if let Some(applied) = applied {
        let goal_state = wait_state(
            &mut client,
            Duration::from_secs(2),
            "state.goals[\"1\"] 等于 clamp 后的 applied",
            |s| motor_field(s, "goals", 1).and_then(Json::as_i64) == Some(applied),
        );
        let goal = motor_field(&goal_state, "goals", 1)
            .and_then(Json::as_i64)
            .expect("goals[1]");
        assert!(
            goal <= 4085,
            "state.goals 应为 clamp 后的值: {goal_state:?}"
        );
    } else {
        // 实现没回 applied 时，至少 state.goals 要是夹紧后的值
        let goal_state = wait_state(
            &mut client,
            Duration::from_secs(2),
            "state.goals[\"1\"] 已被 clamp（<=4085）",
            |s| {
                motor_field(s, "goals", 1)
                    .and_then(Json::as_i64)
                    .map(|v| v <= 4085)
                    .unwrap_or(false)
            },
        );
        assert!(
            !warnings.is_empty(),
            "既无 applied 也无 warnings，无法证明 clamp: {result:?} / {goal_state:?}"
        );
    }
    drop(daemon);

    // ---- reject 模式
    let reject_daemon = TestDaemon::spawn("soft-limit-reject", &sim_arm_json("reject", 0.1));
    let mut reject_client = reject_daemon.connect();
    wait_connected(&mut reject_client, Duration::from_secs(5));
    wait_state(
        &mut reject_client,
        Duration::from_secs(3),
        "reject 臂 limits[\"1\"] 已读到",
        |s| motor_field(s, "limits", 1).is_some(),
    );
    let err = reject_client.call_err(
        "arm.sync_write",
        r#"{"arm":"sim","address":116,"length":4,"values":{"1":5000}}"#,
    );
    assert_eq!(
        err.code, "soft_limit",
        "reject 模式越限应返回 soft_limit 错误: {err:?}"
    );
    // 拒绝后不应把越限目标写进状态
    let after = reject_client.state();
    if let Some(goal) = motor_field(&after, "goals", 1).and_then(Json::as_i64) {
        assert!(goal <= 4095, "reject 后 goals[1] 不应是越限值: {after:?}");
    }
    let _ = reject_daemon;
}

// ---------------------------------------------------------------- 测试 4：重连 20 次 + 传输失败释放 fd

/// §7 统计不变量：live 串口 handle 数恰好等于 `opens - closes`。
fn assert_handle_balance(stats: &Json, expected_live: u64, what: &str) {
    let opens = stats
        .u64_field("opens")
        .unwrap_or_else(|| panic!("{what}: arm.stats 缺少 opens: {stats:?}"));
    let closes = stats
        .u64_field("closes")
        .unwrap_or_else(|| panic!("{what}: arm.stats 缺少 closes: {stats:?}"));
    assert!(
        opens >= closes,
        "{what}: opens({opens}) 不应小于 closes({closes}): {stats:?}"
    );
    assert_eq!(
        opens - closes,
        expected_live,
        "{what}: opens({opens}) - closes({closes}) 应 = {expected_live}（live 串口 handle 数）: {stats:?}"
    );
}

#[test]
fn reconnect_cycles_keep_stream_and_balance_fds() {
    // reconnect_sec 调大到 2s："传输失败 → fd 已释放"的窗口才能稳定观测
    //（生产默认也远大于一次 RPC 往返；20 次手动 reconnect 不等这个周期，不受影响）。
    let daemon = TestDaemon::spawn("reconnect", &sim_arm_json("clamp", 2.0));
    let mut client = daemon.connect();
    let initial = wait_connected(&mut client, Duration::from_secs(5));
    let seq_before = initial.u64_field("seq").unwrap_or(0);

    let before_stats = client.stats();
    let opens_before = before_stats.u64_field("opens").unwrap_or(0);
    let closes_before = before_stats.u64_field("closes").unwrap_or(0);

    let sub = daemon.subscribe(&[ARM], 60.0);
    std::thread::sleep(Duration::from_millis(100));
    let _ = sub.drain();

    // 订阅活跃时连续 reconnect ×20：每个都必须成功（无 error 响应）
    let cycles = 20u32;
    for i in 0..cycles {
        let ack = client.reconnect();
        assert_eq!(
            ack.bool_field("ok"),
            Some(true),
            "第 {i} 次 arm.reconnect 应成功: {ack:?}\narmd 日志:\n{}",
            daemon.log_tail()
        );
    }

    // 重连后：connected 恢复、状态 seq 继续增长
    let recovered = wait_state(
        &mut client,
        Duration::from_secs(5),
        "20 次 reconnect 后 connected=true 且 seq 前进",
        |s| s.bool_field("connected") == Some(true) && s.u64_field("seq").unwrap_or(0) > seq_before,
    );
    assert_eq!(
        recovered.bool_field("connected"),
        Some(true),
        "重连后应 connected: {recovered:?}"
    );

    // 订阅流不断：等到 seq 超过恢复时刻的新状态帧
    let seq_now = recovered.u64_field("seq").unwrap_or(0);
    let frame = sub
        .wait_for(Duration::from_secs(2), |f| {
            is_state_event(f) && f.u64_field("seq").unwrap_or(0) > seq_now
        })
        .unwrap_or_else(|| {
            panic!(
                "reconnect ×{cycles} 后 2s 内订阅没有新状态帧（seq>{seq_now}）；armd 日志:\n{}",
                daemon.log_tail()
            )
        });
    assert_eq!(
        frame.bool_field("connected"),
        Some(true),
        "重连后的状态帧 connected 应为 true: {frame:?}"
    );

    // stats 稳态：恰好 1 个 live 串口 handle → opens - closes == 1
    let stats = client.stats();
    let opens = stats.u64_field("opens").unwrap_or(0);
    let closes = stats.u64_field("closes").unwrap_or(0);
    let reconnects = stats.u64_field("reconnects").unwrap_or(0);
    let delta_opens = opens.saturating_sub(opens_before);
    let delta_closes = closes.saturating_sub(closes_before);
    eprintln!(
        "[reconnect] stats opens={opens} closes={closes} reconnects={reconnects} \
         (delta opens={delta_opens} closes={delta_closes})"
    );
    assert!(
        delta_opens >= cycles as u64,
        "{cycles} 次 reconnect 后 stats.opens 至少增加 {cycles}（实际 +{delta_opens}）: {stats:?}"
    );
    assert!(
        delta_closes >= cycles as u64,
        "{cycles} 次 reconnect 后 stats.closes 至少增加 {cycles}（实际 +{delta_closes}）: {stats:?}"
    );
    assert_handle_balance(&stats, 1, "20 次 reconnect 稳态");

    // ---- 传输失败（模拟 USB 掉线）：worker 必须先 close 释放 fd，再重连 ----
    let seq_before_transport = client.state().u64_field("seq").unwrap_or(0);
    client.call_ok(
        "arm.sim.inject",
        r#"{"arm":"sim","kind":"transport_error"}"#,
    );
    let disconnected = wait_state(
        &mut client,
        Duration::from_secs(3),
        "transport_error 注入后 connected=false",
        |s| s.bool_field("connected") == Some(false),
    );
    eprintln!(
        "[transport] 掉线状态: connected={:?} error={:?}",
        disconnected.bool_field("connected"),
        disconnected.get("error")
    );
    // 关键断言：掉线时旧 handle 必须已经关闭（opens == closes，没有 fd 泄漏）
    let stats_while_down = client.stats();
    eprintln!(
        "[transport] 掉线时 stats opens={:?} closes={:?}",
        stats_while_down.u64_field("opens"),
        stats_while_down.u64_field("closes")
    );
    assert_handle_balance(
        &stats_while_down,
        0,
        "transport_error 掉线后（旧 fd 必须已释放，不能留 handle）",
    );

    // 文档建议的 clear 步骤：串口已关闭时注入可能返回 serial_error(Busy)，
    // 两种情况都接受：重连会打开一个干净的新 SimPort，注入的故障随之消失。
    match client.call("arm.sim.inject", r#"{"arm":"sim","kind":"clear"}"#) {
        Ok(v) => eprintln!("[transport] 掉线期间 clear 返回 {v:?}"),
        Err(e) => eprintln!("[transport] 掉线期间 clear 返回错误（预期：串口已关闭）: {e}"),
    }

    // 等自动重连（reconnect_sec=2s）：重新连接、恢复到 1 个 live handle、订阅流恢复
    let reconnected = wait_state(
        &mut client,
        Duration::from_secs(8),
        "transport_error 后自动重连 connected=true 且 seq 前进",
        |s| {
            s.bool_field("connected") == Some(true)
                && s.u64_field("seq").unwrap_or(0) > seq_before_transport
        },
    );
    eprintln!(
        "[transport] 已自动重连: seq={:?}",
        reconnected.u64_field("seq")
    );
    let stats_after_transport = client.stats();
    assert_handle_balance(&stats_after_transport, 1, "transport_error 重连后");

    let seq_reconnected = reconnected.u64_field("seq").unwrap_or(0);
    let frame = sub
        .wait_for(Duration::from_secs(2), |f| {
            is_state_event(f) && f.u64_field("seq").unwrap_or(0) > seq_reconnected
        })
        .unwrap_or_else(|| {
            panic!(
                "传输恢复后 2s 内订阅没有新状态帧（seq>{seq_reconnected}）；armd 日志:\n{}",
                daemon.log_tail()
            )
        });
    assert_eq!(
        frame.bool_field("connected"),
        Some(true),
        "传输恢复后的状态帧 connected 应为 true: {frame:?}"
    );
    let _ = daemon;
}

// ---------------------------------------------------------------- 测试 5：stall_exclude_motors

#[test]
fn stall_exclude_motors_never_triggers() {
    // 夹爪（motor 6）常被显式排除在堵转保护之外（避免夹住物体时误 torque OFF）
    let daemon = TestDaemon::spawn("stall-exclude", &sim_arm_json_excluding("clamp", 0.1, &[6]));
    let mut client = daemon.connect();
    wait_connected(&mut client, Duration::from_secs(5));

    let state = wait_state(
        &mut client,
        Duration::from_secs(2),
        "state.safety.stall_exclude_motors 包含 6",
        |s| {
            s.get("safety")
                .and_then(|safety| safety.array_field("stall_exclude_motors"))
                .map(|arr| arr.iter().filter_map(Json::as_i64).any(|id| id == 6))
                .unwrap_or(false)
        },
    );
    assert!(
        state.get("safety").is_some(),
        "arm.state 应包含 safety 摘要: {state:?}"
    );

    client.call_ok("arm.torque", r#"{"arm":"sim","enable":true,"motors":[6]}"#);
    let present = joint_pos(&client.state(), 6);
    let goal = present + 800;
    client.call_ok(
        "arm.sync_write",
        &format!(r#"{{"arm":"sim","address":116,"length":4,"values":{{"6":{goal}}}}}"#),
    );
    client.call_ok(
        "arm.sim.inject",
        r#"{"arm":"sim","kind":"stall","motors":[6]}"#,
    );

    // 250ms stall_timeout 之后仍不应触发（排除的电机永远不进入 StallTracker）
    std::thread::sleep(Duration::from_millis(900));
    let after = client.state();
    assert!(
        motor_field(&after, "stalls", 6).is_none(),
        "stall_exclude_motors=[6] 时电机 6 不应触发堵转: {after:?}"
    );
    assert_eq!(
        motor_field(&after, "torque", 6).and_then(Json::as_bool),
        Some(true),
        "被排除的电机 6 不应被自动 torque OFF: {after:?}"
    );
    assert!(
        after.u64_field("seq").unwrap_or(0) > state.u64_field("seq").unwrap_or(0),
        "排除测试期间状态流应继续推进"
    );
    let _ = daemon;
}

// ---------------------------------------------------------------- JSON 自检

#[test]
fn json_parser_smoke() {
    let text = r#"{"a":1,"b":-2.5e1,"c":[true,false,null,"x\"y\nz"],"d":{"e":"中文"},"f":0}"#;
    let json = parse_json(text).expect("解析失败");
    assert_eq!(json.i64_field("a"), Some(1));
    assert_eq!(json.get("b").and_then(Json::as_f64), Some(-25.0));
    let c = json.array_field("c").expect("c");
    assert_eq!(c[0].as_bool(), Some(true));
    assert_eq!(c[1].as_bool(), Some(false));
    assert_eq!(c[2], Json::Null);
    assert_eq!(c[3].as_str(), Some("x\"y\nz"));
    assert_eq!(json.get("d").and_then(|d| d.str_field("e")), Some("中文"));
    assert_eq!(json.i64_field("f"), Some(0));
    assert!(parse_json("").is_err());
    assert!(parse_json("{\"a\":}").is_err());
    assert!(parse_json("[1,]").is_err());
    // serde_json 对 60.0 输出 "60.0"；解析器要能吃下
    assert_eq!(
        parse_json("60.0").and_then(|j| Ok(j.as_f64())),
        Ok(Some(60.0))
    );
}
