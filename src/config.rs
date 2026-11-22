//! 守护进程配置：一条串口总线 = 一个机械臂（arm）。

use serde::{Deserialize, Serialize};

fn default_socket() -> String {
    "/tmp/armd.sock".into()
}
fn default_baud() -> u32 {
    1_000_000
}
fn default_rate() -> f64 {
    60.0
}
fn default_read_timeout() -> u64 {
    30
}
fn default_reconnect() -> f64 {
    2.0
}
fn default_ids() -> Vec<u8> {
    vec![1, 2, 3, 4, 5, 6]
}
fn default_poll_len() -> u16 {
    4
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct Config {
    /// 兼容旧配置：Unix socket 路径
    #[serde(default = "default_socket")]
    pub socket: String,
    /// 监听地址列表，例如 ["unix:/tmp/armd.sock", "tcp://0.0.0.0:8771"]
    /// 为空时等价于 ["unix:<socket>"]
    #[serde(default)]
    pub listen: Vec<String>,
    /// 可选访问令牌：设置后客户端必须先发 `hello` 携带 token
    #[serde(default)]
    pub token: Option<String>,
    /// 身份文件路径（默认 /tmp/armd-identity.json；/run/armd 可写时优先那里）
    #[serde(default)]
    pub identity: Option<String>,
    pub arms: Vec<ArmConfig>,
}

impl Config {
    /// 实际监听地址（`socket` 作为向后兼容的默认值）
    pub fn listen_specs(&self) -> Vec<String> {
        if self.listen.is_empty() {
            vec![format!("unix:{}", self.socket)]
        } else {
            self.listen.clone()
        }
    }
}

impl Config {
    pub fn from_file(path: &str) -> Result<Self, String> {
        let text = std::fs::read_to_string(path).map_err(|e| format!("read {path}: {e}"))?;
        serde_json::from_str(&text).map_err(|e| format!("parse {path}: {e}"))
    }
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct ArmConfig {
    /// 对外 arm id，例如 `koch_v1_1:follower`
    pub id: String,
    #[serde(default)]
    pub label: String,
    /// 串口路径，例如 `/dev/cu.usbmodem5A7C1190831`；sim 模式可空
    #[serde(default)]
    pub port: String,
    #[serde(default = "default_baud")]
    pub baud: u32,
    /// 轮询频率（Hz）
    #[serde(default = "default_rate")]
    pub poll_hz: f64,
    /// 一次 sync_read 的超时（毫秒）
    #[serde(default = "default_read_timeout")]
    pub read_timeout_ms: u64,
    /// 断线后重连间隔（秒）
    #[serde(default = "default_reconnect")]
    pub reconnect_sec: f64,
    #[serde(default)]
    pub sim: bool,
    #[serde(default = "default_ids")]
    pub motors: Vec<u8>,
    /// 轮询的寄存器地址（默认 Present_Position）
    #[serde(default)]
    pub poll_addr: Option<u16>,
    #[serde(default = "default_poll_len")]
    pub poll_len: u16,
    /// 每秒额外读一次电压/温度（Present_Input_Voltage 144 + Present_Temperature 146）
    #[serde(default)]
    pub monitor: bool,
}

impl ArmConfig {
    pub fn poll_addr(&self) -> u16 {
        self.poll_addr.unwrap_or(crate::dxl::ADDR_PRESENT_POSITION)
    }
    pub fn poll_interval(&self) -> std::time::Duration {
        let hz = if self.poll_hz.is_finite() && self.poll_hz > 0.1 { self.poll_hz } else { 60.0 };
        std::time::Duration::from_secs_f64(1.0 / hz.min(500.0))
    }
}
