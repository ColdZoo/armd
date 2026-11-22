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
fn default_true() -> bool {
    true
}
fn default_stall_timeout() -> u64 {
    400
}
fn default_stall_tolerance() -> i64 {
    60
}
fn default_stall_velocity() -> i64 {
    2
}
fn default_stall_margin() -> i64 {
    20
}
fn default_soft_limit_mode() -> String {
    "clamp".into()
}
fn default_soft_limit_margin() -> i64 {
    10
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
    /// 轮询的寄存器地址（默认：启用堵转保护时 128/8 同时读 velocity+position）
    #[serde(default)]
    pub poll_addr: Option<u16>,
    #[serde(default = "default_poll_len")]
    pub poll_len: u16,
    /// 每秒额外读一次电压/温度（Present_Input_Voltage 144 + Present_Temperature 146）
    #[serde(default)]
    pub monitor: bool,
    /// 舵机 id → 关节名（如 {"1": "shoulder_pan", ...}），用于标定 profile 与角度换算
    #[serde(default)]
    pub joint_names: std::collections::BTreeMap<String, String>,
    /// 标定档案路径（默认 ~/.config/armd/profiles/<arm-id>.json）
    #[serde(default)]
    pub profile: Option<String>,

    // ---------------------------------------------------------------- 安全层
    /// 堵转/限位保护总开关（最后一道防线，默认开启）
    #[serde(default = "default_true")]
    pub stall_protection: bool,
    /// Goal 与 Present 偏差持续超过多久判定堵转（毫秒，建议 300~500）
    #[serde(default = "default_stall_timeout")]
    pub stall_timeout_ms: u64,
    /// 判定堵转的 |Goal - Present| 阈值（counts）
    #[serde(default = "default_stall_tolerance")]
    pub stall_tolerance: i64,
    /// 视为静止的 |Present_Velocity| 阈值（X 系列单位 0.229 rpm）
    #[serde(default = "default_stall_velocity")]
    pub stall_velocity_epsilon: i64,
    /// 触发堵转时自动 torque OFF 该电机（默认开启）
    #[serde(default = "default_true")]
    pub auto_torque_off: bool,
    /// 堵转回退目标时离 EEPROM 限位内侧的余量（counts）
    #[serde(default = "default_stall_margin")]
    pub stall_recover_margin: i64,
    /// 不参与堵转自动保护的电机（例如夹爪 6：夹住物体时 Goal 与 Present 长期有差属正常）
    #[serde(default)]
    pub stall_exclude_motors: Vec<u8>,
    /// 写 Goal_Position 时用 EEPROM Min/Max Position Limit 做软限位
    #[serde(default = "default_true")]
    pub soft_limits: bool,
    /// "clamp"（夹紧且返回告警，默认）或 "reject"（返回 soft_limit 错误）
    #[serde(default = "default_soft_limit_mode")]
    pub soft_limit_mode: String,
    /// 软限位内侧余量（counts）
    #[serde(default = "default_soft_limit_margin")]
    pub soft_limit_margin: i64,
}

impl ArmConfig {
    pub fn poll_addr(&self) -> u16 {
        self.poll_addr.unwrap_or(crate::dxl::ADDR_PRESENT_POSITION)
    }

    /// 实际轮询窗口 (addr, len)。
    ///
    /// - 用户显式配置了 `poll_addr`：保持旧语义（把读到的值当 Present_Position），
    ///   velocity 未知 → 堵转检测自动跳过。
    /// - 未显式配置且 `stall_protection`：用 128/8 一个 sync_read 同时取
    ///   Present_Velocity + Present_Position。
    /// - 未显式配置且关闭堵转保护：维持 132/4。
    pub fn poll_window(&self) -> (u16, u16) {
        match self.poll_addr {
            Some(addr) => (addr, self.poll_len),
            None if self.stall_protection => (crate::dxl::ADDR_PRESENT_VELOCITY, 8),
            None => (crate::dxl::ADDR_PRESENT_POSITION, self.poll_len),
        }
    }

    /// 轮询窗口里是否包含 Present_Velocity（决定堵转检测能否工作）。
    pub fn poll_has_velocity(&self) -> bool {
        let (addr, len) = self.poll_window();
        addr <= crate::dxl::ADDR_PRESENT_VELOCITY
            && u32::from(addr) + u32::from(len) >= 132
            && len >= 8
    }

    pub fn poll_interval(&self) -> std::time::Duration {
        let hz = if self.poll_hz.is_finite() && self.poll_hz > 0.1 { self.poll_hz } else { 60.0 };
        std::time::Duration::from_secs_f64(1.0 / hz.min(500.0))
    }

    /// 堵转检测是否真的会生效（配置开启 + 轮询窗口带 velocity）。
    pub fn stall_detection_active(&self) -> bool {
        self.stall_protection && self.poll_has_velocity()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn arm(motors: Vec<u8>) -> ArmConfig {
        ArmConfig {
            id: "sim".into(),
            label: String::new(),
            port: String::new(),
            baud: 1_000_000,
            poll_hz: 60.0,
            read_timeout_ms: 30,
            reconnect_sec: 1.0,
            sim: true,
            motors,
            poll_addr: None,
            poll_len: 4,
            monitor: false,
            joint_names: Default::default(),
            profile: None,
            stall_protection: true,
            stall_timeout_ms: 400,
            stall_tolerance: 60,
            stall_velocity_epsilon: 2,
            auto_torque_off: true,
            stall_recover_margin: 20,
            stall_exclude_motors: Vec::new(),
            soft_limits: true,
            soft_limit_mode: "clamp".into(),
            soft_limit_margin: 10,
        }
    }

    #[test]
    fn poll_window_defaults_to_velocity_and_position() {
        let a = arm(vec![1, 2]);
        assert_eq!(a.poll_window(), (128, 8));
        assert!(a.poll_has_velocity());
        assert!(a.stall_detection_active());
    }

    #[test]
    fn poll_window_old_style_when_stall_disabled() {
        let mut a = arm(vec![1]);
        a.stall_protection = false;
        assert_eq!(a.poll_window(), (132, 4));
        assert!(!a.poll_has_velocity());
    }

    #[test]
    fn explicit_poll_addr_disables_velocity() {
        let mut a = arm(vec![1]);
        a.poll_addr = Some(132);
        assert_eq!(a.poll_window(), (132, 4));
        assert!(!a.poll_has_velocity(), "显式 poll_addr 保持旧语义，velocity 未知");
        assert!(!a.stall_detection_active());
    }

    #[test]
    fn config_parses_without_new_fields() {
        // 旧配置（v0.1.0）必须继续可用；新字段全部有默认值
        let text = r#"{"socket":"/tmp/x.sock","arms":[{"id":"a","port":"/dev/null","motors":[1,2]}]}"#;
        let cfg: Config = serde_json::from_str(text).unwrap();
        let a = &cfg.arms[0];
        assert!(a.stall_protection);
        assert_eq!(a.stall_timeout_ms, 400);
        assert_eq!(a.stall_tolerance, 60);
        assert_eq!(a.stall_velocity_epsilon, 2);
        assert!(a.auto_torque_off);
        assert_eq!(a.stall_recover_margin, 20);
        assert!(a.stall_exclude_motors.is_empty());
        assert!(a.soft_limits);
        assert_eq!(a.soft_limit_mode, "clamp");
        assert_eq!(a.soft_limit_margin, 10);
    }
}
