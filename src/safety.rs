//! 安全层纯逻辑：硬件错误位命名、EEPROM 软限位、堵转检测。
//!
//! 这里不碰串口/线程/时间源之外的东西，方便单元测试。`arm.rs` 的 worker 调用：
//! - 写 Goal_Position 前用 [`apply_soft_limit`] 夹紧/拒绝；
//! - 每轮轮询后对每个电机调用 [`StallTracker::observe`]，触发时 worker 执行
//!   torque OFF + goal 回退 + 上报事件。
//!
//! 设计背景：从臂肘部顶住机械限位持续输出力矩 → 舵机 latch hardware error 0x80，
//! v0.1.0 没有最后一道防线，整条臂被拖垮。详见 `docs/SAFETY-DESIGN.md`。

use std::collections::{HashMap, HashSet};
use std::time::{Duration, Instant};

/// Position Control Mode(3) 的绝对位置范围（单圈 0..=4095）。
pub const POSITION_MIN: i64 = 0;
pub const POSITION_MAX: i64 = 4095;
/// Extended Position Control Mode(4) 的绝对位置范围（多圈 ±1,048,575）。
pub const EXTENDED_POSITION_MIN: i64 = -1_048_575;
pub const EXTENDED_POSITION_MAX: i64 = 1_048_575;

/// 状态包 error 字节 / Hardware_Error_Status(70) 的 bit 含义（X 系列）。
/// bit7（0x80）是 alert：舵机已 latch 硬件错误，必须 Reboot 或断电才能清除。
const KNOWN_ERROR_BITS: [(u8, &str); 5] = [
    (0x01, "input_voltage"),
    (0x04, "overheating"),
    (0x08, "motor_encoder"),
    (0x10, "electrical_shock"),
    (0x20, "overload"),
];

const RESERVED_BIT_NAMES: [&str; 8] = [
    "reserved_bit_0",
    "reserved_bit_1",
    "reserved_bit_2",
    "reserved_bit_3",
    "reserved_bit_4",
    "reserved_bit_5",
    "reserved_bit_6",
    "reserved_bit_7",
];

/// 把 error 字节翻译成人类可读的 bit 名列表（0x80 在最前，未知 bit 标 reserved）。
pub fn hardware_error_names(code: u8) -> Vec<&'static str> {
    let mut names = Vec::new();
    if code & 0x80 != 0 {
        names.push("hardware_alert");
    }
    for (bit, name) in KNOWN_ERROR_BITS {
        if code & bit != 0 {
            names.push(name);
        }
    }
    let known_mask = 0x80 | KNOWN_ERROR_BITS.iter().fold(0u8, |acc, (bit, _)| acc | bit);
    let reserved = code & !known_mask;
    for (bit, name) in RESERVED_BIT_NAMES.iter().enumerate() {
        if reserved & (1u8 << bit) != 0 {
            names.push(name);
        }
    }
    names
}

/// 面向人的错误说明。latched（0x80）时明确提示用 `arm.reboot` 清除。
pub fn hardware_error_message(code: u8) -> String {
    let names = hardware_error_names(code);
    let names = if names.is_empty() {
        "unknown".to_string()
    } else {
        names.join("|")
    };
    if code & 0x80 != 0 {
        format!(
            "hardware error 0x{code:02X} ({names})：舵机已 latch 保护状态，需 arm.reboot 清除（或物理断电）"
        )
    } else {
        format!("hardware error 0x{code:02X} ({names})")
    }
}

// ---------------------------------------------------------------- 软限位

#[derive(Debug, Clone)]
pub struct SoftLimitConfig {
    /// 是否启用 EEPROM 软限位（绝对 0..4095 永远生效）
    pub enabled: bool,
    /// 离 EEPROM 限位内侧保留的余量（counts）
    pub margin: i64,
}

impl Default for SoftLimitConfig {
    fn default() -> Self {
        Self { enabled: true, margin: 10 }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SoftLimitOutcome {
    /// 实际会下发的目标
    pub applied: i64,
    /// 调用方请求的目标
    pub requested: i64,
    pub clamped: bool,
    /// EEPROM 生效限位（读到且合法时才有）
    pub min: Option<i64>,
    pub max: Option<i64>,
    pub warnings: Vec<String>,
}

/// Goal_Position 软限位（**按 Operating_Mode 区分**）：
/// - Position Control Mode(3) / 未知：先夹紧到协议绝对范围 `0..=4095`；`enabled` 且
///   EEPROM 限位合法时再夹紧到 `[min+margin, max-margin]`（与舵机自身对 Goal 的
///   限位校验一致，且把"静默拒绝"变成显式告警）；
/// - Extended Position Control Mode(4)（主臂就是这种）：绝对范围是 ±1,048,575，
///   **不应用** EEPROM Min/Max Position Limit（该限位只在单圈位置模式生效）；
/// - 限位未知（EEPROM 没读到）时不阻止写入，只做绝对范围夹紧（状态里
///   `limits` 缺失 + 启动日志；不静默）。
pub fn apply_soft_limit(
    motor: u8,
    value: i64,
    limits: Option<(i64, i64)>,
    mode: Option<u16>,
    cfg: &SoftLimitConfig,
) -> SoftLimitOutcome {
    let requested = value;
    let extended = mode == Some(4);
    let (abs_min, abs_max) = if extended {
        (EXTENDED_POSITION_MIN, EXTENDED_POSITION_MAX)
    } else {
        (POSITION_MIN, POSITION_MAX)
    };
    let mut applied = value.clamp(abs_min, abs_max);
    let mut warnings = Vec::new();
    if applied != value {
        warnings.push(format!(
            "motor {motor} 目标 {value} 超出绝对范围 {abs_min}..{abs_max}（mode={mode:?}），已夹紧到 {applied}"
        ));
    }

    let mut min = None;
    let mut max = None;
    // 扩展位置模式下 EEPROM 位置限位不生效，跳过（避免把多圈目标夹回单圈范围）
    if cfg.enabled && !extended {
        if let Some((lo, hi)) = limits {
            if lo < hi {
                min = Some(lo);
                max = Some(hi);
                let margin = cfg.margin.max(0);
                let (eff_lo, eff_hi) = {
                    let a = lo + margin;
                    let b = hi - margin;
                    if a <= b {
                        (a, b)
                    } else {
                        // 行程比余量还窄：取中点，避免把目标夹到非法区间
                        let mid = (lo + hi) / 2;
                        (mid, mid)
                    }
                };
                let clamped = applied.clamp(eff_lo, eff_hi);
                if clamped != applied {
                    warnings.push(format!(
                        "motor {motor} 目标 {applied} 超出软限位 [{eff_lo}..{eff_hi}]（EEPROM [{lo}..{hi}]，余量 {margin}），已夹紧到 {clamped}"
                    ));
                    applied = clamped;
                }
            } else {
                warnings.push(format!(
                    "motor {motor} 的 EEPROM 限位非法（min={lo} >= max={hi}），跳过软限位"
                ));
            }
        }
    }

    SoftLimitOutcome { applied, requested, clamped: applied != requested, min, max, warnings }
}

// ---------------------------------------------------------------- 堵转检测

#[derive(Debug, Clone)]
pub struct StallConfig {
    pub enabled: bool,
    /// 条件持续多久触发
    pub timeout: Duration,
    /// |Goal - Present| 阈值（counts）
    pub tolerance: i64,
    /// 视为静止的 |Present_Velocity| 阈值（raw 单位，0.229rpm/单位）
    pub velocity_epsilon: i64,
}

impl Default for StallConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            timeout: Duration::from_millis(400),
            tolerance: 60,
            velocity_epsilon: 2,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StallTrigger {
    pub motor: u8,
    pub position: i64,
    pub goal: i64,
    pub velocity: i64,
    pub stall_ms: u64,
}

/// 每电机堵转计时器。条件：
/// `|present - goal| > tolerance && |velocity| <= velocity_epsilon && torque_on == Some(true)`
/// 连续成立超过 `timeout` → 触发一次（之后必须 reset / 条件消失才会重新计时）。
#[derive(Debug)]
pub struct StallTracker {
    cfg: StallConfig,
    since: HashMap<u8, Instant>,
    fired: HashSet<u8>,
}

impl StallTracker {
    pub fn new(cfg: StallConfig) -> Self {
        Self { cfg, since: HashMap::new(), fired: HashSet::new() }
    }

    pub fn config(&self) -> &StallConfig {
        &self.cfg
    }

    /// 每轮轮询调用。触发时返回 Some（同一段持续条件只会返回一次）。
    pub fn observe(
        &mut self,
        motor: u8,
        present: i64,
        velocity: Option<i64>,
        goal: Option<i64>,
        torque_on: Option<bool>,
        now: Instant,
    ) -> Option<StallTrigger> {
        let active = match (self.cfg.enabled, velocity, goal, torque_on) {
            (true, Some(v), Some(g), Some(true)) => {
                (present - g).abs() > self.cfg.tolerance && v.abs() <= self.cfg.velocity_epsilon
            }
            _ => false,
        };
        if !active {
            self.since.remove(&motor);
            self.fired.remove(&motor);
            return None;
        }
        if self.fired.contains(&motor) {
            return None;
        }
        let since = *self.since.entry(motor).or_insert(now);
        let elapsed = now.saturating_duration_since(since);
        if elapsed < self.cfg.timeout {
            return None;
        }
        self.fired.insert(motor);
        self.since.remove(&motor);
        Some(StallTrigger {
            motor,
            position: present,
            goal: goal.unwrap_or(present),
            velocity: velocity.unwrap_or(0),
            stall_ms: elapsed.as_millis() as u64,
        })
    }

    /// 显式清除某电机的计时与"已触发"标记（上层 clear_error / 重新使能 / 新目标时用）。
    pub fn reset(&mut self, motor: u8) {
        self.since.remove(&motor);
        self.fired.remove(&motor);
    }

    pub fn reset_all(&mut self) {
        self.since.clear();
        self.fired.clear();
    }

    /// 当前正在累计的堵转计时起点（调试/状态用）。
    pub fn active_since(&self, motor: u8) -> Option<Instant> {
        self.since.get(&motor).copied()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ------------------------------------------------------------ 错误位

    #[test]
    fn error_names_alert_and_known_bits() {
        assert_eq!(hardware_error_names(0x80), vec!["hardware_alert"]);
        assert_eq!(hardware_error_names(0x20), vec!["overload"]);
        assert_eq!(
            hardware_error_names(0x01 | 0x04 | 0x80),
            vec!["hardware_alert", "input_voltage", "overheating"]
        );
        assert_eq!(hardware_error_names(0x00), Vec::<&str>::new());
    }

    #[test]
    fn error_names_reserved_bits() {
        // 0x42 = reserved bit6 + reserved bit1
        assert_eq!(hardware_error_names(0x42), vec!["reserved_bit_1", "reserved_bit_6"]);
    }

    #[test]
    fn error_message_mentions_reboot_for_latched() {
        let msg = hardware_error_message(0x80);
        assert!(msg.contains("arm.reboot"), "{msg}");
        assert!(msg.contains("0x80"), "{msg}");
        let msg2 = hardware_error_message(0x20);
        assert!(msg2.contains("overload"), "{msg2}");
        assert!(!msg2.contains("arm.reboot"), "非 latched 错误不必提示 reboot: {msg2}");
    }

    // ------------------------------------------------------------ 软限位

    fn sl(enabled: bool, margin: i64) -> SoftLimitConfig {
        SoftLimitConfig { enabled, margin }
    }

    #[test]
    fn soft_limit_within_limits_untouched() {
        let out = apply_soft_limit(3, 2000, Some((1607, 3309)), None, &sl(true, 10));
        assert_eq!(out.applied, 2000);
        assert!(!out.clamped);
        assert_eq!((out.min, out.max), (Some(1607), Some(3309)));
        assert!(out.warnings.is_empty());
    }

    #[test]
    fn soft_limit_clamps_into_eeprom_range() {
        // 已知坑：肘部 EEPROM Max=3309，上层写 3386 会被舵机静默拒绝
        let out = apply_soft_limit(3, 3386, Some((1607, 3309)), Some(3), &sl(true, 10));
        assert_eq!(out.applied, 3299);
        assert!(out.clamped);
        assert_eq!(out.requested, 3386);
        assert_eq!(out.warnings.len(), 1);
        assert!(out.warnings[0].contains("软限位"), "{:?}", out.warnings);

        let low = apply_soft_limit(3, 1500, Some((1607, 3309)), Some(3), &sl(true, 10));
        assert_eq!(low.applied, 1617);
        assert!(low.clamped);
    }

    #[test]
    fn soft_limit_absolute_range_always_applies() {
        let out = apply_soft_limit(1, 5000, None, Some(3), &sl(false, 10));
        assert_eq!(out.applied, POSITION_MAX);
        assert!(out.clamped);
        assert!(out.warnings.iter().any(|w| w.contains("绝对范围")), "{:?}", out.warnings);
        let neg = apply_soft_limit(1, -5, None, Some(3), &sl(false, 10));
        assert_eq!(neg.applied, POSITION_MIN);
    }

    #[test]
    fn soft_limit_unknown_limits_only_absolute() {
        let out = apply_soft_limit(3, 4000, None, Some(3), &sl(true, 10));
        assert_eq!(out.applied, 4000, "限位未知时不阻止写入（由状态暴露 limits 缺失）");
        assert!(!out.clamped);
        assert!(out.warnings.is_empty());
        assert_eq!((out.min, out.max), (None, None));
    }

    #[test]
    fn soft_limit_extended_mode_allows_multiturn() {
        // 主臂（mode=4）：present 可以是 6133 / -856；不能按 0..4095 夹
        let out = apply_soft_limit(1, 6133, Some((0, 4084)), Some(4), &sl(true, 10));
        assert_eq!(out.applied, 6133);
        assert!(!out.clamped);
        let neg = apply_soft_limit(5, -2889, Some((0, 4095)), Some(4), &sl(true, 10));
        assert_eq!(neg.applied, -2889);
        assert!(!neg.clamped);
        // 但扩展模式的绝对范围仍然生效
        let huge = apply_soft_limit(1, 9_999_999, None, Some(4), &sl(true, 10));
        assert_eq!(huge.applied, EXTENDED_POSITION_MAX);
        assert!(huge.clamped);
    }

    #[test]
    fn soft_limit_invalid_limits_skipped_with_warning() {
        let out = apply_soft_limit(3, 2000, Some((3309, 1607)), Some(3), &sl(true, 10));
        assert_eq!(out.applied, 2000);
        assert!(out.warnings[0].contains("非法"), "{:?}", out.warnings);
    }

    #[test]
    fn soft_limit_narrow_range_uses_midpoint() {
        let out = apply_soft_limit(3, 1000, Some((2000, 2005)), Some(3), &sl(true, 10));
        assert_eq!(out.applied, 2002, "行程比余量窄时取中点，不能返回非法区间");
        assert!(out.clamped);
    }

    // ------------------------------------------------------------ 堵转

    fn tracker(timeout_ms: u64) -> StallTracker {
        StallTracker::new(StallConfig {
            enabled: true,
            timeout: Duration::from_millis(timeout_ms),
            tolerance: 60,
            velocity_epsilon: 2,
        })
    }

    #[test]
    fn stall_triggers_after_timeout_when_holding_against_limit() {
        let mut t = tracker(400);
        let t0 = Instant::now();
        // 目标 3386，实际卡在 3300，速度 0，torque ON
        assert_eq!(t.observe(3, 3300, Some(0), Some(3386), Some(true), t0), None);
        assert_eq!(
            t.observe(3, 3300, Some(0), Some(3386), Some(true), t0 + Duration::from_millis(250)),
            None
        );
        let trig = t
            .observe(3, 3300, Some(0), Some(3386), Some(true), t0 + Duration::from_millis(410))
            .expect("400ms 后应触发");
        assert_eq!(trig.motor, 3);
        assert_eq!(trig.position, 3300);
        assert_eq!(trig.goal, 3386);
        assert!(trig.stall_ms >= 400);
        // 条件持续也只触发一次
        assert_eq!(
            t.observe(3, 3300, Some(0), Some(3386), Some(true), t0 + Duration::from_millis(900)),
            None
        );
    }

    #[test]
    fn stall_not_triggered_while_moving() {
        let mut t = tracker(200);
        let t0 = Instant::now();
        for i in 0..40 {
            let now = t0 + Duration::from_millis(i * 20);
            // 速度不为 0（在移动），偏差再大也不算堵转
            assert_eq!(t.observe(3, 3000, Some(9), Some(3386), Some(true), now), None);
        }
    }

    #[test]
    fn stall_not_triggered_when_torque_off_or_unknown() {
        let mut t = tracker(100);
        let t0 = Instant::now();
        for i in 0..20 {
            let now = t0 + Duration::from_millis(i * 20);
            assert_eq!(t.observe(3, 3300, Some(0), Some(3386), Some(false), now), None);
            assert_eq!(t.observe(3, 3300, Some(0), Some(3386), None, now), None);
        }
    }

    #[test]
    fn stall_resets_when_condition_clears() {
        let mut t = tracker(200);
        let t0 = Instant::now();
        assert_eq!(t.observe(3, 3300, Some(0), Some(3386), Some(true), t0), None);
        // 中间动了一下 → 计时清零
        assert_eq!(
            t.observe(3, 3300, Some(50), Some(3386), Some(true), t0 + Duration::from_millis(150)),
            None
        );
        assert_eq!(
            t.observe(3, 3300, Some(0), Some(3386), Some(true), t0 + Duration::from_millis(300)),
            None,
            "条件中途消失后要重新累计"
        );
        let trig = t.observe(3, 3300, Some(0), Some(3386), Some(true), t0 + Duration::from_millis(520));
        assert!(trig.is_some());
    }

    #[test]
    fn stall_can_retrigger_after_reset() {
        let mut t = tracker(100);
        let t0 = Instant::now();
        assert_eq!(t.observe(3, 3300, Some(0), Some(3386), Some(true), t0), None);
        assert!(t.observe(3, 3300, Some(0), Some(3386), Some(true), t0 + Duration::from_millis(120)).is_some());
        t.reset(3);
        assert!(
            t.observe(3, 3300, Some(0), Some(3386), Some(true), t0 + Duration::from_millis(140)).is_none(),
            "reset 后重新计时"
        );
        assert!(t.observe(3, 3300, Some(0), Some(3386), Some(true), t0 + Duration::from_millis(260)).is_some());
    }

    #[test]
    fn stall_tolerance_boundary() {
        let mut t = tracker(100);
        let t0 = Instant::now();
        // 偏差恰好 60（不大于阈值）→ 不算堵转
        assert_eq!(t.observe(3, 3300, Some(0), Some(3360), Some(true), t0), None);
        for i in 0..10 {
            let now = t0 + Duration::from_millis(i * 20);
            assert_eq!(t.observe(3, 3300, Some(0), Some(3360), Some(true), now), None);
        }
    }

    #[test]
    fn stall_disabled_never_triggers() {
        let mut t = StallTracker::new(StallConfig {
            enabled: false,
            timeout: Duration::from_millis(50),
            tolerance: 60,
            velocity_epsilon: 2,
        });
        let t0 = Instant::now();
        for i in 0..10 {
            let now = t0 + Duration::from_millis(i * 20);
            assert_eq!(t.observe(3, 3300, Some(0), Some(3386), Some(true), now), None);
        }
    }

    #[test]
    fn stall_without_velocity_never_triggers() {
        let mut t = tracker(50);
        let t0 = Instant::now();
        for i in 0..10 {
            let now = t0 + Duration::from_millis(i * 20);
            assert_eq!(t.observe(3, 3300, None, Some(3386), Some(true), now), None);
        }
    }
}
