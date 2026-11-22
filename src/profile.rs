//! 机械臂标定档案（profile）：零位 / 方向 / 夹爪行程，由 armd 持有。
//!
//! 设计目标：**标定数据的唯一事实源在驱动进程**，任何应用（zeropose、遥操作、脚本）
//! 都通过 RPC 读写，不再各自维护一份 JSON。schema 与 zeropose 兼容：
//!
//! ```json
//! {
//!   "zero": {"shoulder_pan": 2003, ...},   // 每个关节的 raw 零位
//!   "sign": {"shoulder_pan": -1, ...},     // 方向 ±1
//!   "grip": {"open": 3293, "closed": 2047, "angle": 90.0},
//!   "spin": {"wrist_roll": 0.0},           // 可选
//!   "savedAt": 1795321780
//! }
//! ```

use serde_json::{json, Map, Value};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

const HISTORY_KEEP: usize = 20;

fn now_secs() -> i64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs() as i64).unwrap_or(0)
}

fn check_raw(name: &str, value: Option<&Value>) -> Result<i64, String> {
    let v = value.ok_or_else(|| format!("缺少 {name}"))?;
    let n = v.as_i64().ok_or_else(|| format!("{name} 必须是整数 raw 值"))?;
    if !(0..=4095).contains(&n) {
        return Err(format!("{name} 超出 0..4095: {n}"));
    }
    Ok(n)
}

/// 校验并标准化 profile。`joint_names`（motor_id -> joint name）用于检查覆盖与计算。
pub fn validate(payload: &Value, joint_names: &BTreeMap<String, String>) -> Result<Value, String> {
    let obj = payload.as_object().ok_or("profile 必须是 JSON 对象")?;
    let zero = obj.get("zero").and_then(|v| v.as_object()).ok_or("缺少 zero 对象")?;
    let sign = obj.get("sign").and_then(|v| v.as_object()).ok_or("缺少 sign 对象")?;
    let grip = obj.get("grip").and_then(|v| v.as_object()).ok_or("缺少 grip 对象")?;

    let mut clean_zero = Map::new();
    let mut clean_sign = Map::new();
    // 只校验配置里声明过的关节；没声明 joint_names 时按 profile 自己给的关节校验
    let joints: Vec<String> = if joint_names.is_empty() {
        zero.keys().cloned().collect()
    } else {
        joint_names.values().cloned().collect()
    };
    for j in &joints {
        let v = check_raw(&format!("zero.{j}"), zero.get(j))?;
        clean_zero.insert(j.clone(), json!(v));
        let s = sign.get(j).and_then(|v| v.as_i64()).unwrap_or(1);
        if s != 1 && s != -1 {
            return Err(format!("sign.{j} 必须是 +1 或 -1"));
        }
        clean_sign.insert(j.clone(), json!(s));
    }

    let open = check_raw("grip.open", grip.get("open"))?;
    let closed = check_raw("grip.closed", grip.get("closed"))?;
    if (open - closed).abs() < 100 {
        return Err("夹爪张开/闭合相差太小（<100 counts），无法区分行程".into());
    }
    let mut clean_grip = Map::new();
    clean_grip.insert("open".into(), json!(open.max(closed)));
    clean_grip.insert("closed".into(), json!(closed.min(open)));
    if let Some(angle) = grip.get("angle").and_then(|v| v.as_f64()) {
        if (5.0..=140.5).contains(&angle) {
            clean_grip.insert("angle".into(), json!((angle * 10.0).round() / 10.0));
        }
    }

    let mut clean = Map::new();
    clean.insert("zero".into(), Value::Object(clean_zero));
    clean.insert("sign".into(), Value::Object(clean_sign));
    clean.insert("grip".into(), Value::Object(clean_grip));
    if let Some(spin) = obj.get("spin").and_then(|v| v.as_object()) {
        let mut clean_spin = Map::new();
        for (k, v) in spin {
            if let Some(deg) = v.as_f64() {
                if (-180.0..=180.0).contains(&deg) {
                    clean_spin.insert(k.clone(), json!((deg * 10.0).round() / 10.0));
                }
            }
        }
        if !clean_spin.is_empty() {
            clean.insert("spin".into(), Value::Object(clean_spin));
        }
    }
    let saved_at = obj.get("savedAt").and_then(|v| v.as_i64()).unwrap_or_else(now_secs);
    clean.insert("savedAt".into(), json!(saved_at));
    Ok(Value::Object(clean))
}

/// 用 profile 计算关节角度（度）。夹爪按 grip.open/closed 线性插值，`angle` 缺省 137.5°。
pub fn joint_degrees(
    raw: &BTreeMap<u8, i64>,
    joint_names: &BTreeMap<String, String>,
    profile: &Value,
) -> BTreeMap<String, f64> {
    let mut out = BTreeMap::new();
    let zero = profile.get("zero").and_then(|v| v.as_object());
    let sign = profile.get("sign").and_then(|v| v.as_object());
    let grip = profile.get("grip").and_then(|v| v.as_object());
    let Some(zero) = zero else { return out };
    let Some(sign) = sign else { return out };

    for (motor_id, joint) in joint_names {
        let Ok(id) = motor_id.parse::<u8>() else { continue };
        let Some(raw_v) = raw.get(&id) else { continue };
        if joint == "gripper" {
            if let Some(grip) = grip {
                let open = grip.get("open").and_then(|v| v.as_i64()).unwrap_or(0) as f64;
                let closed = grip.get("closed").and_then(|v| v.as_i64()).unwrap_or(0) as f64;
                let angle = grip.get("angle").and_then(|v| v.as_f64()).unwrap_or(137.5);
                let span = (open - closed).abs();
                if span > 1.0 {
                    let frac = ((*raw_v as f64 - closed) / span).clamp(0.0, 1.0);
                    out.insert(joint.clone(), frac * angle);
                }
            }
            continue;
        }
        let z = zero.get(joint).and_then(|v| v.as_i64()).unwrap_or(2048);
        let s = sign.get(joint).and_then(|v| v.as_i64()).unwrap_or(1) as f64;
        // 连续关节：wrap 到 ±2048 再换算，避免 ±180° 跳变
        let mut diff = (*raw_v - z) % 4096;
        if diff > 2048 {
            diff -= 4096;
        } else if diff < -2048 {
            diff += 4096;
        }
        out.insert(joint.clone(), diff as f64 * 360.0 / 4096.0 * s);
    }
    out
}

/// profile 文件存储（每个 arm 一个文件；原子写）。
pub struct ProfileStore {
    path: PathBuf,
}

impl ProfileStore {
    pub fn new(path: impl Into<PathBuf>) -> Self {
        Self { path: path.into() }
    }

    pub fn default_path(arm_id: &str) -> PathBuf {
        let safe: String = arm_id
            .chars()
            .map(|c| if c.is_ascii_alphanumeric() || c == '-' || c == '_' || c == '.' { c } else { '-' })
            .collect();
        let home = std::env::var("HOME").unwrap_or_else(|_| "/tmp".into());
        PathBuf::from(format!("{home}/.config/armd/profiles/{safe}.json"))
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    pub fn load(&self) -> Option<Value> {
        let text = std::fs::read_to_string(&self.path).ok()?;
        serde_json::from_str(&text).ok()
    }

    /// 历史目录：`<profile 同目录>/history/<stem>-<unix>.json`
    pub fn history_dir(&self) -> PathBuf {
        self.path.parent().map(|p| p.join("history")).unwrap_or_else(|| PathBuf::from("history"))
    }

    fn snapshot_history(&self) {
        let old = match std::fs::read_to_string(&self.path) {
            Ok(text) => text,
            Err(_) => return,
        };
        let dir = self.history_dir();
        let _ = std::fs::create_dir_all(&dir);
        let stem = self
            .path
            .file_stem()
            .map(|s| s.to_string_lossy().to_string())
            .unwrap_or_else(|| "profile".into());
        let ts = SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0);
        let dest = dir.join(format!("{stem}-{ts}.json"));
        if std::fs::write(&dest, old).is_err() {
            return;
        }
        // 只保留最近 HISTORY_KEEP 份
        if let Ok(entries) = std::fs::read_dir(&dir) {
            let mut names: Vec<(String, PathBuf)> = entries
                .flatten()
                .filter_map(|e| {
                    let name = e.file_name().to_string_lossy().to_string();
                    if name.starts_with(&format!("{stem}-")) && name.ends_with(".json") {
                        Some((name.clone(), e.path()))
                    } else {
                        None
                    }
                })
                .collect();
            names.sort_by(|a, b| a.0.cmp(&b.0));
            while names.len() > HISTORY_KEEP {
                let (_, path) = names.remove(0);
                let _ = std::fs::remove_file(path);
            }
        }
    }

    /// 列出历史快照（按时间从新到旧）：[(文件名, savedAt)]
    pub fn history(&self) -> Vec<(String, Option<i64>)> {
        let dir = self.history_dir();
        let stem = self
            .path
            .file_stem()
            .map(|s| s.to_string_lossy().to_string())
            .unwrap_or_else(|| "profile".into());
        let mut out = Vec::new();
        if let Ok(entries) = std::fs::read_dir(&dir) {
            for e in entries.flatten() {
                let name = e.file_name().to_string_lossy().to_string();
                if !(name.starts_with(&format!("{stem}-")) && name.ends_with(".json")) {
                    continue;
                }
                let saved_at = std::fs::read_to_string(e.path())
                    .ok()
                    .and_then(|t| serde_json::from_str::<Value>(&t).ok())
                    .and_then(|v| v.get("savedAt").and_then(|x| x.as_i64()));
                out.push((name, saved_at));
            }
        }
        out.sort_by(|a, b| b.0.cmp(&a.0));
        out
    }

    /// 读取某份历史快照内容
    pub fn history_file(&self, name: &str) -> Result<Value, String> {
        if !name.ends_with(".json") || name.contains('/') || name.contains('\\') {
            return Err("非法历史文件名".into());
        }
        let path = self.history_dir().join(name);
        let text = std::fs::read_to_string(&path).map_err(|e| format!("读取历史失败: {e}"))?;
        serde_json::from_str(&text).map_err(|e| format!("解析历史失败: {e}"))
    }

    pub fn save(&self, profile: &Value) -> Result<(), String> {
        if let Some(parent) = self.path.parent() {
            std::fs::create_dir_all(parent).map_err(|e| format!("创建目录失败: {e}"))?;
        }
        self.snapshot_history();
        let body = serde_json::to_string_pretty(profile).map_err(|e| e.to_string())?;
        let tmp = self.path.with_extension("json.tmp");
        std::fs::write(&tmp, format!("{body}\n")).map_err(|e| format!("写临时文件失败: {e}"))?;
        std::fs::rename(&tmp, &self.path).map_err(|e| format!("替换 profile 失败: {e}"))?;
        Ok(())
    }

    pub fn delete(&self) -> bool {
        std::fs::remove_file(&self.path).is_ok()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn names() -> BTreeMap<String, String> {
        [("1", "shoulder_pan"), ("2", "shoulder_lift"), ("6", "gripper")]
            .into_iter()
            .map(|(k, v)| (k.to_string(), v.to_string()))
            .collect()
    }

    fn sample() -> Value {
        json!({
            "zero": {"shoulder_pan": 2003, "shoulder_lift": 2028, "gripper": 2049},
            "sign": {"shoulder_pan": -1, "shoulder_lift": -1, "gripper": 1},
            "grip": {"open": 3293, "closed": 2047, "angle": 90.0},
            "savedAt": 1795311169
        })
    }

    #[test]
    fn validate_ok() {
        let clean = validate(&sample(), &names()).unwrap();
        assert_eq!(clean["grip"]["open"], 3293);
        assert_eq!(clean["sign"]["shoulder_pan"], -1);
        assert_eq!(clean["savedAt"], 1795311169);
    }

    #[test]
    fn validate_rejects_bad() {
        let mut bad = sample();
        bad["zero"]["shoulder_pan"] = json!(9999);
        assert!(validate(&bad, &names()).is_err());
        let mut bad2 = sample();
        bad2["grip"]["closed"] = json!(3290);
        assert!(validate(&bad2, &names()).is_err());
    }

    #[test]
    fn degrees_from_profile() {
        let raw: BTreeMap<u8, i64> =
            [(1u8, 1511i64), (2, 612), (6, 2887)].into_iter().collect();
        let deg = joint_degrees(&raw, &names(), &sample());
        // pan: (1511-2003) = -492 → -43.24° × (-1) = +43.24
        assert!((deg["shoulder_pan"] - 43.24).abs() < 0.01);
        // grip: (2887-2047)/1246 = 0.674 → 60.66°
        assert!((deg["gripper"] - 60.66).abs() < 0.1);
    }

    #[test]
    fn store_keeps_history_on_overwrite() {
        let dir = std::env::temp_dir().join(format!("armd-profile-hist-{}", std::process::id()));
        let store = ProfileStore::new(dir.join("p.json"));
        let mut a = validate(&sample(), &names()).unwrap();
        a["savedAt"] = json!(100);
        store.save(&a).unwrap();
        let mut b = validate(&sample(), &names()).unwrap();
        b["savedAt"] = json!(200);
        store.save(&b).unwrap();
        let hist = store.history();
        assert_eq!(hist.len(), 1, "{hist:?}");
        let old = store.history_file(&hist[0].0).unwrap();
        assert_eq!(old["savedAt"], 100);
        assert_eq!(store.load().unwrap()["savedAt"], 200);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn store_roundtrip() {
        let dir = std::env::temp_dir().join(format!("armd-profile-test-{}", std::process::id()));
        let store = ProfileStore::new(dir.join("p.json"));
        assert!(store.load().is_none());
        store.save(&validate(&sample(), &names()).unwrap()).unwrap();
        assert_eq!(store.load().unwrap()["grip"]["open"], 3293);
        assert!(store.delete());
        assert!(store.load().is_none());
        let _ = std::fs::remove_dir_all(&dir);
    }
}
