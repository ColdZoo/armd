# armd 安全层设计（v0.2.0，2026-11-22）

> 事故背景：从臂肘部（motor 3，XL430/XL330）顶住机械限位持续输出力矩 → 舵机 latch
> `hardware error 0x80` → armd follower worker 读失败/重连循环、串口 fd 未释放
> （lsof fd 223）→ `Device or resource busy` 永久无法恢复，只能完整重启 armd + 物理断电。
> 现场记录：`viking://resources/zeropose/` 的「事故记录」「已知坑 2」。
>
> 本文是 v0.2.0 的**冻结接口规范**：Rust 模块签名、状态/事件 JSON、RPC、配置字段、
> 行为语义都在这里定死。实现、客户端、文档、测试全部以此为准。

## 1. 目标与验收

| # | 要求 | 实现要点 | 验收方式 |
|---|------|---------|---------|
| 1 | 堵转/限位保护（最后一道防线） | 每电机 StallTracker：`|goal-present|>tolerance && |velocity|<=epsilon && torque_on` 持续 `stall_timeout_ms`（默认 400ms）→ **该电机** torque OFF + goal 回退到 present（限位内侧余量）+ state/事件标记 | sim 注入堵转：500ms 内上报 `stalled`、读 Torque_Enable=0、无 0x80、同臂其他电机正常、状态流不断 |
| 2 | 单电机故障隔离 | sync_read 按电机返回 `{data, errors}`；硬件 error / 连续无响应只标记该电机，`connected` 保持 true；恢复自动清除；订阅帧带 `motor_errors` | sim 注入 hardware_error：仅该电机 error，arm 仍 connected，`arm.reboot` 后清除 |
| 3 | 串口重连无 fd 泄漏 | `open` 前**显式 drop 旧 handle**（`Link::open_with` 先 close 再 open）；读失败 3 次先 close 再等 reconnect；失败路径保证 io=None | 单元测试 100 次 open/close + 失败注入，live handle 恒 ≤1、open 前 live==0；真机 `arm.reconnect` ×20 后 lsof 每设备 1 个 fd |
| 4 | 新增 RPC | `arm.reboot`（Dynamixel Reboot 0x08）、`arm.clear_error`、`arm.torque_off`、`arm.sim.inject`（仅 sim） | 见 §6；client/docs 同步 |
| 5 | 安全默认 | 写 Goal_Position(116) 按电机 Operating_Mode 做软限位（mode3 参考 EEPROM Min/Max Position Limit + 绝对 0..4095；mode4 扩展范围 ±1,048,575 且不套 EEPROM 限位；clamp 默认返回 `clamped`/`warnings`，reject 模式回 `soft_limit`）；stall 配置默认开启 | sim：goal=5000 → clamp 到 4095-margin 并返回警告；reject 模式报错；单测覆盖 mode4 多圈目标不被夹回单圈 |

**回归**：两臂 60Hz 不退化（poll 窗口从 132/4 改为 128/8 只多 4 字节/电机），
`read_us` 不显著上升；现有全部测试通过（`cargo test`）。

## 2. 根因核实（v0.1.0 代码）

| 根因 | 结论 | 证据（v0.1.0） |
|---|---|---|
| A 只转发 Goal，无 stall/limit 检测 | **成立** | `src/arm.rs::dispatch` 的 `WriteRegister`/`SyncWrite` 直接下发（旧 509-523 行）；`poll()` 只读位置缓存，无 goal 跟踪、无 velocity。`src/config.rs` 无任何 stall 配置 |
| B 单电机硬件错误拖垮整臂 worker | **成立** | `src/port.rs::read_status` 收到 `st.error != 0` 立即 `Err(IoError::Device)`；`sync_read` 把它当整次读失败返回（旧 142-144 行）；`src/arm.rs::poll` 连续 3 次错误即 `connected=false` 并清空状态流（旧 459-474 行） |
| C reopen 失败路径不释放旧 handle | **成立** | `Worker::open_io` 先 `DxlPort::open` 成功后才覆盖 `self.io`（旧 277-285 行）；`ensure_connected` 在 `io=Some` 时直接尝试再次 open（旧 287-329 行）；`poll` 失败后**保留** `self.io`（旧 459-474）。串口独占（TIOCEXCL）使第二次 open 永远 `Device or resource busy`，旧 fd 永不释放 → 看到 fd 223 / 本机 23:25 现场 fd 4u 仍挂在 `/dev/cu.usbmodem5A7C1190831`（设备节点已消失） |
| D 无 Reboot/clear error/torque_off | **成立** | `src/dxl.rs` 只有 PING/READ/WRITE/SYNC_READ/SYNC_WRITE，无 `INST_REBOOT=0x08`；`src/server.rs` 方法表无 reboot/clear_error |
| E（额外发现）断线后 worker 100% CPU 自旋 | **成立** | `src/arm.rs::run` 旧 588-596：`!connected` 时 `poll()` 被跳过，`next_poll` 永不过期 → `wait=0` → `spin_loop()` 死循环。air.my 旧进程实测 CPU 100.2%（2:12 CPU / 2:13 墙钟），同时重复 `open: Device or resource busy` |

> 现场佐证（2026-11-22 23:25–23:26，air.my）：`arm.list` follower `connected=false`，
> `error="serial error: open /dev/cu.usbmodem5A7C1190831 @ 1000000: Device or resource busy"`；
> 同进程 `lsof` 仍有 `4u CHR ... /dev/cu.usbmodem5A7C1190831`；leader 正常 60Hz。
> 之后设备节点消失，旧 fd 仍在。即"自己占着 fd，又永远 open 不了"。

## 3. 模块与接口（冻结）

### 3.1 `src/dxl.rs`

新增常量/函数：

```rust
pub const INST_REBOOT: u8 = 0x08;
pub const ADDR_MAX_POSITION_LIMIT: u16 = 48;   // 2B, EEPROM
pub const ADDR_MIN_POSITION_LIMIT: u16 = 50;   // 2B, EEPROM
pub const ADDR_HARDWARE_ERROR_STATUS: u16 = 70; // 1B, RAM
pub const ADDR_PRESENT_VELOCITY: u16 = 128;    // 4B
// ADDR_PRESENT_POSITION 132 / TEMPERATURE 146 / INPUT_VOLTAGE 144 保持
pub fn reboot_packet(id: u8) -> Vec<u8>;       // packet(INST_REBOOT, id, &[])
/// 轮询窗口 addr=128 len=8：前 4 字节 Present_Velocity，后 4 字节 Present_Position
pub fn velocity_position(data: &[u8]) -> Option<(i64, i64)>;
/// EEPROM 48/50 的 4 字节窗口：Max(2B) + Min(2B)
pub fn position_limits(data: &[u8]) -> Option<(i64, i64)>; // (min, max)
```

### 3.2 `src/port.rs`

```rust
#[derive(Debug, Clone, Default)]
pub struct SyncReadResult {
    pub data: BTreeMap<u8, Vec<u8>>,   // motor id -> 寄存器原始字节
    pub errors: BTreeMap<u8, u8>,      // motor id -> 状态包 error 字节（硬件告警）
}

pub enum IoError {
    Port(String), Timeout { expected: Vec<u8>, got: Vec<u8> }, Crc,
    Device { id: u8, error: u8 },
    SoftLimit { motor: u8, value: i64, min: Option<i64>, max: Option<i64> },
    Unsupported, Busy,
}

pub trait ArmIo: Send {
    fn sync_read(&mut self, addr: u16, len: u16, ids: &[u8], timeout: Duration) -> IoResult<SyncReadResult>;
    fn write_register(&mut self, id: u8, addr: u16, data: &[u8]) -> IoResult<()>;
    fn sync_write(&mut self, addr: u16, values: &[(u8, Vec<u8>)]) -> IoResult<()>;
    fn ping(&mut self, id: u8) -> IoResult<u16>;
    fn reboot(&mut self, id: u8) -> IoResult<()>;                  // id=0xFE 允许广播
    fn inject_fault(&mut self, kind: &str, ids: &[u8], enable: bool) -> IoResult<()> {
        let _ = (kind, ids, enable); Err(IoError::Unsupported)      // 默认不支持（真串口）
    }
    fn describe(&self) -> String;
}
```

语义：
- `sync_read` 只有**传输层**失败才返回 `Err`（写串口失败/总线无任何回包 → Timeout）；
  某个舵机状态包带 error 位时**不再整次失败**：记入 `errors[id]` 后继续收其他包。
  完全没响应的电机不出现在 `data`/`errors` 中。
- `read_status` 内部不再把 `st.error != 0` 转成 Err；`write_register`/`ping` 在拿到状态包后
  自行把 error 转 `IoError::Device`。
- `Link`：串口句柄生命周期唯一入口，**open 前必须先 close（drop 旧 handle）**：

```rust
pub struct Link { io: Option<Box<dyn ArmIo>> }
impl Link {
    pub fn new() -> Self;
    pub fn is_open(&self) -> bool;
    pub fn io(&mut self) -> Option<&mut (dyn ArmIo + 'static)>;
    pub fn close(&mut self) -> bool;  // 返回是否真的关了一个
    /// 先 close（若 old handle 存在，Drop 即 close(fd)），再调用 open()。
    /// open() 失败时保证 self.io == None。
    pub fn open_with<F: FnOnce() -> IoResult<Box<dyn ArmIo>>>(&mut self, open: F) -> IoResult<()>;
}
```
- `DxlPort`：`reboot` 发 Reboot 指令；单电机等状态包（50ms，容忍 error 位），广播不等。
- `SimPort`：仿真完整寄存器与动力学：
  - 位置按速度向 goal 移动（默认 1500 counts/s，60Hz 轮询下每帧 ~25 counts），velocity 据此估算；
  - 支持 sync_read：`128/8`（velocity+position）、`132/4`、`64/1`、`48/4`、`70/1`、`116/4`、`144/3`；
  - 支持 sync_write/write_register：`64`（torque）、`116`（goal）、`48/50`（limits）；
  - `inject_fault`: `"stall"`（该电机当前姿态被顶住：位置冻结、velocity=0，goal 仍被记录）、
    `"hardware_error"`（sync_read 对该电机返回 error=0x80，直到 `reboot` 或注入 clear）、
    `"no_response"`（该电机不出现在 data/errors，模拟掉线）、
    `"transport_error"`（整条总线读写返回传输错误，模拟 USB 掉线，用于验证"先释放 fd 再重连"）、
    `"clear"`（清除全部注入）；
  - `reboot(id)`：清除该电机 hardware_error/stall 注入，torque=0；广播则全部。

### 3.3 `src/safety.rs`（新文件，纯逻辑，可单测）

```rust
pub fn hardware_error_names(code: u8) -> Vec<&'static str>;
// 0x80 -> "hardware_alert"；0x01 -> "input_voltage"；0x04 -> "overheating"；
// 0x08 -> "motor_encoder"；0x10 -> "electrical_shock"；0x20 -> "overload"；
// 其他位 -> "reserved_bit_N"
pub fn hardware_error_message(code: u8) -> String;
// 含 "latched; 用 arm.reboot 清除" 的说明（0x80 时）

pub struct SoftLimitConfig { pub enabled: bool, pub margin: i64 }
pub struct SoftLimitOutcome {
    pub applied: i64, pub requested: i64, pub clamped: bool,
    pub min: Option<i64>, pub max: Option<i64>, pub warnings: Vec<String>,
}
/// 按 Operating_Mode 区分：
/// - mode=3（Position Control，从臂）或未知：绝对范围 0..=4095；enabled 且有合法
///   EEPROM limits 时再 clamp 到 [min+margin, max-margin]（区间为空取中点）；
/// - mode=4（Extended Position Control，主臂）：绝对范围 ±1,048,575，
///   **不应用** EEPROM 位置限位（该限位只对单圈位置模式生效，否则多圈目标会被错误夹回）；
/// - limits 未知时只给状态/启动日志告警，不阻止写入（不做静默的额外改写）。
pub fn apply_soft_limit(motor: u8, value: i64, limits: Option<(i64, i64)>,
                        mode: Option<u16>, cfg: &SoftLimitConfig) -> SoftLimitOutcome;

pub struct StallConfig { pub enabled: bool, pub timeout: Duration, pub tolerance: i64, pub velocity_epsilon: i64 }
pub struct StallTrigger { pub motor: u8, pub position: i64, pub goal: i64, pub velocity: i64, pub stall_ms: u64 }
pub struct StallTracker { /* since: HashMap<u8, Instant> */ }
impl StallTracker {
    pub fn new(cfg: StallConfig) -> Self;
    /// 条件成立累计超过 timeout 时返回 Some（每电机只触发一次，需 reset）。
    /// torque_on != Some(true) 或 velocity == None 或 goal == None 时条件不成立并清计时。
    pub fn observe(&mut self, motor: u8, present: i64, velocity: Option<i64>,
                   goal: Option<i64>, torque_on: Option<bool>, now: Instant) -> Option<StallTrigger>;
    pub fn reset(&mut self, motor: u8);
    pub fn reset_all(&mut self);
}
```

### 3.4 `src/arm.rs`

`ArmState` 新增字段（JSON 见 §5）：

```rust
pub motor_errors: BTreeMap<u8, MotorError>,
pub stalls: BTreeMap<u8, StallEvent>,
pub torque: BTreeMap<u8, bool>,
pub limits: BTreeMap<u8, (i64, i64)>,
pub goals: BTreeMap<u8, i64>,
pub velocity: BTreeMap<u8, i64>,
pub safety: SafetyInfo,   // 生效配置摘要 {stall_protection, stall_timeout_ms, stall_tolerance, auto_torque_off, soft_limits}
```

```rust
pub struct MotorError { pub code: Option<u8>, pub names: Vec<String>, pub message: String,
                        pub ts_ms: u64, pub since_ms: u64 }
pub struct StallEvent { pub motor: u8, pub reason: String, pub position: i64, pub goal: i64,
                        pub stall_ms: u64, pub ts_ms: u64, pub torque_off: bool,
                        pub action: String, pub reverted_goal: i64 }
```

`Cmd` 新增（server 依赖此枚举）：

```rust
pub enum Cmd {
    // ... 原 SyncRead/ReadRegister/WriteRegister/SyncWrite/Ping/Reconnect/SetRate
    Reboot { ids: Vec<u8> },
    ClearError { ids: Vec<u8> },
    SimInject { kind: String, ids: Vec<u8>, enable: bool },
}
```

**Worker 行为（实现细节）**：
1. 连接：`Link::open_with` 打开（sim 用 SimPort）。握手 ping 各电机：≥1 个响应即
   `connected=true`（缺的电机记 `no_response` 标记，不拖垮整臂）；0 个响应则 close 并等
   下个 `reconnect_sec` 周期重试（**不留旧 handle**）。
2. 连接后刷新每电机 `Operating_Mode(11,1)`、`Torque_Enable(64,1)` 与 EEPROM
   `Min/Max Position Limit(48,4)`；之后这些状态由 armd 对 11/64/48/50 写入的跟踪维护
   （teleop 启动会把从臂切 POSITION(3)、可能修复 EEPROM 限位，必须跟踪）。只有仍未知的电机
   才每秒重试一次，避免每秒额外 3 次 sync_read 占用总线。
3. 轮询窗口：`poll_addr` 未显式配置且 `stall_protection=true` 时用 `(128, 8)` 同时取
   velocity+position；若用户显式配置 `poll_addr/poll_len` 则维持旧语义（只有 position，
   velocity 未知 → stall 检测自动跳过，启动时/首次轮询打一条警告日志）。
4. 每次轮询：
   - data 里有的电机：更新 joints/velocity，清除对应 `no_response` 标记（hardware error
     标记只在**读到数据**后清除），`last_read_ok_ms=now`；
   - errors 里有的电机：`motor_errors[id]` 更新（code/names/message/ts），首次出现发
     `event:"motor_error"`；此前有 error 的电机读到数据 → 清除并发 `event:"motor_recovered"`；
   - 完全没响应的电机：`read_fails+=1`，连续 ≥10 次（≈167ms@60Hz）标记
     `code=None, names=["no_response"]`（只发一次事件）；恢复即清除；
   - `data` 与 `errors` 都为空且连续 ≥3 次 → 传输层掉线：`Link::close()`（立即释放 fd）、
     `connected=false`、发 `event:"arm"`、等 `reconnect_sec` 后重开；
   - arm 级 `connected` 不因单电机错误/无响应而变 false。
5. 写 Goal（addr 116 len 4，`write_register` 或 `sync_write`）：先按 §3.3 软限位
   （按该电机实时 Operating_Mode 决定绝对范围；扩展模式下不套 EEPROM 限位）；
   clamp 模式把夹紧后的值下发，响应附 `clamped`/`warnings`；reject 模式整体拒绝、
   返回 `IoError::SoftLimit`（RPC `soft_limit`）。成功下发后记录 `goals[id]`，清除该电机
   `stalls[id]`（新的显式目标 = 重新使能意图）。
6. 堵转检测：每次轮询对 `torque==Some(true) && goals 有值 && velocity 有值` 的电机跑
   `StallTracker::observe`。触发时（在 worker 线程内、同一次 poll 中）：
   1) 先 `sync_write(Torque_Enable, [(id,[0])])`（若 `auto_torque_off=true`；默认 true），
      更新 `torque[id]=false`，`stats.torque_offs+=1`；
   2) `sync_write(Goal_Position, [(id, revert)])`，`revert = clamp(present,
      [min+stall_recover_margin, max-stall_recover_margin])`（未知限位时取 present），
      记录 `goals[id]=revert`；
   3) 记 `stalls[id]`、`stats.stalls+=1`，发 `event:"stall"`；下一次 `publish_state` 自然带出；
   4) 其他电机完全不受影响；不触发 hardware error。
7. `arm.torque`（写 64）：更新 torque 状态；`enable=true` 时清除该电机 stall 标记。
8. `Cmd::Reboot`：对 ids 逐个 `io.reboot(id)`（ids==全部电机时可广播 0xFE 一次）；
   等待 ~600ms（期间 poll 不把缺包记成 no_response：设置 `suppress_until`），然后逐个 ping
   （最多 ~2s），成功的电机：清 `motor_errors`/`stalls`、`torque[id]=false`、重新读
   limits/torque/position；失败的标记 `no_response`。返回 rebooted/failed。**不关闭串口**。
9. `Cmd::ClearError`：清 `motor_errors`/`stalls` 标记；对每个 id 读
   `Hardware_Error_Status(70,1)` 附在结果里；若仍非 0 → warnings 提示用 `arm.reboot`。
10. `Cmd::SimInject`：仅 sim 臂；转发 `io.inject_fault`；非 sim 返回 `IoError::Unsupported`。
11. `Cmd::Reconnect`：`Link::close()` 后立即尝试 open（失败则等 reconnect_sec 重试）。
12. `run()` 断线时不得自旋：`!connected` 时每轮至少 sleep 2ms，并推进 `next_poll`。
13. worker 退出（stop）前 `Link::close()`。

`Stats` 新增：`stalls: AtomicU64`、`torque_offs: AtomicU64`、`opens: AtomicU64`、
`closes: AtomicU64`（JSON 同名）。`Link` 的 open/close 次数由 worker 计数。

### 3.5 `src/config.rs`

`ArmConfig` 新增字段（全部 `#[serde(default)]`，旧配置无需改动）：

| 字段 | 默认 | 含义 |
|---|---|---|
| `stall_protection` | `true` | 堵转保护总开关 |
| `stall_timeout_ms` | `400` | 条件持续多久触发（建议 300–500） |
| `stall_tolerance` | `60` | Goal-Present 偏差阈值（counts） |
| `stall_velocity_epsilon` | `2` | 视为静止的 Present_Velocity 阈值（0.229rpm/单位） |
| `auto_torque_off` | `true` | 触发时自动 torque OFF 该电机 |
| `stall_recover_margin` | `20` | 回退目标离限位内侧的余量（counts） |
| `stall_exclude_motors` | `[]` | 不参与堵转自动保护的电机（建议把夹爪 6 排除：夹住物体时 Goal-Present 长期有差属正常） |
| `soft_limits` | `true` | 写 Goal 时用 EEPROM 限位做软限位 |
| `soft_limit_mode` | `"clamp"` | `"clamp"`（夹紧+警告）或 `"reject"`（报错） |
| `soft_limit_margin` | `10` | 软限位内侧余量（counts） |

`poll_window()` 辅助函数：`poll_addr.is_none() && stall_protection` → `(128, 8)`，
否则 `(poll_addr.unwrap_or(132), poll_len)`。

### 3.6 `src/server.rs`（RPC 层）

见 §6。新增方法：`arm.reboot`、`arm.clear_error`、`arm.torque_off`、`arm.sim.inject`；
`arm.write_register`/`arm.sync_write` 的 Goal 写入返回 `clamped`/`warnings`；
`arm.list` 每项附加 `motor_errors`/`stalls` 摘要；`arm.state` 自动包含 §5 新字段；
`IoError::Unsupported` → `not_sim`（sim.inject 非 sim 臂时）；
`IoError::SoftLimit` → `soft_limit`；`IoError::Device` → `serial_error`。

`call_arm` 等待时间：`arm.reboot` 5000ms；`arm.clear_error` 2000ms；其余不变。

## 4. 事件（订阅推送）

```jsonc
{"event":"stall","sub":3,"arm":"koch_v1_1:follower","motor":3,"reason":"stall",
 "position":3300,"goal":3386,"stall_ms":401,"torque_off":true,
 "action":"torque_off+goal_revert","reverted_goal":3300,"ts":1795361000000}
{"event":"motor_error","sub":3,"arm":"...","motor":3,"code":128,
 "names":["hardware_alert"],"message":"motor 3 hardware error 0x80 (hardware_alert)：latched；用 arm.reboot 清除",
 "ts":1795361000000}
{"event":"motor_recovered","sub":3,"arm":"...","motor":3,"ts":1795361000000}
```

`event:"state"` 帧在原有字段上增加 §5 的 `velocity/motor_errors/stalls/torque/limits/goals/safety`。
`event:"arm"`（掉线/恢复）保持原样。

## 5. 状态 JSON 增量（`arm.state` 与订阅帧）

```jsonc
{
  "velocity": {"1": 0, "3": 0},                      // raw，窗口不含 velocity 时省略/为空
  "goals": {"3": 3300},                              // armd 最后接受的目标（软限位后）
  "torque": {"1": true, "2": true, "3": false},      // 已知才出现
  "limits": {"3": {"min": 1607, "max": 3309}},       // EEPROM，读到才出现
  "motor_errors": {"3": {"code":128,"names":["hardware_alert"],
                         "message":"...","ts":..., "since_ms":...}},
  "stalls": {"3": {"motor":3,"reason":"stall","position":3300,"goal":3386,
                   "stall_ms":401,"ts":...,"torque_off":true,
                   "action":"torque_off+goal_revert","reverted_goal":3300}},
  "safety": {"stall_protection":true,"stall_timeout_ms":400,"stall_tolerance":60,
             "auto_torque_off":true,"soft_limits":true,"stall_exclude_motors":[6]},
  "connected": true,                                  // 单电机错误不影响
  "error": null                                       // 仅整臂/传输层错误
}
```

## 6. RPC 增量（docs/api.json 以此为准）

| 方法 | params | 返回 | 说明 |
|---|---|---|---|
| `arm.reboot` | `arm`, `motors?` | `{ok,rebooted:[u8],failed:[u8],errors:{id:str},torque:{id:bool},limits:{id:{min,max}},warnings:[str]}` | 发 Dynamixel Reboot(0x08) 清除 latched 硬件错误；默认全部 `motors`。等待最长 5s。 |
| `arm.clear_error` | `arm`, `motors?` | `{ok,cleared:[u8],hardware_error_status:{id:u8},warnings:[str]}` | 只清 armd 侧标记 + 读 Hardware_Error_Status(70)；latched 错误仍需 reboot。 |
| `arm.torque_off` | `arm`, `motors?` | `{ok,torque:{id:bool}}` | 按电机 torque OFF 的便捷方法（等价 `arm.torque enable=false`）。 |
| `arm.sim.inject` | `arm`, `kind`（`stall`/`hardware_error`/`no_response`/`transport_error`/`clear`）, `motors?`, `enable?`（默认 true） | `{ok,kind,motors,enable,sim:true}` | 仅 `sim:true` 的臂；否则 `not_sim`。 |
| `arm.write_register` | 原参数（Goal=116 时） | 原 `{ok}` + `{ok,applied,clamped?,warnings?}` | 软限位；reject 模式 `soft_limit`。 |
| `arm.sync_write` | 原参数（Goal=116 时） | 原 `{ok}` + 同上（多电机 `clamped` 对象） | 同上。 |
| `arm.read` / `arm.state` | 原参数 | `positions` 之外再带 `errors:{id:u8}` | `arm.read` 的硬件错误按电机返回，不整次失败。 |
| `arm.stats` | 原 | 新增 `stalls,torque_offs,opens,closes` | 计数器。 |
| `arm.list` | 原 | 每项新增 `motor_errors`, `stalls` | 摘要。 |

错误码新增：`soft_limit`（软限位 reject）、`not_sim`（仿真注入用在真机臂）。

## 7. 文档/客户端/测试清单

- `clients/python/armd_client.py`：新增 `reboot/clear_error/torque_off/sim_inject`、
  `safety(arm)` 便捷方法、CLI 子命令；`ArmdClient(auto_reconnect=True)` 连接断开自动重连 +
  自动重订阅（保存 subscribe 参数），`on_disconnect/on_reconnect` 回调可选；
  `BrokenPipeError`/`ConnectionResetError` 不直接抛给调用方（重连成功后重试一次）。
- `docs/api.json` + `scripts/gen-armd-doc.py` → 重新生成 `docs/armd.html`、`docs/armd-api.md`；
  `docs/DESIGN.md` 加「安全层 / 单电机隔离 / fd 生命周期 / reboot / 软限位」章节；
  `README.md` 更新特性、配置表、RPC、排障；`configs/koch*.json` 显式写出新配置项。
- 测试：
  - 单元（src 内）：§3.3 全部（stall 计时/复位、错误位命名、软限位 clamp/reject/未知限位）、
    dxl reboot 帧与 velocity/limits 解析、`Link` 的 100 次 open/close + open 失败注入
    （Drop 计数证明"open 前旧 handle 已释放"）。
  - 集成（`tests/`，起 sim 进程走真实 RPC/订阅）：
    1. 堵转：`arm.sim.inject stall` → 500ms 内 `arm.state.stalls` 出现、`read_register(64)==0`、
       其他电机 joints 继续更新、`connected=true`、无 `hardware_alert`；
    2. 单电机 error：inject hardware_error → `motor_errors["3"].code==128`、connected=true、
       其他电机正常；`arm.reboot` → 清除、torque=false、connected=true；
    3. 软限位：写 goal 5000 → clamp 到 4095-margin + `clamped` 警告；reject 模式 → `soft_limit`；
    4. 重连/并发：`arm.reconnect` ×20 + 订阅不断流；稳态下**活跃串口句柄恰为 1**：
       `stats.opens - stats.closes == 1`（worker 关闭后二者相等）；传输失败时先 close：
       用 `transport_error` 注入让 worker 断开，等待 `connected=false` 后
       `stats.opens == stats.closes`（没有把旧 handle 留在手里），清除注入后恢复成差值 1。
  - 脚本：`scripts/fd_reconnect_test.py`（对运行中的 armd 做 N 次 `arm.reconnect`，
    lsof 断言每设备串口 fd 恒为 1）；`scripts/sim_acceptance.py`（可选，复用集成场景）。

## 8. 实现与验收结果（v0.2.0，2026-11-22/23，air.my）

> 详细复盘见 [`docs/INCIDENT-2026-11-22.md`](INCIDENT-2026-11-22.md) §8。

| 项 | 结果 |
|---|---|
| 代码 | 新增 `src/safety.rs`（错误位/软限位/StallTracker + 单测）；`port.rs` 的 `SyncReadResult`/`Link`/`DxlPort::reboot`/`SimPort` 故障注入；`arm.rs` 安全层集成；`server.rs` 新 RPC；`config.rs` 新字段；版本 0.2.0 |
| 自动测试 | `cargo test`：**46 unit + 6 integration 全过**；`scripts/sim_acceptance.py` **7/7 PASS**（stall 267ms、error 隔离+reboot、clamp/reject、20×reconnect、transport_error fd 释放） |
| 真机部署 | `~/.local/bin/armd` 0.2.0 + `launchctl kickstart -k com.armd`；两臂 6/6 响应、per-motor errors=0；配置加 `stall_exclude_motors:[6]` |
| 真机验收 | 40 次 `arm.reconnect`/2 臂：每设备恒 1 个 fd、无 busy、订阅不断；真机 `arm.reboot` 单机+广播全 6 电机成功，从臂 motor 3 连续 reboot 20 次全部成功且 fd 恒 1；`arm.clear_error` 硬件错误状态全 0；`arm.sim.inject` 对真机返回 `not_sim` |
| 性能回归 | 与 v0.1.0 同环境 A/B：poll ≈59.0 vs 59.6 Hz（宿主机内存压力导致 OS 定时器退化，两版同受影响）；`read_us` +200~350µs（每电机多读 4B velocity，占周期 12%）；写 RPC p50 11.9/15.5ms vs 13.9/17.5ms（未退化） |
| 未执行 | 物理拔插 USB 与物理堵转（设备侧已恢复、无人值守不做机械实验）；用 20 次真机 servo Reboot + 40 次 reconnect + sim `transport_error` + `Link` 单测覆盖同一失效路径 |
