# armd API 参考（v0.2.0，更新 2026-11-22）

Dynamixel 机械臂串口守护进程：独占串口总线，对上层提供 NDJSON RPC 与发布订阅；持有标定档案（profile）；v0.2.0 起内置安全层（堵转保护 / 软限位 / 单电机故障隔离）。

> 完整 HTML 文档（含架构/安装/排障/示例）：`viking://resources/armd/armd.html`；
> 本机文档站 http://127.0.0.1:8790/ ｜局域网 http://air.my:8790/ ｜机器可读规范 `viking://resources/armd/api.json`

## 传输

- Unix socket：/tmp/armd.sock（macOS/Linux 本机；权限 0600）
- TCP：tcp://host:8771（下位机/网络场景，配置 listen 里开启）
- 帧格式：NDJSON：一行一个 JSON。请求/响应用 id 配对；推送帧带 event 字段、无 id。
- 鉴权：配置 token 后，客户端必须先发 hello 带 token，否则所有方法返回 unauthorized。
- JSON-RPC：兼容 JSON-RPC 2.0：请求带 "jsonrpc":"2.0" 时响应也带。

## 请求 / 响应示例

```json
{"id": 7, "method": "arm.state", "params": {"arm": "koch_v1_1:follower"}}
```

```json
{"id": 7, "result": {"seq": 123, "connected": true, "joints": {"1": 1511}, "deg": {"shoulder_pan": 43.24}, "torque": {"1": true}, "goals": {"1": 1511}, "stalls": {}, "motor_errors": {}, "safety": {"stall_protection": true, "stall_timeout_ms": 400, "stall_tolerance": 60, "auto_torque_off": true, "soft_limits": true, "stall_exclude_motors": [6]}}}
```

错误响应：

```json
{"id": 7, "error": {"code": "no_such_arm", "message": "unknown arm 'x'"}}
```

## 方法总览

| 方法 | 说明 |
|---|---|
| `ping` | 健康检查。 |
| `hello` | 鉴权握手（配置了 token 时必需）。 |
| `info` | 守护进程信息。 |
| `arm.list` | 所有机械臂摘要：id/label/port/sim/motors/connected/hz/seq/error/profile_saved_at/motor_errors/stalls/stats。 |
| `arm.state` | 读状态缓存，不占总线。deg 只在有 profile 时出现。v0.2.0 新增安全层字段（velocity/goals/torque/limits/motor_errors/stalls/safety）。 |
| `arm.read` | 立刻 sync_read 一次（占用总线），用于即时取值/标定采点。v0.2.0 起按电机返回硬件 error（errors），单电机故障不再整次失败。 |
| `arm.read_register` | 读任意控制表寄存器（小端有符号解析）。 |
| `arm.write_register` | 写任意寄存器（等舵机状态包确认）。写 Goal_Position(116) 时先过软限位：clamp 模式夹紧并返回 clamped/warnings；reject 模式返回 soft_limit 错误。 |
| `arm.sync_write` | 同步写多个舵机（广播式，不等回包）。Goal_Position(116) 同样过软限位，clamped 为按电机的对象；reject 模式任一电机越界即整体拒绝（soft_limit）。 |
| `arm.torque` | 扭矩开关（写 Torque_Enable=64）。enable=true 时清除该电机 stall 标记；写成功后 state.torque 与 goal 跟踪同步更新。 |
| `arm.torque_off` | 按电机 torque OFF 的便捷方法（等价 arm.torque enable=false，默认全部电机）。应急卸力矩用。 |
| `arm.reboot` | 发 Dynamixel Reboot(0x08) 清除 latched 硬件错误（如 0x80 hardware_alert）；默认全部 motors。reboot 后 RAM 复位：torque=false、goal 清空、重新读 limits/position。等待最长 5s。 |
| `arm.clear_error` | 只清 armd 侧 motor_errors/stalls 标记 + 读 Hardware_Error_Status(70)。若硬件仍报错（latched），warnings 会提示用 arm.reboot；不替代 reboot。 |
| `arm.sim.inject` | 仅仿真臂的故障注入：stall（顶住该电机）/ hardware_error（回 error=0x80 直到 reboot）/ no_response（完全不应答）/ transport_error（整条总线读写返回传输错误，模拟 USB 掉线，用于验证「先释放 fd 再重连」）/ clear（清除全部注入）。真机臂返回 not_sim。 |
| `arm.ping` | 单舵机 ping，返回 model number。硬件 error 位会转 serial_error 并在 motor_errors 里标记该电机。 |
| `arm.set_rate` | 改轮询频率。 |
| `arm.reconnect` | 强制关串口再重开（排查用）。v0.2.0 保证先 close 旧 fd 再 open，不再出现 Device or resource busy。 |
| `arm.stats` | 计数器。v0.2.0 新增 stalls/torque_offs/opens/closes。fd 不变量：connected 期间恰好 1 个 live 串口句柄，即 opens - closes == 1；worker 关闭/断线释放 fd 后二者相等。 |
| `arm.profile.get` | 读取标定档案。 |
| `arm.profile.set` | 校验并持久化档案（覆盖前自动留历史快照）。 |
| `arm.profile.history` | 历史快照列表（保留最近 20 份）。 |
| `arm.profile.restore` | 从历史快照恢复。 |
| `arm.profile.delete` | 删除档案。 |
| `arm.calibration.snapshot` | 标定采点用：当前 raw（按关节名）+角度+健康，不占总线。 |
| `sub.subscribe` | 订阅状态流。rate>=54 时直通每个轮询帧；更低频率按时间抽帧。 |
| `sub.unsubscribe` | 退订。 |
| `sub.rate` | 改订阅频率。 |

## 方法详情

### `ping`

健康检查。

返回：

```json
{
  "pong": "bool",
  "version": "str",
  "uptime_sec": "int"
}
```

### `hello`

鉴权握手（配置了 token 时必需）。

| 参数 | 类型 |
|---|---|
| `token` | str（可选） |

返回：

```json
{
  "ok": "bool",
  "auth_required": "bool",
  "version": "str"
}
```

### `info`

守护进程信息。

返回：

```json
{
  "version": "str",
  "uptime_sec": "int",
  "socket": "str",
  "arms": "[str]"
}
```

### `arm.list`

所有机械臂摘要：id/label/port/sim/motors/connected/hz/seq/error/profile_saved_at/motor_errors/stalls/stats。

返回：

```json
{
  "[arm]": "见 arm.state",
  "additional": "每项新增 motor_errors/stalls 摘要（v0.2.0）"
}
```

### `arm.state`

读状态缓存，不占总线。deg 只在有 profile 时出现。v0.2.0 新增安全层字段（velocity/goals/torque/limits/motor_errors/stalls/safety）。

| 参数 | 类型 |
|---|---|
| `arm` | str |

返回：

```json
{
  "seq": "int",
  "ts": "int ms",
  "connected": "bool（单电机错误不影响，保持 true）",
  "joints": "{motor_id: raw}（多圈关节/主臂 mode 4 可超出 0..4095，如 6133 / -2889）",
  "velocity": "{motor_id: raw}（轮询窗口不含 velocity 时为空；多圈关节同样可超出 0..4095）",
  "deg": "{joint: deg}",
  "joint_names": "{motor_id: joint}",
  "health": "{motor_id: {voltage, temp}}",
  "hz": "float",
  "read_us": "int",
  "error": "str|null（仅整臂/传输层错误）",
  "motor_errors": "{motor_id: {code:int|null, names:[str], message:str, ts:int, since_ms:int}}",
  "stalls": "{motor_id: {motor, reason, position, goal, stall_ms, ts, torque_off, action, reverted_goal}}",
  "torque": "{motor_id: bool}（已知才出现）",
  "limits": "{motor_id: {min, max}}（EEPROM，读到才出现）",
  "goals": "{motor_id: raw}（armd 最后接受的目标，软限位后）",
  "safety": "{stall_protection, stall_timeout_ms, stall_tolerance, auto_torque_off, soft_limits, stall_exclude_motors}",
  "profile_saved_at": "int|null",
  "arm": "str",
  "port": "str",
  "sim": "bool",
  "poll_addr": "int",
  "motors": "[u8]"
}
```

### `arm.read`

立刻 sync_read 一次（占用总线），用于即时取值/标定采点。v0.2.0 起按电机返回硬件 error（errors），单电机故障不再整次失败。

| 参数 | 类型 |
|---|---|
| `arm` | str |
| `motors` | [u8]（可选） |
| `timeout_ms` | int（可选），默认 50 |

返回：

```json
{
  "positions": "{motor_id: raw}",
  "errors": "{motor_id: error_byte}"
}
```

### `arm.read_register`

读任意控制表寄存器（小端有符号解析）。

| 参数 | 类型 |
|---|---|
| `arm` | str |
| `motor` | u8 |
| `address` | u16 |
| `length` | u16 |

返回：

```json
{
  "data": "[u8]",
  "value": "int",
  "model": "int|null（预留）"
}
```

### `arm.write_register`

写任意寄存器（等舵机状态包确认）。写 Goal_Position(116) 时先过软限位：clamp 模式夹紧并返回 clamped/warnings；reject 模式返回 soft_limit 错误。

| 参数 | 类型 |
|---|---|
| `arm` | str |
| `motor` | u8 |
| `address` | u16 |
| `length` | u16 |
| `value` | int |

返回：

```json
{
  "ok": "bool",
  "applied": "int|null（Goal 写入时经软限位后的实际下发值）",
  "clamped": "{requested, applied, min, max}|null",
  "warnings": "[str]"
}
```

### `arm.sync_write`

同步写多个舵机（广播式，不等回包）。Goal_Position(116) 同样过软限位，clamped 为按电机的对象；reject 模式任一电机越界即整体拒绝（soft_limit）。

| 参数 | 类型 |
|---|---|
| `arm` | str |
| `address` | u16 |
| `length` | u16 |
| `values` | {motor_id: value} |

返回：

```json
{
  "ok": "bool",
  "clamped": "{motor_id: {requested, applied, min, max}}",
  "warnings": "[str]",
  "torque": "{motor_id: bool}（写 Torque_Enable 时）"
}
```

### `arm.torque`

扭矩开关（写 Torque_Enable=64）。enable=true 时清除该电机 stall 标记；写成功后 state.torque 与 goal 跟踪同步更新。

| 参数 | 类型 |
|---|---|
| `arm` | str |
| `enable` | bool |
| `motors` | [u8]（可选） |

返回：

```json
{
  "ok": "bool",
  "torque": "{motor_id: bool}"
}
```

### `arm.torque_off`

按电机 torque OFF 的便捷方法（等价 arm.torque enable=false，默认全部电机）。应急卸力矩用。

| 参数 | 类型 |
|---|---|
| `arm` | str |
| `motors` | [u8]（可选） |

返回：

```json
{
  "ok": "bool",
  "torque": "{motor_id: bool}"
}
```

### `arm.reboot`

发 Dynamixel Reboot(0x08) 清除 latched 硬件错误（如 0x80 hardware_alert）；默认全部 motors。reboot 后 RAM 复位：torque=false、goal 清空、重新读 limits/position。等待最长 5s。

| 参数 | 类型 |
|---|---|
| `arm` | str |
| `motors` | [u8]（可选） |

返回：

```json
{
  "ok": "bool",
  "rebooted": "[u8]",
  "failed": "[u8]",
  "errors": "{motor_id: str}（Reboot 指令本身失败的电机）",
  "failures": "{motor_id: str}（重启后 ping 仍失败的原因）",
  "torque": "{motor_id: bool}",
  "limits": "{motor_id: {min, max}}",
  "warnings": "[str]"
}
```

### `arm.clear_error`

只清 armd 侧 motor_errors/stalls 标记 + 读 Hardware_Error_Status(70)。若硬件仍报错（latched），warnings 会提示用 arm.reboot；不替代 reboot。

| 参数 | 类型 |
|---|---|
| `arm` | str |
| `motors` | [u8]（可选） |

返回：

```json
{
  "ok": "bool",
  "cleared": "[u8]",
  "hardware_error_status": "{motor_id: u8}",
  "warnings": "[str]"
}
```

### `arm.sim.inject`

仅仿真臂的故障注入：stall（顶住该电机）/ hardware_error（回 error=0x80 直到 reboot）/ no_response（完全不应答）/ transport_error（整条总线读写返回传输错误，模拟 USB 掉线，用于验证「先释放 fd 再重连」）/ clear（清除全部注入）。真机臂返回 not_sim。

| 参数 | 类型 |
|---|---|
| `arm` | str |
| `kind` | str（stall|hardware_error|no_response|transport_error|clear） |
| `motors` | [u8]（可选） |
| `enable` | bool（可选），默认 True |

返回：

```json
{
  "ok": "bool",
  "kind": "str",
  "motors": "[u8]",
  "enable": "bool",
  "sim": true
}
```

### `arm.ping`

单舵机 ping，返回 model number。硬件 error 位会转 serial_error 并在 motor_errors 里标记该电机。

| 参数 | 类型 |
|---|---|
| `arm` | str |
| `motor` | u8 |

返回：

```json
{
  "model": "int"
}
```

### `arm.set_rate`

改轮询频率。

| 参数 | 类型 |
|---|---|
| `arm` | str |
| `hz` | float (0.5~500) |

返回：

```json
{
  "ok": "bool",
  "hz": "float"
}
```

### `arm.reconnect`

强制关串口再重开（排查用）。v0.2.0 保证先 close 旧 fd 再 open，不再出现 Device or resource busy。

| 参数 | 类型 |
|---|---|
| `arm` | str |

返回：

```json
{
  "ok": "bool"
}
```

### `arm.stats`

计数器。v0.2.0 新增 stalls/torque_offs/opens/closes。fd 不变量：connected 期间恰好 1 个 live 串口句柄，即 opens - closes == 1；worker 关闭/断线释放 fd 后二者相等。

| 参数 | 类型 |
|---|---|
| `arm` | str |

返回：

```json
{
  "polls": "int",
  "read_errors": "int",
  "writes": "int",
  "frames_published": "int",
  "frames_dropped": "int",
  "reconnects": "int",
  "urgent_queued": "int",
  "normal_queued": "int",
  "stalls": "int（堵转触发次数）",
  "torque_offs": "int（安全层自动/手动 torque off 次数）",
  "opens": "int（串口 open 次数）",
  "closes": "int（串口 close 次数）"
}
```

### `arm.profile.get`

读取标定档案。

| 参数 | 类型 |
|---|---|
| `arm` | str |

返回：

```json
{
  "profile": "profile|null",
  "path": "str",
  "has_profile": "bool"
}
```

### `arm.profile.set`

校验并持久化档案（覆盖前自动留历史快照）。

| 参数 | 类型 |
|---|---|
| `arm` | str |
| `profile` | profile |

返回：

```json
{
  "profile": "profile",
  "path": "str"
}
```

### `arm.profile.history`

历史快照列表（保留最近 20 份）。

| 参数 | 类型 |
|---|---|
| `arm` | str |

返回：

```json
{
  "history": "[{name, savedAt}]",
  "dir": "str"
}
```

### `arm.profile.restore`

从历史快照恢复。

| 参数 | 类型 |
|---|---|
| `arm` | str |
| `name` | str |

返回：

```json
{
  "restored": "str",
  "profile": "profile"
}
```

### `arm.profile.delete`

删除档案。

| 参数 | 类型 |
|---|---|
| `arm` | str |

返回：

```json
{
  "deleted": "bool"
}
```

### `arm.calibration.snapshot`

标定采点用：当前 raw（按关节名）+角度+健康，不占总线。

| 参数 | 类型 |
|---|---|
| `arm` | str |

返回：

```json
{
  "raw": "{joint: raw}",
  "deg": "{joint: deg}",
  "seq": "int",
  "ts": "int",
  "connected": "bool",
  "health": "{motor_id: {voltage, temp}}"
}
```

### `sub.subscribe`

订阅状态流。rate>=54 时直通每个轮询帧；更低频率按时间抽帧。

| 参数 | 类型 |
|---|---|
| `arms` | [str]（可选） |
| `rate_hz` | float（可选），默认 60 |

返回：

```json
{
  "sub_id": "int",
  "rate_hz": "float",
  "arms": "[str]"
}
```

### `sub.unsubscribe`

退订。

| 参数 | 类型 |
|---|---|
| `sub_id` | int |

返回：

```json
{
  "ok": "bool"
}
```

### `sub.rate`

改订阅频率。

| 参数 | 类型 |
|---|---|
| `sub_id` | int |
| `rate_hz` | float |

返回：

```json
{
  "ok": "bool"
}
```

## 发布订阅事件

### `state`

周期状态帧。v0.2.0 在原有字段上增加 velocity/motor_errors/stalls/torque/limits/goals/safety。

```json
{
  "event": "\"state\"",
  "sub": "int",
  "arm": "str",
  "seq": "int",
  "ts": "int ms",
  "connected": "bool",
  "hz": "float",
  "joints": "{motor_id: raw}",
  "velocity": "{motor_id: raw}",
  "deg": "{joint: deg}",
  "health": "{motor_id: {voltage, temp}}",
  "torque": "{motor_id: bool}",
  "limits": "{motor_id: {min, max}}",
  "goals": "{motor_id: raw}",
  "motor_errors": "{motor_id: {code, names, message, ts, since_ms}}",
  "stalls": "{motor_id: {motor, reason, position, goal, stall_ms, ts, torque_off, action, reverted_goal}}",
  "safety": "{stall_protection, stall_timeout_ms, stall_tolerance, auto_torque_off, soft_limits, stall_exclude_motors}"
}
```

### `arm`

连接状态变化事件（掉线/恢复）。单电机 error 不触发此事件。

```json
{
  "event": "\"arm\"",
  "sub": "int",
  "arm": "str",
  "connected": "bool",
  "error": "str|null",
  "ts": "int ms"
}
```

### `stall`

堵转保护触发（每电机只发一次，重新使能/新目标/clear_error 后清除）。

```json
{
  "event": "\"stall\"",
  "sub": "int",
  "arm": "str",
  "motor": "u8",
  "reason": "\"stall\"",
  "position": "int raw（触发时位置）",
  "goal": "int raw（触发时的目标）",
  "stall_ms": "int",
  "torque_off": "bool",
  "action": "\"torque_off+goal_revert\" | \"goal_revert\"",
  "reverted_goal": "int（回退后的目标）",
  "ts": "int ms"
}
```

### `motor_error`

单电机硬件 error 或连续无响应首次出现。不影响整臂 connected。

```json
{
  "event": "\"motor_error\"",
  "sub": "int",
  "arm": "str",
  "motor": "u8",
  "code": "u8|null（null=连续无响应）",
  "names": "[str]（hardware_alert / no_response / ...）",
  "message": "str",
  "ts": "int ms"
}
```

### `motor_recovered`

该电机故障清除（重新读到数据 / reboot / clear_error 后）。

```json
{
  "event": "\"motor_recovered\"",
  "sub": "int",
  "arm": "str",
  "motor": "u8",
  "ts": "int ms"
}
```

## 安全层（stall / 软限位 / 故障隔离）

2026-11-22 事故：从臂肘部（motor 3，XL430/XL330）顶住机械限位持续输出力矩 → 舵机 latch hardware error 0x80 → follower worker 读失败/重连 → 串口 fd 未释放，Device or resource busy 无法恢复，只能重启 armd + 物理断电。事故记录见 docs/INCIDENT-2026-11-22.md，冻结规范见 docs/SAFETY-DESIGN.md。v0.2.0 把「不把机械臂顶坏、不让单电机拖垮整臂、不泄漏串口 fd」固化为三道防线。

### 三道防线

#### ① 堵转保护（最后一道防线）

- 触发：每电机 StallTracker：|goal - present| > stall_tolerance(60) 且 |velocity| <= stall_velocity_epsilon(2) 且 torque_on，持续 stall_timeout_ms(400)；stall_exclude_motors 里的电机跳过
- 动作：只对该电机：先 sync_write Torque_Enable=0（auto_torque_off=true 时，默认开）→ goal 回退到 present 附近（夹到 [min+stall_recover_margin, max-stall_recover_margin]）→ state.stalls + stats.stalls + event:"stall"；同臂其他电机完全不受影响
- 观察/验收：arm.state.stalls / event:"stall"；arm.read_register(motor, 64, 1) 应读到 0；不会产生 0x80
- 默认值：`stall_protection=true, stall_timeout_ms=400, stall_tolerance=60, stall_velocity_epsilon=2, auto_torque_off=true, stall_recover_margin=20, stall_exclude_motors=[]（建议夹爪填 [6]）`

#### ② 软限位（写 Goal 前拦截）

- 触发：写 Goal_Position(116)（arm.write_register / arm.sync_write）且 soft_limits=true；生效范围由该电机的 Operating_Mode(11) 决定
- 动作：位置模式(mode 3) 或模式未知：先夹到绝对范围 0..=4095，再按 EEPROM Min/Max Position Limit 夹到 [min+soft_limit_margin, max-soft_limit_margin]；扩展位置模式(mode 4)：只夹到绝对范围 ±1,048,575，不套 EEPROM Min/Max（它只适用于单圈位置模式，否则多圈目标会被错误夹住）。clamp 模式（默认）下发夹紧值并回 clamped/warnings；reject 模式返回 soft_limit 错误、不下发
- 观察/验收：mode 3 下 arm.write_register(116, 5000) → applied=4095-margin 或 soft_limit 错误；mode 4（主臂）允许负值/>4095 的多圈目标；写成功清除该电机 stall 标记。worker 连接时读 Operating_Mode 并每秒刷新、跟踪对 addr 11 的写入（遥操作启动时把从臂切到 3）
- 默认值：`soft_limits=true, soft_limit_mode="clamp", soft_limit_margin=10`

#### ③ 单电机故障隔离 + 串口 fd 生命周期

- 触发：某个舵机状态包 error 位非 0（如 0x80）或连续 ≥10 次轮询无响应
- 动作：sync_read 按电机返回 {data, errors}，单电机 error 只写 motor_errors[id]（事件 motor_error），connected 保持 true、其他电机正常；故障恢复自动清除并发 motor_recovered；Link 保证 open 前先 close 旧句柄、读失败 3 次先 close 再重连，打开失败时 io=None，绝不占着 fd 再 open
- 观察/验收：arm.list[].motor_errors / arm.state.motor_errors；fd 不变量：connected 期间 opens - closes == 1（恰好一个 live 串口句柄），断线释放后二者相等；arm.reconnect ×20 后 lsof 每设备仍只有 1 个 fd；用 sim.inject transport_error 可验证断线后 fd 先释放
- 默认值：`无需配置；connected 语义见 arm.state.connected`

### 恢复流程

#### latched 硬件错误（0x80 hardware_alert）恢复

1. arm.list 或 arm.state 确认 motor_errors[电机].code == 128（names 含 hardware_alert），记下电机 id
2. arm.state.stalls 查看是否由堵转触发；排查机械卡死/限位/NFS 等外部原因
3. arm.reboot(motors=[该电机]) —— Dynamixel Reboot(0x08) 清除 latched 错误（最长等 5s；成功时 torque 自动置 false）
4. arm.clear_error(motors=[该电机]) 只清 armd 侧标记；若 hardware_error_status 仍非 0，warnings 会提示必须 reboot/断电
5. 确认 motor_errors 已空、connected=true 后，再 arm.torque(enable=true) 恢复使能（或重新写目标）

#### 堵转触发后恢复

1. arm.state.stalls[电机] 看 position/goal/stall_ms/reverted_goal；event:"stall" 可实时发现
2. 排除机械卡死：手动活动关节、检查线缆/限位/负载，确认不是软件目标导致顶死
3. 需要继续动作时：先给安全的新目标（arm.write_register 116 或 arm.sync_write，会先过软限位），或在应用里把目标收回到可达范围
4. target 更新成功后该电机 stall 标记自动清除；再 arm.torque(enable=true) 重新使能
5. 反复触发说明负载/限位/标定参数有问题，不要简单关掉 stall_protection

#### 串口 Device or resource busy / fd 泄漏（事故根因）

1. v0.1.0 中读失败后旧 fd 不释放、再次 open 永远 EBUSY（lsof 可见每设备多 fd）。升级 v0.2.0 后该根因已修复
2. 现场处置：升级 armd → arm.reconnect（先 close 再 open，fd 立即释放）→ 仍 busy 时用 lsof /dev/cu.usbmodem* 确认没有其它进程占用（zeropose 直连等）
3. 设备节点消失/仅剩旧 fd：unplug 后重新插上，armd 会自动重连；用 arm.stats.opens/closes 判断是否有泄漏：connected 期间差值为 1（恰好一个 live 句柄），断线释放后二者相等；差值 >1 才说明泄漏
4. latched 0x80 不会因重连消失，必须 arm.reboot 或物理断电；连不上总线时 arm.list.error 会给出 open 错误

### 硬件 error 位（Hardware_Error_Status=70）

| 位 | 名称 | 含义 |
|---|---|---|
| `0x80` | `hardware_alert` | 硬件告警（典型：堵转/过载导致的 latched 错误），需 arm.reboot 清除 |
| `0x20` | `overload` | 过载 |
| `0x10` | `electrical_shock` | 电气冲击 |
| `0x08` | `motor_encoder` | 电机/编码器异常 |
| `0x04` | `overheating` | 过温 |
| `0x01` | `input_voltage` | 输入电压异常 |
| `其他位` | `reserved_bit_N` | 保留位，按位名上报 |

### 安全配置默认值

| 配置项 | 默认 | 含义 |
|---|---|---|
| `stall_protection` | `true` | 堵转保护总开关 |
| `stall_timeout_ms` | `400` | 条件持续多久触发（建议 300–500） |
| `stall_tolerance` | `60` | |Goal-Present| 偏差阈值（counts） |
| `stall_velocity_epsilon` | `2` | 视为静止的 |Present_Velocity| 阈值（X 系列单位 0.229rpm） |
| `auto_torque_off` | `true` | 触发时自动对故障电机 torque OFF |
| `stall_recover_margin` | `20` | 回退目标离 EEPROM 限位内侧的余量（counts） |
| `stall_exclude_motors` | `[]（shipped configs："[6]"）` | 不参与堵转自动保护的电机；建议把夹爪（6）排除：夹住物体时 Goal-Present 长期有偏差属正常，否则会误触发 |
| `soft_limits` | `true` | 写 Goal_Position 时用 EEPROM Min/Max Position Limit 做软限位 |
| `soft_limit_mode` | `"clamp"` | clamp=夹紧+警告；reject=返回 soft_limit 错误 |
| `soft_limit_margin` | `10` | 软限位内侧余量（counts） |

### 注意事项

- 堵转检测依赖轮询窗口里的 Present_Velocity。poll_addr 未显式配置且 stall_protection=true 时，poll 窗口自动用 addr=128、len=8（一次 sync_read 同时读 velocity+position，只多 4 字节/电机）；显式配置 poll_addr/poll_len 时维持旧语义，velocity 未知 → 堵转检测自动跳过并打一条警告日志。
- arm.read/arm.state 的 errors 是「该电机状态包带 error 位」，不是传输失败；完全没响应的电机不出现在 data/errors 中，连续 ≥10 次（约 167ms@60Hz）标记 no_response。
- 夹住物体时夹爪的电机会持续存在 Goal-Present 偏差且 velocity≈0，属正常保持而非堵转。用 stall_exclude_motors 把夹爪（如 6）排除出堵转自动保护；本仓库 configs/koch*.json 已写 [6]。事故中的 motor 3（肘部）不在排除列表里。
- 软限位按 Operating_Mode(11) 生效：位置模式(mode 3) 用 EEPROM Min/Max 且绝对 0..4095；扩展位置模式(mode 4) 只做 ±1,048,575 绝对范围，不套 EEPROM 限位（主臂真实电机是 mode 4）。worker 连接时读 Operating_Mode，每秒刷新并跟踪对 addr 11 的写入（遥操作启动时会把从臂切到 3）。
- 多圈关节（主臂 mode 4）的 state.joints / state.velocity / goals 可以超出 0..4095（实测 pan 6133、roll -2889），这是正常的多圈位置，不是坏值；换算角度时由 profile 的 wrap 逻辑处理。
- fd 不变量：connected 期间恰好有 1 个 live 串口句柄，即 stats.opens - stats.closes == 1；worker 关闭/断线释放 fd 后两者相等。arm.sim.inject 的 transport_error 往整条总线注入传输错误（模拟 USB 掉线），worker 连续 3 次传输失败后先 close 释放 fd 再按 reconnect_sec 重连，可用于验证释放路径；清除注入后恢复。
- 所有新配置项均可省略（serde default），旧 v0.1.0 配置无需改动；旧客户端可忽略新增字段。

## 错误码

| 错误码 | 含义 |
|---|---|
| `bad_request` | 参数缺失/类型错误/校验失败 |
| `no_such_arm` | arm id 不存在（可用 arm.list 查询） |
| `no_such_sub` | 订阅 id 不存在 |
| `unknown_method` | 未知方法 |
| `queue_full` | urgent/normal 队列满，客户端应退避重试 |
| `timeout` | worker 未在期限内回复（总线可能卡住；arm.reboot 最长等 5s） |
| `serial_error` | 串口/舵机错误。v0.2.0 起单电机硬件 error 不再导致整次读失败（只记入 state.motor_errors），这里只剩传输层/整臂错误 |
| `soft_limit` | soft_limit_mode="reject" 时写入的 Goal_Position 超出软限位，整次写入被拒绝 |
| `not_sim` | arm.sim.inject 用在了非仿真臂（sim=false）上 |
| `io_error` | 本地文件读写失败（如 profile 写入） |
| `unauthorized` | 已配置 token，但连接未先 hello |
| `closed` | 客户端已关闭（客户端侧错误） |

## 标定档案 profile

标定档案：raw→URDF 关节角的换算参数。存在 ~/.config/armd/profiles/<arm-id>.json。

| 字段 | 含义 |
|---|---|
| `zero` | {joint: raw} |
| `sign` | {joint: 1|-1} |
| `grip` | open: int；closed: int；angle: float (5~140.5, 可选) |
| `spin` | wrist_roll: float (-180~180, 可选) |
| `savedAt` | int (unix 秒) |

换算公式：

```
非夹爪关节：deg = wrap(raw - zero) % 4096（折到 ±2048）× 360/4096 × sign；夹爪：deg = clamp((raw - closed)/(open - closed), 0, 1) × (angle 或 137.5)
```

## 配置项

| 配置项 | 默认值 | 说明 |
|---|---|---|
| `socket` | /tmp/armd.sock | 兼容字段：未配 listen 时等价 listen=[unix:<socket>] |
| `listen` | [] | 监听地址数组：unix:<path> / tcp://host:port |
| `token` | null | 可选访问令牌 |
| `identity` | /tmp/armd-identity.json 或 /run/armd/identity.json | 身份文件路径 |
| `arms[].id` | — | 对外 arm id，建议 <robot>:<role> |
| `arms[].port` | — | 串口设备路径（sim=true 可空） |
| `arms[].baud` | 1000000 | 波特率 |
| `arms[].poll_hz` | 60 | 轮询频率 |
| `arms[].motors` | [1,2,3,4,5,6] | 轮询的舵机 id |
| `arms[].poll_addr` | 132（未显式配置且 stall_protection=true 时自动用 128） | 轮询寄存器地址。显式配置后维持旧语义（把读到的值当 Present_Position），velocity 未知 → 堵转检测自动跳过 |
| `arms[].poll_len` | 4（未显式配置且 stall_protection=true 时自动用 8） | 轮询寄存器长度。默认安全配置下单次 sync_read 读 addr=128/len=8（Present_Velocity + Present_Position），只多 4 字节/电机 |
| `arms[].monitor` | False | 每秒读电压/温度（寄存器 144/146） |
| `arms[].joint_names` | {} | 舵机 id→关节名，标定档案与角度换算必需 |
| `arms[].profile` | ~/.config/armd/profiles/<arm-id>.json | 标定档案路径 |
| `arms[].sim` | False | 仿真臂（arm.sim.inject 仅对 sim 臂可用） |
| `arms[].reconnect_sec` | 2.0 | 断线重连间隔 |
| `arms[].read_timeout_ms` | 30 | 单次 sync_read 超时 |
| `arms[].stall_protection` | True | 堵转/限位保护总开关（最后一道防线，建议保持开启） |
| `arms[].stall_timeout_ms` | 400 | Goal-Present 偏差 + 静止条件持续多久触发堵转保护（建议 300–500） |
| `arms[].stall_tolerance` | 60 | 判定堵转的 |Goal-Present| 阈值（counts） |
| `arms[].stall_velocity_epsilon` | 2 | 视为静止的 |Present_Velocity| 阈值（X 系列单位 0.229rpm） |
| `arms[].auto_torque_off` | True | 堵转触发时自动对故障电机 torque OFF |
| `arms[].stall_recover_margin` | 20 | 堵转回退目标离 EEPROM 限位内侧的余量（counts） |
| `arms[].stall_exclude_motors` | [] | 不参与堵转自动保护的电机（shipped configs 用 [6] 排除夹爪：夹住物体时 Goal-Present 长期有偏差属正常） |
| `arms[].soft_limits` | True | 写 Goal_Position(116) 时做软限位；按 Operating_Mode：mode 3/未知用 0..4095 + EEPROM Min/Max，mode 4 只用 ±1,048,575 绝对范围（主臂为 mode 4） |
| `arms[].soft_limit_mode` | "clamp" | "clamp"（夹紧+返回警告，默认）或 "reject"（返回 soft_limit 错误） |
| `arms[].soft_limit_margin` | 10 | 软限位内侧余量（counts） |

仓库：Gogs adam/armd, GitHub ColdZoo/armd, 本地 /raid/Projects/armd；客户端：clients/python/armd_client.py（纯标准库；ArmdClient 支持 auto_reconnect 自动重连+重订阅）
CLI：`python3 clients/python/armd_client.py --socket /tmp/armd.sock <info|arms|state|read|stats|watch|torque|write|profile|snapshot|reboot|clear-error|torque-off|safety|sim-inject>`

## 版本

### v0.2.0（2026-11-22）

- 安全层：堵转保护（400ms 默认，torque OFF + goal 回退 + 事件）、Operating_Mode 感知软限位（mode 3 用 0..4095+EEPROM 限位，mode 4 用 ±1,048,575 绝对范围；clamp/reject）、单电机故障隔离（per-motor errors，connected 保持 true）
- 新增 RPC：arm.reboot / arm.clear_error / arm.torque_off / arm.sim.inject；arm.read 带 errors；write/sync_write 带 clamped/warnings
- 新增事件：stall / motor_error / motor_recovered；state 帧新增 velocity/goals/torque/limits/motor_errors/stalls/safety；stats 新增 stalls/torque_offs/opens/closes
- 串口 fd 生命周期修复：open 前先 close，连续读失败先释放 fd 再重连，断线不再 100% CPU 自旋（事故根因）
- 新增配置：stall_* / soft_limit_* / stall_exclude_motors（默认即安全，旧配置兼容）

### v0.1.0（2026-11）

- JSON-RPC 2.0 兼容、token 鉴权、Unix+TCP 双传输
- 标定档案 profile + 历史快照/恢复
- 电压/温度监控、ARM 交叉编译产物
- 订阅限流、单实例锁 + identity 文件
