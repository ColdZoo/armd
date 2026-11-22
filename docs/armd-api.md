# armd API 参考（v0.1.0，更新 2026-11-22）

Dynamixel 机械臂串口守护进程：独占串口总线，对上层提供 NDJSON RPC 与发布订阅；持有标定档案（profile）。

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
{"id": 7, "result": {"seq": 123, "connected": true, "joints": {"1": 1511}, "deg": {"shoulder_pan": 43.24}}}
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
| `arm.list` | 所有机械臂摘要：id/label/port/sim/motors/connected/hz/seq/error/profile_saved_at/stats。 |
| `arm.state` | 读状态缓存，不占总线。deg 只在有 profile 时出现。 |
| `arm.read` | 立刻 sync_read 一次（占用总线），用于即时取值/标定采点。 |
| `arm.read_register` | 读任意控制表寄存器（小端有符号解析）。 |
| `arm.write_register` | 写任意寄存器（等舵机状态包确认）。 |
| `arm.sync_write` | 同步写多个舵机（广播式，不等回包）。 |
| `arm.torque` | 扭矩开关（写 Torque_Enable=64）。 |
| `arm.ping` | 单舵机 ping，返回 model number。 |
| `arm.set_rate` | 改轮询频率。 |
| `arm.reconnect` | 强制重开串口（排查用）。 |
| `arm.stats` | 计数器。 |
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

所有机械臂摘要：id/label/port/sim/motors/connected/hz/seq/error/profile_saved_at/stats。

返回：

```json
{
  "[arm]": "见 arm.state"
}
```

### `arm.state`

读状态缓存，不占总线。deg 只在有 profile 时出现。

| 参数 | 类型 |
|---|---|
| `arm` | str |

返回：

```json
{
  "seq": "int",
  "ts": "int ms",
  "connected": "bool",
  "joints": "{motor_id: raw}",
  "deg": "{joint: deg}",
  "joint_names": "{motor_id: joint}",
  "health": "{motor_id: {voltage, temp}}",
  "hz": "float",
  "read_us": "int",
  "error": "str|null",
  "profile_saved_at": "int|null"
}
```

### `arm.read`

立刻 sync_read 一次（占用总线），用于即时取值/标定采点。

| 参数 | 类型 |
|---|---|
| `arm` | str |
| `motors` | [u8]（可选） |
| `timeout_ms` | int（可选），默认 50 |

返回：

```json
{
  "positions": "{motor_id: raw}"
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
  "value": "int"
}
```

### `arm.write_register`

写任意寄存器（等舵机状态包确认）。

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
  "ok": "bool"
}
```

### `arm.sync_write`

同步写多个舵机（广播式，不等回包）。

| 参数 | 类型 |
|---|---|
| `arm` | str |
| `address` | u16 |
| `length` | u16 |
| `values` | {motor_id: value} |

返回：

```json
{
  "ok": "bool"
}
```

### `arm.torque`

扭矩开关（写 Torque_Enable=64）。

| 参数 | 类型 |
|---|---|
| `arm` | str |
| `enable` | bool |
| `motors` | [u8]（可选） |

返回：

```json
{
  "ok": "bool"
}
```

### `arm.ping`

单舵机 ping，返回 model number。

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

强制重开串口（排查用）。

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

计数器。

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
  "normal_queued": "int"
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

周期状态帧。

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
  "deg": "{joint: deg}",
  "health": "{motor_id: {voltage, temp}}"
}
```

### `arm`

连接状态变化事件（掉线/恢复）。

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

## 错误码

| 错误码 | 含义 |
|---|---|
| `bad_request` | 参数缺失/类型错误/校验失败 |
| `no_such_arm` | arm id 不存在（可用 arm.list 查询） |
| `no_such_sub` | 订阅 id 不存在 |
| `unknown_method` | 未知方法 |
| `queue_full` | urgent/normal 队列满，客户端应退避重试 |
| `timeout` | worker 未在期限内回复（总线可能卡住） |
| `serial_error` | 串口/舵机错误（含舵机硬件 error 位） |
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
| `arms[].monitor` | False | 每秒读电压/温度（寄存器 144/146） |
| `arms[].joint_names` | {} | 舵机 id→关节名，标定档案与角度换算必需 |
| `arms[].profile` | ~/.config/armd/profiles/<arm-id>.json | 标定档案路径 |
| `arms[].sim` | False | 仿真臂 |
| `arms[].reconnect_sec` | 2.0 | 断线重连间隔 |
| `arms[].read_timeout_ms` | 30 | 单次 sync_read 超时 |

仓库：Gogs adam/armd, GitHub ColdZoo/armd, 本地 /raid/Projects/armd；客户端：clients/python/armd_client.py（纯标准库）
