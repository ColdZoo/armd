# armd 设计文档

## 1. 要解决的问题

1. **串口是独占的半双工资源**：多个程序同时打开同一条 Dynamixel 总线，帧会互相插入，
   表现为随机超时/CRC 错/读到别的指令的回复。
2. **每个客户端各自轮询成本高**：以前 ZeroPose 每个会话 60Hz sync_read；再开遥操作脚本就
   变成两条总线流量。
3. **Python 定时抖动**：macOS 后台 QoS 下 `sleep(16ms)` 会被合并到 ~50ms，只能靠
   「1ms 小睡 + 1.5ms 自旋」硬顶到 60Hz（烧 CPU、仍是尽力而为）。

结论：把「跟串口说话」抽成常驻守护进程（Rust），总线只被一个线程按固定节拍访问；
上层应用只做 RPC/订阅，任何语言都能接。

## 2. 进程与线程模型

```
armd 进程
├── main thread        监听 Unix socket，accept 连接
├── conn-N thread      每个客户端连接一个线程：解析请求、写响应、维护订阅
├── writer-N thread    每条连接一个写线程（响应与推送统一出口，保证不交织）
└── arm worker thread  每条总线一个：独占串口，执行命令 + 定时轮询 + 扇出发布
```

- **一条总线 = 一个 arm id = 一个 worker**。worker 是唯一碰串口的代码。
- 客户端可多开（zeropose 两个页签、遥操作脚本、调试 CLI 同时连），互不影响；
  写命令按到达顺序串行执行。
- 进程内没有任何跨臂共享的可变状态（除了 daemon 的订阅注册表）。

## 3. 队列设计（性能/实时性的核心）

每个 worker 有三类队列：

| 队列 | 容量 | 消费策略 | 用途 |
|---|---|---|---|
| `urgent` | 64 | 每轮**优先全清** | 写寄存器、扭矩、sync_write、重连（要求尽快生效） |
| `normal` | 256 | 每轮最多 16 条 | 读快照、ping、改速率（不能饿死轮询） |
| pub（每订阅者一条） | 32 | 满了**丢最旧**并计数 | 状态帧推送（慢客户端不拖累总线） |

- **轮询合并**：不是「读请求排队」，而是 worker 按 `poll_hz` 自己定时 sync_read 一次，
  结果写入状态缓存（`seq/ts/joints/hz/error`），所有订阅者共享这一次总线读取。
- **写不排队等待读**：写命令到达后 ≤2ms 内被处理（worker 睡眠上限 2ms），等舵机状态包确认后回 RPC。
- **背压**：RPC 队列满 → 直接返回错误 `queue_full`（不阻塞总线）；订阅队列满 → 丢帧
  （`frames_dropped` 计数），实时性优先。
- **节拍**：`next_poll += interval` 的漂移无关调度；落后时立即补一次但不会连发。

## 4. 传输协议

- **Unix domain socket**（默认 `/tmp/armd.sock`，权限 `0600`）。不上 TCP：本机使用、零配置、无鉴权风险。
- **NDJSON**：一行一个 JSON。请求/响应按 `id` 配对；推送帧带 `event` 字段、无 `id`。
- 一连接上可以同时做 RPC 和订阅（响应与推送由同一写线程写出，不会交错半行）。

### 请求

```json
{"id": 7, "method": "arm.state", "params": {"arm": "koch_v1_1:follower"}}
```

### 响应

```json
{"id": 7, "result": {"seq": 123, "ts": 1795321234567, "connected": true,
                     "joints": {"1": 1511, "2": 612, "3": 3298, "4": 796, "5": -2889, "6": 3203},
                     "hz": 60.1, "read_us": 812, "arm": "koch_v1_1:follower"}}
```

错误：

```json
{"id": 7, "error": {"code": "no_such_arm", "message": "unknown arm 'x'，可用：[...]"}}
```

| code | 含义 |
|---|---|
| `bad_request` | 参数缺失/类型错误 |
| `no_such_arm` | arm id 不存在 |
| `no_such_sub` | 订阅 id 不存在 |
| `unknown_method` | 未知方法 |
| `queue_full` | urgent/normal 队列满（客户端应退避重试） |
| `timeout` | worker 未在期限内回复（总线可能卡住） |
| `serial_error` | 串口/舵机错误。v0.2.0 起单电机硬件 error 不再导致整次读失败（只记 `state.motor_errors`），只剩传输层/整臂错误 |
| `soft_limit` | `soft_limit_mode="reject"` 时目标超出软限位，写入被拒绝 |
| `not_sim` | `arm.sim.inject` 用在非仿真臂（`sim=false`） |
| `closed` | 客户端已关闭 |

## 5. RPC 方法

| 方法 | params | 返回 | 说明 |
|---|---|---|---|
| `ping` | — | `{pong, version, uptime_sec}` | 健康检查 |
| `info` | — | `{version, uptime_sec, socket, arms[]}` | 守护进程信息 |
| `arm.list` | — | `[{id,label,port,sim,motors,connected,hz,seq,error,profile_saved_at,motor_errors,stalls,stats}]` | 所有臂摘要（v0.2.0 加安全层摘要） |
| `arm.state` | `arm` | 状态快照 + `velocity/goals/torque/limits/motor_errors/stalls/safety`（见 §6） | 读缓存，不占总线 |
| `arm.read` | `arm, motors?, timeout_ms?` | `{positions:{id:raw}, errors:{id:error_byte}}` | 立刻 sync_read（占总线）；按电机返回硬件 error，单电机故障不整次失败 |
| `arm.read_register` | `arm, motor, address, length` | `{data,value}` | 读任意寄存器 |
| `arm.write_register` | `arm, motor, address, length, value` | `{ok, applied?, clamped?, warnings?}` | 写寄存器（等状态包确认）；写 Goal(116) 先过软限位 |
| `arm.sync_write` | `arm, address, length, values:{id:value}` | `{ok, clamped?, warnings?, torque?}` | 同步写多舵机；Goal(116) 同样过软限位（reject 模式整体拒绝） |
| `arm.torque` | `arm, enable, motors?` | `{ok, torque:{id:bool}}` | 扭矩开关（同步写 Torque_Enable）；使能时清除该电机 stall 标记 |
| `arm.ping` | `arm, motor` | `{model}` | 单舵机 ping |
| `arm.set_rate` | `arm, hz` | `{ok,hz}` | 改轮询频率（0.5~500） |
| `arm.reconnect` | `arm` | `{ok}` | 强制关串口再重开（v0.2.0 先 close 释放 fd 再 open） |
| `arm.torque_off` | `arm, motors?` | `{ok, torque:{id:bool}}` | 按电机 torque OFF（应急卸力矩；等价 `arm.torque enable=false`） |
| `arm.reboot` | `arm, motors?` | `{ok,rebooted,failed,errors,failures,torque,limits,warnings}` | Dynamixel Reboot(0x08)，清 latched 硬件错误（如 0x80），最长 5s |
| `arm.clear_error` | `arm, motors?` | `{ok,cleared,hardware_error_status,warnings}` | 只清 armd 侧标记 + 读 Hardware_Error_Status(70)；latched 仍需 reboot |
| `arm.sim.inject` | `arm, kind, motors?, enable?` | `{ok,kind,motors,enable,sim:true}` | 仅 sim 臂：stall/hardware_error/no_response/transport_error/clear |
| `arm.stats` | `arm` | `{polls,read_errors,writes,frames_published,frames_dropped,reconnects,urgent_queued,normal_queued,stalls,torque_offs,opens,closes}` | 计数器；fd 不变量：connected 时 `opens-closes==1`，释放后相等 |
| `sub.subscribe` | `arms?, rate_hz?` | `{sub_id, rate_hz, arms}` | 订阅状态流（至少每臂一条） |
| `sub.unsubscribe` | `sub_id` | `{ok}` | 退订 |
| `sub.rate` | `sub_id, rate_hz` | `{ok}` | 改订阅频率 |

## 6. 发布订阅

订阅后服务端按 `rate_hz` 推送：

```json
{"event":"state","sub":3,"arm":"koch_v1_1:follower","seq":124,"ts":1795321234583,
 "connected":true,"hz":60.1,"joints":{"1":1511,"2":612,"3":3298,"4":796,"5":-2889,"6":3203}}
```

`event:"state"` 帧在原有字段上增加安全层字段（v0.2.0，字段含义同 `arm.state`）：
`velocity / goals / torque / limits / motor_errors / stalls / safety`。

连接状态变化会推事件帧（订阅 arms 变化时也会收到）；**单电机故障不触发此事件**：

```json
{"event":"arm","sub":3,"arm":"koch_v1_1:leader","connected":false,"error":"timeout: ...","ts":...}
```

安全层事件（v0.2.0）：

```jsonc
{"event":"stall","sub":3,"arm":"koch_v1_1:follower","motor":3,"reason":"stall",
 "position":3300,"goal":3386,"stall_ms":401,"torque_off":true,
 "action":"torque_off+goal_revert","reverted_goal":3300,"ts":1795361000000}
{"event":"motor_error","sub":3,"arm":"koch_v1_1:follower","motor":3,"code":128,
 "names":["hardware_alert"],"message":"motor 3 ... 用 arm.reboot 清除","ts":...}
{"event":"motor_recovered","sub":3,"arm":"koch_v1_1:follower","motor":3,"ts":...}
```

- `stall`：堵转保护触发（每电机一次，直到重新使能/给新目标/clear_error）。`action` 为
  `torque_off+goal_revert`（auto_torque_off=true，默认）或 `goal_revert`。
- `motor_error`：单电机硬件 error（`code` 为状态包 error 字节，如 0x80）或连续无响应
  （`code=null`，`names=["no_response"]`）首次出现。
- `motor_recovered`：该电机恢复正常。

**速率语义**：`rate_hz >= 54` 时直通（每个轮询帧都推，避免抖动抽帧导致 60Hz 订阅掉到 50fps）；
更低频率按时间抽帧（±抖动可能导致略低于目标）。慢客户端不会阻塞总线：队列满即丢帧。

## 7. 串口/协议实现

- Dynamixel Protocol 2.0，1Mbps 8N1。帧格式：
  - 指令：`FF FF FD 00 ID LEN_L LEN_H INST PARAMS CRC_L CRC_H`（LEN = params + 3）
  - 状态：`FF FF FD 00 ID LEN_L LEN_H 0x55 ERR PARAMS CRC_L CRC_H`（LEN = params + 4）
  - **注意 LEN 是 2 字节小端**；CRC 为 poly 0x8005 / init 0 的非反射 CRC-16（覆盖除末尾 CRC 外整帧）。
- 单元测试用真机抓包字节做断言（ping 回复、位置回复、CRC 值）。
- 轮询用 Sync Read（一条指令读全部 id），解析时跳过无关 id 的包；个别舵机掉线不会导致整臂失败。
- v0.2.0 轮询窗口默认 `addr=128 len=8`（Present_Velocity + Present_Position，一次读回），
  写 Goal 用 `addr=116 len=4`；显式配置 `poll_addr/poll_len` 时维持旧语义（velocity 未知 → 堵转检测跳过）。
- 连接时 ping 全部 `motors` 握手，**≥1 个响应即 `connected=true`**（缺的电机记 per-motor
  `no_response`，不拖垮整臂）；0 个响应则立即 close 释放 fd，等下个 `reconnect_sec` 重试。
- 连续 3 次**传输层**读失败才判定掉线并进入重连循环，且**先 close 释放 fd 再重连**。
- `sync_read` 只有传输层失败才返回 Err；单电机状态包 error 记入 `errors[id]`，不再整次失败。

## 8. 迁移：ZeroPose 接入

- `zeropose/armd_backend.py`：把 armd 订阅流包装成 `BaseBackend`（`open/read/close`）。
- CLI：`python -m zeropose --armd /tmp/armd.sock`；不传 `--armd` 时保持旧的直连串口模式。
- arm id 约定 `<robot>:<role>`，与 ZeroPose 的「型号-主从」一致。
- LaunchAgent：`com.armd` 先起（独占串口），`com.zeropose.web` 加 `--armd`；
  zeropose 启动时若 socket 未就绪会每 3s 自动重试。

## 9. 单实例、身份文件、防抢占（借鉴 robotd）

- **串口独占的两层**：`serialport` 已经设了 `TIOCEXCL` + 独占 flock（非 root 第二次打开会 EBUSY），
  但 root 不受影响 —— 所以 armd 自己在打开串口前先拿 `<socket>.lock` 的 `File::try_lock()`：
  第二个实例直接启动失败，绝不出现"两个 armd 各读一半状态包"。
  锁文件**永不删除**（删了会出现两个 inode 的竞争者），进程退出内核自动释放。
- **不抢占活着的 socket**：启动时如果 socket 路径存在，先尝试连接：
  能连上 → 说明另一个 daemon 活着，拒绝启动；连不上 → 判定为崩溃残留，才删除重建。
- **identity 文件**（Linux `/run/armd/identity.json`，macOS `/tmp/armd-identity.json`）：
  `{service, version, pid, started_ms, listen, token_required, arms}`。
  工具可以据此确认"当前到底是谁在跑、什么版本、监听哪些地址"。
- **systemd**：`Restart=always`、`KillSignal=SIGTERM`、`TimeoutStopSec=10`；
  与旧串口程序互斥用 `Conflicts=`（模板里给了注释示例）。

## 10. ARM 下位机适配

- 同一个二进制跑在 macOS（air.my）或 ARM Linux（下位机）：传输层只依赖 Unix socket / TCP，
  串口层只依赖 Linux termios / macOS 的 tty（`serialport` crate 两者都支持）。
- 监听列表 `listen: ["unix:/run/armd.sock", "tcp://0.0.0.0:8771"]`，Unix 给本机工具、TCP 给网络客户端。
- `config.token` 设置后，客户端必须先 `hello` 带 token，否则所有方法返回 `unauthorized`。
- 只读遥测：`monitor: true` 时每秒额外 `sync_read` 一次 `Present_Input_Voltage(144,2B) + Present_Temperature(146,1B)`，
  在 `arm.state.health` 里给出 `{voltage, temp}`，供下位机做掉电/过温保护。
- 交叉编译：`scripts/build-arm.sh`（`aarch64-unknown-linux-gnu`，支持 cargo-zigbuild 或板上编译）。

## 11. 参考实现对比（2026-11 调研）

| 项目 | 做了什么 | 我们借鉴了什么 | 没照搬的原因 |
|---|---|---|---|
| [pollen-robotics/microduck](https://github.com/pollen-robotics/microduck)（`docs/design/robotd-design.md`、`architecture.md`） | 轮足机器人板载 `robotd`：单进程单总线 50Hz 控制环，JSON-RPC 2.0 + NDJSON over unix socket，intents 进 / state 出，订阅限流，safety 层独占总线，单实例锁 + identity.json + systemd `Conflicts=` | 单实例锁 + 防抢占 socket + identity 文件；JSON-RPC 2.0 兼容；订阅限流；"控制环线程不能被 IPC 挡住"的分层 | 它是板载闭环+策略推理（intents/safety），我们目前是标定/遥操作取数，先保持底层寄存器 API |
| [pollen-robotics/rustypot](https://github.com/pollen-robotics/rustypot)、[createra-robotics/rust_servocom](https://github.com/createra-robotics/rust_servocom) | Rust 的 Dynamixel/Feetech 协议库（v1/v2、sync read/write、控制表、扫描工具、pyo3 绑定） | 协议层与"控制器拥有 io"分层的写法；扫描/寄存器工具的思路；pyo3 绑定可替代手写客户端 | 我们只需要 X 系列 + protocol 2.0，自实现 ~300 行更可控；后续可切 rustypot 减少维护 |
| [dora-rs](https://dora-rs.ai/) | 分布式数据流：topic/service/action/streaming 四种模式，本地共享内存 + 跨机 Zenoh | 四种通信模式的划分（我们目前只有 RPC + topic 推送）；跨机通信用 Zenoh 的取舍 | 引入 dora 会把整个运行时带进来；NDJSON 更轻 |
| lerobot `unitree_g1/unitree_sdk2_socket.py` | ZMQ PUSH/SUB，`CONFLATE=1`，低频命令 + 高频最新状态 | **状态只保留最新一帧**（我们的 per-subscriber 丢帧队列等价）；命令通道与状态通道分离 | ZMQ 需要额外依赖；现有 NDJSON 在 60Hz 足够 |
| [autonomous-ai/lampOS](https://github.com/autonomous-ai/lampOS)（`robots/so101/ROBOT.md`） | 家庭机器人网关：`autonomous.device.v1` schema，websocket 协议，policy/camera/system 分路由 | 设备清单 schema 化的思路；websocket 作为前端实时通道 | 我们是"驱动"不是"网关"；websocket 前端由 zeropose 自己做 |
| [DipFlip/telegrip](https://deepwiki.com/DipFlip/telegrip/8-api-reference) | SO100 遥操作：REST（配置/状态）+ WebSocket（VR 控制流） | HTTP/WS 双通道的取舍（静态/低频 Rest，实时走 WS） | 我们的实时流走 pub/sub 更直接 |
| [jess-moss/koch-v1-1](https://github.com/jess-moss/koch-v1-1)、[strands-labs/robots](https://github.com/strands-labs/robots/blob/main/docs/robots/koch.md) | Koch v1.1 原始硬件/驱动、lerobot `DynamixelDriver` 封装 | 电机型号表（XL430×2 + XL330×4）与我们一致；`DynamixelDriver` 这种"设备对象"命名 | 无需移植 |
| [IliaLarchenko/robot_MCP](https://github.com/IliaLarchenko/robot_MCP) | 把 SO-ARM100 包装成 MCP tool 给 LLM 用 | 未来给 LLM/agent 用可以加一层 MCP 适配（不放进 armd 核心） | 驱动层保持最小 |

## 12. 标定档案（profile）与 ZeroPose 迁移

**原则：标定数据的唯一事实源是 armd**，应用不再各存一份 JSON。

- schema（与 zeropose 兼容）：
  `{zero:{joint:raw}, sign:{joint:±1}, grip:{open,closed,angle?}, spin:{wrist_roll?}, savedAt}`
- 校验在 armd 里做（`src/profile.rs`）：raw 0..4095、sign ±1、夹爪跨度 ≥100、angle 5..140.5°；
  关节覆盖按配置里的 `joint_names` 检查。
- 持久化：每臂一个文件（默认 `~/.config/armd/profiles/<arm-id>.json`），临时文件 + rename 原子替换。
- `arm.state` 附带 `deg`（用 profile 换算的关节角，连续关节做 ±2048 wrap）和 `health`；
  其他应用（zeropose、遥操作脚本、可视化）不需要自己实现 raw→角度换算。
- ZeroPose 的接线（`zeropose/armd_backend.py` + `zeropose/server.py`）：
  - `/api/alignment` 的 GET/POST/DELETE 直接转发到 `arm.profile.*`；
  - 第一次连上 armd 时，若 armd 没有 profile、本地 `~/.config/zeropose/<robot>-<role>.json` 存在 → 一次性推送迁移；
  - 保存时同时写一份本地镜像（备份/可回退），读取优先 armd；
  - 订阅帧超过 1 秒没更新（armd 重启/断流）→ 当作掉线，会话自动重连并重新迁移/读取。

## 13. 安全层（v0.2.0）：堵转检测算法

**背景**：v0.1.0 只转发 Goal，没有堵转/限位检测。2026-11-22 从臂肘部（motor 3，XL430/XL330）
顶住机械限位持续输出力矩 → 舵机 latch `hardware error 0x80` → armd worker 读失败/重连循环，
串口 fd 未释放 → `Device or resource busy` 永久无法恢复，只能重启 armd + 物理断电。
v0.2.0 把「不把机械臂顶坏、不让单电机拖垮整臂、不泄漏串口 fd」固化为三道防线。
冻结规范：`docs/SAFETY-DESIGN.md`；事故记录：`docs/INCIDENT-2026-11-22.md`。

**堵转判定（每电机独立 `StallTracker`）**：

```
torque_on == true
  && goals[id] 已知
  && velocity[id] 已知
  && |goal - present| > stall_tolerance        (默认 60 counts)
  && |velocity| <= stall_velocity_epsilon      (默认 2，X 系列单位 0.229rpm)
  && 上述条件持续 >= stall_timeout_ms          (默认 400ms)
  → 判定该电机堵转，每电机只触发一次（reset 后才可再次触发）
```

`stall_exclude_motors` 里的电机不参与自动保护（shipped configs 为夹爪 `[6]`：夹住物体时
Goal-Present 长期有偏差属正常保持，否则会误触发）。

**触发动作（worker 线程内、当次 poll 中完成，只影响该电机）**：

1. `auto_torque_off=true`（默认）时 `sync_write(Torque_Enable=64, [[id,0]])` 卸力矩，
   `torque[id]=false`，`stats.torque_offs+=1`；
2. `sync_write(Goal_Position=116, [[id, revert]])`，`revert = clamp(present,
   [min+stall_recover_margin, max-stall_recover_margin])`（限位未知时取 present）；
3. 记 `state.stalls[id]`、`stats.stalls+=1`，发 `event:"stall"`（`action` 为
   `torque_off+goal_revert` 或 `goal_revert`），下一次 `publish_state` 自然带出；
4. 同臂其他电机完全不受影响，不会产生 hardware error。

**状态/事件字段**：`arm.state` / 订阅 `state` 帧带 `velocity/goals/torque/limits/
motor_errors/stalls/safety`；`safety` 是生效配置摘要
`{stall_protection, stall_timeout_ms, stall_tolerance, auto_torque_off, soft_limits,
stall_exclude_motors}`。事件见 §6。

**恢复流程**：见 `event:"stall"` / `arm.state.stalls` → 排除机械卡死 → 给安全的新目标
（写入即清除该电机 stall 标记；新目标会先过软限位）→ `arm.torque(enable=true)` 重新使能。
反复触发应先查负载/限位/标定，而不是关掉 `stall_protection`。

**默认配置**（全部 `#[serde(default)]`，旧 v0.1.0 配置无需改动）：

| 字段 | 默认 | 含义 |
|---|---|---|
| `stall_protection` | `true` | 堵转保护总开关 |
| `stall_timeout_ms` | `400` | 条件持续多久触发（建议 300–500） |
| `stall_tolerance` | `60` | \|Goal-Present\| 偏差阈值（counts） |
| `stall_velocity_epsilon` | `2` | 视为静止的 \|Present_Velocity\| 阈值 |
| `auto_torque_off` | `true` | 触发时自动 torque OFF 该电机 |
| `stall_recover_margin` | `20` | 回退目标离 EEPROM 限位内侧的余量 |
| `stall_exclude_motors` | `[]`（shipped：`[6]`） | 不参与自动保护的电机（夹爪） |
| `soft_limits` | `true` | 写 Goal 时做软限位 |
| `soft_limit_mode` | `"clamp"` | `clamp`（夹紧+警告）/ `reject`（报错） |
| `soft_limit_margin` | `10` | 软限位内侧余量（counts） |

**轮询窗口**：`poll_addr` 未显式配置且 `stall_protection=true` 时自动用 `(128, 8)`，
一次 sync_read 同时取 velocity+position（只多 4 字节/电机）；显式配置了 `poll_addr/poll_len`
则维持旧语义，velocity 未知 → 堵转检测自动跳过并在启动时打一条警告日志。

## 14. 单电机故障隔离（per-motor errors）

**v0.1.0 问题**：`read_status` 收到 `st.error != 0` 立即返回 `Err(IoError::Device)`，`sync_read`
把它当整次读失败；`poll` 连续 3 次错误即 `connected=false` 并清空状态流——一个舵机的
硬件告警会拖垮整臂 worker（事故中的 0x80）。

**v0.2.0 语义**：

- `sync_read` 返回 `{data: {id: bytes}, errors: {id: error_byte}}`。**只有传输层失败**
  （写串口失败/总线无任何回包 → Timeout）才返回 `Err`；某电机状态包带 error 位时记入
  `errors[id]`，继续收其他包。完全没响应的电机不出现在 `data`/`errors` 中。
- 硬件 error 的电机只返回 `errors`（`data` 里没有该电机位置——位置不可信）。
- `arm.read` 因此返回 `positions` **和** `errors`（`{motor_id: error_byte}`）；RPC/订阅都不再
  因单电机 error 整次失败。
- 每电机状态写入 `state.motor_errors[id] = {code, names, message, ts, since_ms}`：
  - 硬件 error：`code` 为状态包 error 字节（如 `128`/`0x80`），`names` 含 `hardware_alert` 等；
  - 连续无响应：连续 ≥10 次轮询（约 167ms@60Hz）标记 `code=null, names=["no_response"]`。
- 首次出现发 `event:"motor_error"`；此前有 error 的电机重新读到数据 → 清除并发
  `event:"motor_recovered"`；`connected` **保持 true**，其他电机/订阅正常运行。
- arm 级 `connected:false` 只留给整臂/传输层错误（掉线、全部无响应等）。
- **latched 硬件错误（典型 0x80）不会因重连自动清除，必须 `arm.reboot`（Reboot 0x08）或物理断电**；
  `arm.clear_error` 只清 armd 侧标记并读 `Hardware_Error_Status(70)` 验证，仍有错误时 warnings
  会提示用 reboot。

## 15. 串口 fd 生命周期与重连

**v0.1.0 根因**（事故 C）：`Worker::open_io` 先 `DxlPort::open` 成功后才覆盖 `self.io`；
`ensure_connected` 在 `io=Some` 时直接再次 open；`poll` 失败后**保留** `self.io`。串口独占
（`TIOCEXCL`）使第二次 open 永远 `Device or resource busy`，旧 fd 永不释放 → lsof 可见 fd 223 /
现场 fd 4u 仍挂在已消失的设备节点上。同时断线后 `!connected` 时 `poll()` 被跳过、`next_poll`
永不过期 → `spin_loop()` 死循环（实测 CPU 100.2%）。

**v0.2.0 设计**：

- `Link` 是串口句柄生命周期的唯一入口，**open 前必须先 close（drop 旧 handle → close(fd)）**：

  ```rust
  pub struct Link { io: Option<Box<dyn ArmIo>> }
  impl Link {
      pub fn is_open(&self) -> bool;
      pub fn io(&mut self) -> Option<&mut (dyn ArmIo + 'static)>;
      pub fn close(&mut self) -> bool;  // 返回是否真的关了一个
      /// 先 close（若存在），再 open()；open() 失败时保证 self.io == None。
      pub fn open_with<F: FnOnce() -> IoResult<Box<dyn ArmIo>>>(&mut self, open: F) -> IoResult<()>;
  }
  ```

- 读失败 3 次先 `Link::close()`（立即释放 fd）再等 `reconnect_sec` 重连；打开失败路径
  保证 `io=None`，绝不出现「自己占着 fd 又永远 open 不了」。
- worker 退出（stop）前 `Link::close()`；重连后不保留旧 goals/stalls（舵机可能已断电重启，
  绝不自动把旧目标再推一遍）。
- 断线时 `run()` 每轮至少 sleep 2ms 并推进 `next_poll`，不再 100% CPU 自旋。
- `stats.opens/closes` 由 worker 计数。**fd 不变量：connected 期间恰好 1 个 live 串口句柄，
  即 `opens - closes == 1`；worker 关闭/断线释放后二者相等**（差值 >1 才是泄漏）。
- 验证：单元测试 100 次 open/close + open 失败注入（Drop 计数证明「open 前旧 handle 已释放」）；
  `arm.reconnect` ×20 后 lsof 每设备 1 个 fd；`scripts/fd_reconnect_test.py` 对运行中的 armd
  做 N 次 reconnect 并断言 fd 恒为 1。`arm.sim.inject kind="transport_error"` 可注入整条总线
  传输错误，验证「先释放 fd 再重连」路径。

**busy 排障**：`lsof | grep usbmodem` / `fuser -v /dev/ttyACM0` 确认没有其它进程（zeropose 直连、
旧 armd 实例）占用；升级 v0.2.0 后现场处置顺序为 `arm.reconnect` → 插拔 USB → 仍 busy 则
重启 armd（`launchctl kickstart -k gui/$(id -u)/com.armd`）。不要通过删锁文件解决。

## 16. Dynamixel Reboot / clear_error / torque_off

- `INST_REBOOT = 0x08`（Protocol 2.0）。`arm.reboot` 对指定电机（默认全部）发 Reboot：
  - ids 等于全部电机时用广播 `0xFE` 一次，否则逐个发；
  - 等 ~600ms（期间设置 `suppress_missing_until`，poll 不把缺包误记 no_response），
    然后逐个 ping（最长约 2s）；
  - 成功的电机：清 `motor_errors`/`stalls`/`goals`，`torque[id]=false`（RAM 复位后默认 OFF），
    重新读 limits/torque/position；
  - 失败标记 `no_response` / 仍报 error。**不关闭串口**。
  - 返回 `{ok, rebooted[], failed[], errors{id:str}, failures{id:str}, torque{}, limits{}, warnings[]}`，
    RPC 等待上限 5000ms。`failures` 是重启后 ping 失败的可读原因（`failed` 是 id 列表）。
- `arm.clear_error`：清 armd 侧 `motor_errors`/`stalls` 标记 + 读每个 id 的
  `Hardware_Error_Status(70,1)`；若仍非 0 → warnings 提示用 `arm.reboot`（RPC 等待 2000ms）。
  **不替代 reboot**：latched 错误只有 Reboot/断电能清。
- `arm.torque_off`：按电机 torque OFF 的便捷方法（等价 `arm.torque enable=false`，默认全部），
  用于应急卸力矩；RPC 等待 1000ms。
- `arm.sim.inject`（仅 sim）：`stall` / `hardware_error`（error=0x80 直到 reboot）/
  `no_response` / `transport_error`（整条总线传输错误，模拟 USB 掉线，验证 fd 释放）/
  `clear`（清除全部注入）。非 sim 臂返回 `not_sim`。

## 17. 软限位（Operating_Mode 感知）

写 `Goal_Position(116)`（`arm.write_register` / `arm.sync_write`）时先过软限位（`soft_limits=true`）：

- worker 连接时读 `Operating_Mode(11,1)`，之后每秒刷新，并跟踪对 addr 11 的写入
  （遥操作启动时会把从臂切到 POSITION(3)）；
- **mode 3（位置控制，从臂）/ 模式未知**：先夹到绝对范围 `0..=4095`，再按 EEPROM
  `Min/Max Position Limit(48/50)` 夹到 `[min+soft_limit_margin, max-soft_limit_margin]`
  （若该区间为空取两者中点）；
- **mode 4（扩展位置控制，主臂真实电机是 mode 4）**：绝对范围 `±1,048,575`，
  **不套 EEPROM Min/Max Position Limit**（该限位只适用于单圈位置模式，否则多圈目标会被错误夹住）；
- EEPROM 限位读取失败/未知：只给 warning（`motor N 限位未知（EEPROM 读取失败）`），
  不阻止写入，绝对范围仍生效；
- `soft_limit_mode="clamp"`（默认）：把夹紧后的值下发，响应附
  `clamped`/`warnings`（单电机 `{requested, applied, min, max}`；sync_write 为按电机的对象）；
  `soft_limit_mode="reject"`：整次写入拒绝，返回 RPC 错误 `soft_limit`（sync_write 任一电机
  越界即整体拒绝）；
- 写成功记录 `state.goals[id]`（夹紧后的实际值）并清除该电机 stall 标记
  （新的显式目标 = 重新使能意图）。

**连续关节 wrap 注意事项**：多圈关节（主臂 mode 4）的 `state.joints`/`velocity`/`goals`
可以超出 `0..4095`（实测 pan 6133、roll -2889），这是正常的多圈位置，不是坏值；
角度换算仍由 profile 的 `wrap(raw - zero) % 4096` 处理，不要对 raw 做 0..4095 假设或
二次 clamp。夹爪（mode 3）用 EEPROM 限位 + 软限位约束行程。
