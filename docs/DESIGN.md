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
| `serial_error` | 串口/舵机错误（含舵机硬件 error 位） |
| `closed` | 客户端已关闭 |

## 5. RPC 方法

| 方法 | params | 返回 | 说明 |
|---|---|---|---|
| `ping` | — | `{pong, version, uptime_sec}` | 健康检查 |
| `info` | — | `{version, uptime_sec, socket, arms[]}` | 守护进程信息 |
| `arm.list` | — | `[{id,label,port,sim,motors,connected,hz,seq,error,stats}]` | 所有臂 |
| `arm.state` | `arm` | 状态快照（见上） | 读缓存，不占总线 |
| `arm.read` | `arm, motors?, timeout_ms?` | `{positions:{id:raw}}` | 立刻 sync_read（占总线） |
| `arm.read_register` | `arm, motor, address, length` | `{data,value}` | 读任意寄存器 |
| `arm.write_register` | `arm, motor, address, length, value` | `{ok}` | 写寄存器（等状态包确认） |
| `arm.sync_write` | `arm, address, length, values:{id:value}` | `{ok}` | 同步写多舵机 |
| `arm.torque` | `arm, enable, motors?` | `{ok}` | 扭矩开关（同步写 Torque_Enable） |
| `arm.ping` | `arm, motor` | `{model}` | 单舵机 ping |
| `arm.set_rate` | `arm, hz` | `{ok,hz}` | 改轮询频率（0.5~500） |
| `arm.reconnect` | `arm` | `{ok}` | 强制重开串口 |
| `arm.stats` | `arm` | `{polls,read_errors,writes,frames_published,frames_dropped,reconnects,urgent_queued,normal_queued}` | 计数器 |
| `sub.subscribe` | `arms?, rate_hz?` | `{sub_id, rate_hz, arms}` | 订阅状态流（至少每臂一条） |
| `sub.unsubscribe` | `sub_id` | `{ok}` | 退订 |
| `sub.rate` | `sub_id, rate_hz` | `{ok}` | 改订阅频率 |

## 6. 发布订阅

订阅后服务端按 `rate_hz` 推送：

```json
{"event":"state","sub":3,"arm":"koch_v1_1:follower","seq":124,"ts":1795321234583,
 "connected":true,"hz":60.1,"joints":{"1":1511,"2":612,"3":3298,"4":796,"5":-2889,"6":3203}}
```

连接状态变化会推事件帧（订阅 arms 变化时也会收到）：

```json
{"event":"arm","sub":3,"arm":"koch_v1_1:leader","connected":false,"error":"timeout: ...","ts":...}
```

**速率语义**：`rate_hz >= 54` 时直通（每个轮询帧都推，避免抖动抽帧导致 60Hz 订阅掉到 50fps）；
更低频率按时间抽帧（±抖动可能导致略低于目标）。慢客户端不会阻塞总线：队列满即丢帧。

## 7. 串口/协议实现

- Dynamixel Protocol 2.0，1Mbps 8N1。帧格式：
  - 指令：`FF FF FD 00 ID LEN_L LEN_H INST PARAMS CRC_L CRC_H`（LEN = params + 3）
  - 状态：`FF FF FD 00 ID LEN_L LEN_H 0x55 ERR PARAMS CRC_L CRC_H`（LEN = params + 4）
  - **注意 LEN 是 2 字节小端**；CRC 为 poly 0x8005 / init 0 的非反射 CRC-16（覆盖除末尾 CRC 外整帧）。
- 单元测试用真机抓包字节做断言（ping 回复、位置回复、CRC 值）。
- 轮询用 Sync Read（一条指令读全部 id），解析时跳过无关 id 的包；个别舵机掉线不会导致整臂失败。
- 连接时 ping 全部 `motors` 握手，全响应才置 `connected=true`；连续 3 次读失败才判定掉线并进入重连循环。

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
