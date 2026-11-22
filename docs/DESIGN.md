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
