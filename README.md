# armd — 机械臂串口守护进程（Rust）

**一条串口总线只允许一个程序碰**。armd 独占主臂/从臂的串口，对上层应用提供
Unix socket 上的 **RPC + 发布订阅**；zeropose 等程序不再直接读串口。

```
┌──────────────┐   ┌──────────────┐   ┌──────────────┐
│  zeropose    │   │  遥操作/取证  │   │  你自己的脚本 │      ← 任意语言，仅 RPC
└──────┬───────┘   └──────┬───────┘   └──────┬───────┘
       └────────── Unix socket (/tmp/armd.sock, 0600) ──────────┘
                              │  NDJSON：RPC + 订阅流
                    ┌─────────┴──────────┐
                    │       armd         │  Rust，单进程
                    │  worker(主臂) ×1   │  每臂一线程、独占一条总线
                    │  worker(从臂) ×1   │  各自命令/响应/发布队列
                    └───┬────────────┬───┘
                /dev/cu.usbmodem5AB01834471   主臂（Dynamixel XL330 ×6）
                /dev/cu.usbmodem5A7C1190831   从臂（XL430 ×2 + XL330 ×4）
```

## 为什么拆

- 串口半双工、独占：两个程序同时开同一条总线会互相打断帧，轻则超时、重则乱序。
- 之前每个客户端各自轮询：60Hz×N 个进程 = 总线负载翻倍，定时抖动也各自承担。
- armd 里**一次轮询 → 分发给所有订阅者**，总线只被一个线程按固定节拍访问。

## 快速开始

```bash
cargo build --release            # 产物在 /tmp/armd-target/release/armd（.cargo/config.toml 指定）
cp target/release/armd ~/.local/bin/   # 或 cargo install --path .

# 1) 无硬件自测（带一个 sim 臂，正弦轨迹）
armd --sim --socket /tmp/armd-sim.sock
python3 clients/python/armd_client.py --socket /tmp/armd-sim.sock watch sim

# 2) 真机（配置见 configs/koch.json，默认读 ~/.config/armd/config.json）
cp configs/koch.json ~/.config/armd/config.json
armd --config ~/.config/armd/config.json
armd_client() { python3 clients/python/armd_client.py --socket /tmp/armd.sock "$@"; }
armd_client arms
armd_client state koch_v1_1:follower
```

air.my 上已装 LaunchAgent `com.armd`（登录自启 + 崩溃重启，日志 `/tmp/armd.log`）：

```bash
launchctl kickstart -k gui/$(id -u)/com.armd    # 重启
launchctl bootout gui/$(id -u)/com.armd         # 停止
```

## 配置（JSON）

```json
{
  "socket": "/tmp/armd.sock",
  "arms": [
    {
      "id": "koch_v1_1:follower",
      "label": "Koch v1.1 从臂",
      "port": "/dev/cu.usbmodem5A7C1190831",
      "baud": 1000000,
      "poll_hz": 60,
      "read_timeout_ms": 30,
      "reconnect_sec": 2.0,
      "motors": [1, 2, 3, 4, 5, 6]
    }
  ]
}
```

| 字段 | 默认 | 说明 |
|---|---|---|
| `id` | — | 对外 arm id；zeropose 用 `<robot>:<role>` |
| `port` | — | 串口设备路径（`sim: true` 时可空） |
| `baud` | 1000000 | 总线波特率 |
| `poll_hz` | 60 | 轮询频率（一次 sync_read 全部电机） |
| `read_timeout_ms` | 30 | 单次 sync_read 超时 |
| `reconnect_sec` | 2.0 | 断线重连间隔 |
| `motors` | 1..6 | 轮询的舵机 id |
| `poll_addr` / `poll_len` | 132 / 4 | 轮询寄存器（默认 Present_Position） |
| `sim` | false | 仿真臂 |

## 客户端

- Python（零依赖，标准库）：[`clients/python/armd_client.py`](clients/python/armd_client.py)
  —— `ArmdClient` 同步 RPC + `StateStream` 订阅最新状态；CLI 支持 `info/arms/state/read/stats/watch/torque/write`。
- 其他语言：协议是 **NDJSON over Unix socket**，照 [`docs/DESIGN.md`](docs/DESIGN.md) 实现即可（十几行）。

## 性能（真机，air.my）

- 总线轮询 60Hz 稳定（两个臂同时在线，各 60Hz，`read_errors=0`）
- 订阅 60Hz 直通（订阅频率 ≥54Hz 时不按时间抽帧），帧间隔均值 16.7ms
- 本机 RPC 往返 <1ms；发送写命令到总线的延迟 ≤2ms（worker 每 2ms 检查一次队列）
- 对比：之前 Python 直连时 macOS 定时器合并会把 60Hz 拖到 20Hz，只能靠自旋补偿

## 结构

```
src/dxl.rs        Dynamixel Protocol 2.0（CRC/帧构造/解析，真机字节流验证）
src/port.rs       串口传输层（真实串口 + sim 实现，同一 trait）
src/arm.rs        每臂 worker：队列/轮询/状态缓存/发布/重连
src/server.rs     Unix socket RPC + 发布订阅（NDJSON）
src/config.rs     配置
clients/python/   Python 客户端
docs/DESIGN.md    架构、RPC 协议全文、队列与背压设计
```
