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

## 部署到 ARM 开发板（下位机）

目标拓扑：**机械臂的串口插在 ARM 板上，板上跑 armd；air.my / 手机 / 其他设备通过网络连它。**

```
   air.my（zeropose / 遥操作 / 脚本）        手机 / 平板
        │  tcp://arm-board.my:8771              │ tcp://arm-board.my:8771
        └───────────────┬───────────────────────┘
                        ▼
              ┌────────────────────┐
              │  ARM 开发板（Linux） │  armd（systemd 服务）
              │  /dev/ttyACM0/1     │  ← 独占两条 Dynamixel 总线
              └─────────┬──────────┘
                  主臂 / 从臂
```

步骤：

```bash
# 1) 编译：优先在板上直接 cargo build --release（最简单）
#    交叉编译：./scripts/build-arm.sh   （cargo-zigbuild 或现成 aarch64 工具链）
# 2) 拷到板上安装（systemd 单元 + 配置模板）
scp target/release/armd arm-board:/tmp/
scp configs/koch-arm-board.json arm-board:/tmp/config.json
ssh arm-board 'cd /tmp && git clone … 或 scp deploy/ -r /tmp/'
ssh arm-board 'sudo ./deploy/install-arm.sh /tmp/armd /tmp/config.json'
# 3) 串口权限 + 屏蔽串口控制台（Armbian 常见坑）
ssh arm-board 'sudo usermod -aG dialout armd; sudo systemctl mask serial-getty@ttyS2'
# 4) 检查
ssh arm-board 'systemctl status armd; cat /run/armd/identity.json; journalctl -u armd -n 20'
```

客户端侧（网络）：

```bash
# 配置里 listen 加了 tcp://0.0.0.0:8771 之后：
python3 clients/python/armd_client.py --socket tcp://arm-board.my:8771 --token $ARMD_TOKEN arms
# zeropose 直接走网络后端（ARMD_TOKEN 用环境变量传）：
ARMD_TOKEN=$ARMD_TOKEN python -m zeropose --armd tcp://arm-board.my:8771
```

注意事项：

- **只有板上的 armd 能碰串口**：air.my 上不要再起 armd（或把它的 `arms` 清空），否则两边会抢总线
  （两个 armd 各自有单实例锁，但锁是"同一台机器同一路径"，跨机器靠人工约束）。
- **网络一定要开 token**（`config.token`），否则局域网内任何设备都能写舵机寄存器。
- 下位机没有桌面，`on_disconnect` 行为要提前想好：网络断了，舵机是保持最后姿态还是松力矩 ——
  目前 armd 默认维持 `Torque_Enable` 不变（不主动上电/断电），上电保持姿态属于应用层策略。
- 网络传输（NDJSON，60Hz×6 电机 ≈ 1–2 KB/s）带宽压力很小；但延迟从本机 <1ms 变成 Wi-Fi/LAN 的 2–20ms，
  遥操作类应用建议在客户端做插值/前瞻。

## 结构

```
src/dxl.rs        Dynamixel Protocol 2.0（CRC/帧构造/解析，真机字节流验证）
src/port.rs       串口传输层（真实串口 + sim 实现，同一 trait）
src/arm.rs        每臂 worker：队列/轮询/状态缓存/发布/重连
src/server.rs     Unix socket / TCP RPC + 发布订阅（NDJSON），单实例锁、identity 文件
src/config.rs     配置
clients/python/   Python 客户端
docs/DESIGN.md    架构、RPC 协议全文、队列与背压设计
```
