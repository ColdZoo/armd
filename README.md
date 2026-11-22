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

## 安全层（v0.2.0，三道防线）

2026-11-22 事故：从臂肘部（motor 3）顶住机械限位持续输出力矩 → 舵机 latch `hardware error 0x80`
→ 旧版 worker 读失败/重连循环且串口 fd 未释放 → `Device or resource busy` 永久无法恢复。
v0.2.0 修复并把保护固化为三道防线（冻结规范：[`docs/SAFETY-DESIGN.md`](docs/SAFETY-DESIGN.md)，
事故记录：[`docs/INCIDENT-2026-11-22.md`](docs/INCIDENT-2026-11-22.md)）：

| 防线 | 做什么 | 默认 |
|---|---|---|
| ① 堵转保护 | 每电机 `StallTracker`：`\|goal-present\|>60 && \|velocity\|<=2 && torque_on` 持续 400ms → **只对故障电机** torque OFF + goal 回退到限位内侧（`auto_torque_off`），发 `event:"stall"`，同臂其他电机不受影响 | `stall_protection=true` |
| ② 软限位 | 写 `Goal_Position(116)` 前按 `Operating_Mode` 拦截：位置模式(3) 用 `0..4095` + EEPROM Min/Max；扩展位置模式(4，主臂) 只用 `±1,048,575` 绝对范围，不套 EEPROM 限位 | `soft_limits=true`, `soft_limit_mode="clamp"` |
| ③ 单电机故障隔离 + fd 生命周期 | 单电机 hardware error/连续无响应只记 `motor_errors`，`connected` 保持 true；`Link` open 前先 close、读失败 3 次先释放 fd 再重连 | 无需配置 |

- 夹爪夹住物体时 Goal-Present 长期有偏差属正常，shipped configs 用 `stall_exclude_motors: [6]`
  把夹爪排除出堵转自动保护（事故中的 motor 3 不排除）。
- 软限位按 `Operating_Mode` 生效：位置模式(3) 用 EEPROM Min/Max 且绝对 `0..4095`；扩展位置模式(4)
  只做 `±1,048,575` 绝对范围，**不套 EEPROM 限位**（主臂为 mode 4）。多圈关节的
  `state.joints/velocity` 可超出 `0..4095`（实测 pan 6133、roll -2889），不是坏值。
- latched `0x80` 不会被重连清除，必须 `arm.reboot` 或物理断电；`arm.clear_error` 只清 armd 侧标记。

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
armd_client safety koch_v1_1:follower        # 安全层状态（stalls/motor_errors/torque/limits）
armd_client reboot koch_v1_1:follower --motors 3   # 清 latched 0x80（Dynamixel Reboot 0x08）
armd_client torque-off koch_v1_1:follower    # 应急卸力矩
# 无硬件验证安全逻辑：sim 臂注入堵转/硬件错误/传输错误
armd_client --socket /tmp/armd-sim.sock sim-inject sim stall --motors 3
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
      "motors": [1, 2, 3, 4, 5, 6],
      "stall_protection": true,
      "stall_timeout_ms": 400,
      "stall_tolerance": 60,
      "stall_velocity_epsilon": 2,
      "auto_torque_off": true,
      "stall_recover_margin": 20,
      "stall_exclude_motors": [6],
      "soft_limits": true,
      "soft_limit_mode": "clamp",
      "soft_limit_margin": 10
    }
  ]
}
```

> 安全相关字段全部可省略（默认即安全，v0.1.0 旧配置无需改动）。完整字段见
> [`docs/api.json`](docs/api.json) 的 `config_fields` 与 [`docs/SAFETY-DESIGN.md`](docs/SAFETY-DESIGN.md)。

| 字段 | 默认 | 说明 |
|---|---|---|
| `id` | — | 对外 arm id；zeropose 用 `<robot>:<role>` |
| `port` | — | 串口设备路径（`sim: true` 时可空） |
| `baud` | 1000000 | 总线波特率 |
| `poll_hz` | 60 | 轮询频率（一次 sync_read 全部电机） |
| `read_timeout_ms` | 30 | 单次 sync_read 超时 |
| `reconnect_sec` | 2.0 | 断线重连间隔 |
| `motors` | 1..6 | 轮询的舵机 id |
| `poll_addr` / `poll_len` | 未配置且 `stall_protection=true` 时自动 128 / 8 | 轮询寄存器。默认安全配置下同时读 Present_Velocity+Present_Position（只多 4 字节/电机）；显式配置则维持旧语义（velocity 未知 → 堵转检测跳过） |
| `sim` | false | 仿真臂（`arm.sim.inject` 仅对 sim 臂可用） |
| `stall_protection` | true | 堵转/限位保护总开关（最后一道防线） |
| `stall_timeout_ms` | 400 | Goal-Present 偏差 + 静止条件持续多久触发（建议 300–500） |
| `stall_tolerance` | 60 | 判定堵转的 \|Goal-Present\| 阈值（counts） |
| `stall_velocity_epsilon` | 2 | 视为静止的 \|Present_Velocity\| 阈值（0.229rpm/单位） |
| `auto_torque_off` | true | 触发时自动对故障电机 torque OFF |
| `stall_recover_margin` | 20 | 回退目标离 EEPROM 限位内侧的余量（counts） |
| `stall_exclude_motors` | `[]`（本仓库 configs 用 `[6]`） | 不参与堵转自动保护的电机；夹爪夹住物体时 Goal-Present 长期有偏差属正常，排除避免误触发 |
| `soft_limits` | true | 写 Goal_Position(116) 时做软限位（按 Operating_Mode 生效） |
| `soft_limit_mode` | "clamp" | `clamp`（夹紧+警告）/ `reject`（返回 `soft_limit` 错误） |
| `soft_limit_margin` | 10 | 软限位内侧余量（counts） |

## RPC 速查（v0.2.0）

NDJSON over Unix socket / TCP，完整参数/返回见 [`docs/api.json`](docs/api.json) 与 [`docs/armd.html`](docs/armd.html)。

| 方法 | 说明 |
|---|---|
| `ping` / `info` / `hello` | 健康检查、守护进程信息、token 握手 |
| `arm.list` | 所有臂摘要（含 `motor_errors`/`stalls` 安全层摘要） |
| `arm.state` | 状态缓存：joints/velocity/deg/health + goals/torque/limits/motor_errors/stalls/safety |
| `arm.read` | 立刻 sync_read，返回 `positions` + 按电机 `errors` |
| `arm.read_register` / `arm.write_register` / `arm.sync_write` | 任意寄存器读写；写 Goal(116) 先过软限位（`clamped`/`warnings`，reject 模式 `soft_limit`） |
| `arm.torque` / `arm.torque_off` | 扭矩开关 / 按电机 torque OFF（应急卸力矩） |
| `arm.reboot` | Dynamixel Reboot(0x08)，清 latched 硬件错误（如 0x80），最长 5s |
| `arm.clear_error` | 清 armd 侧 `motor_errors`/`stalls` 标记 + 读 Hardware_Error_Status(70) |
| `arm.sim.inject` | 仅 sim 臂故障注入：`stall`/`hardware_error`/`no_response`/`transport_error`/`clear` |
| `arm.ping` / `arm.set_rate` / `arm.reconnect` / `arm.stats` | 单电机 ping、改轮询频率、强制重连（先关后开）、计数器（含 stalls/torque_offs/opens/closes） |
| `arm.profile.*` / `arm.calibration.snapshot` | 标定档案读取/写入/历史/恢复/删除、采点 |
| `sub.subscribe` / `sub.unsubscribe` / `sub.rate` | 状态流订阅/退订/改频率；事件 `state`/`arm`/`stall`/`motor_error`/`motor_recovered` |

错误码新增 `soft_limit`（软限位 reject）与 `not_sim`（sim.inject 用在真机臂）。

## 标定档案（profile）

标定数据（零位 / 方向 / 夹爪行程）由 armd 持有，**所有应用通过 RPC 读写**，不再各存一份：

```bash
python3 clients/python/armd_client.py --socket /tmp/armd.sock profile koch_v1_1:follower   # 读取
python3 clients/python/armd_client.py --socket /tmp/armd.sock snapshot koch_v1_1:follower  # 采点：raw(按关节名)+角度+电压温度
python3 clients/python/armd_client.py --socket /tmp/armd.sock profile koch_v1_1:follower --set profile.json
```

- 文件：`~/.config/armd/profiles/<arm-id>.json`（可在配置里用 `profile` 覆盖路径）
- 配置里 `joint_names` 声明「舵机 id → 关节名」，armd 才能校验 profile 并换算角度
- `arm.state` 会带 `deg`（标定后的关节角）和 `health`（电压/温度），其他应用直接可用
- 插上臂第一次启动时，若 armd 没有 profile，zeropose 会自动把本地旧标定迁移进来

## 开发文档（给其他 agent/应用）

- **HTML 文档（自包含，可直接浏览器打开）**：[`docs/armd.html`](docs/armd.html) —— 架构、配置、协议、全部 RPC 方法、事件、安全层、profile、示例、排障 FAQ
- **机器可读规范**：[`docs/api.json`](docs/api.json)（HTML 由它生成：`python3 scripts/gen-armd-doc.py`）
- **安全层冻结规范**：[`docs/SAFETY-DESIGN.md`](docs/SAFETY-DESIGN.md)（v0.2.0 接口/行为语义的唯一事实源）
- **事故记录**：[`docs/INCIDENT-2026-11-22.md`](docs/INCIDENT-2026-11-22.md)（fd 泄漏 / 0x80 根因与时间线）
- **本地文档站**：`armd --docs`（默认 http://127.0.0.1:8790/ ，`/api.json` 给程序读）；
  air.my 上已装 LaunchAgent `com.armd.docs`，常驻 http://127.0.0.1:8790/ ，局域网可看 `http://air.my:8790/`
- 文档里 `…`、字段含义、错误码齐全，其他 agent 基于 `docs/api.json` + `docs/armd.html` 开发即可

## 客户端

- Python（零依赖，标准库）：[`clients/python/armd_client.py`](clients/python/armd_client.py)
  —— `ArmdClient` 同步 RPC + `StateStream` 订阅最新状态；支持 `auto_reconnect=True` 断线自动重连 + 自动重订阅；
  CLI 支持 `info/arms/state/read/stats/watch/torque/write/profile/snapshot` 以及 v0.2.0 安全层
  `reboot/clear-error/torque-off/safety/sim-inject`。
- 其他语言：协议是 **NDJSON over Unix socket**，照 [`docs/DESIGN.md`](docs/DESIGN.md) 实现即可（十几行）。

## 性能（真机，air.my）

- 总线轮询 60Hz 稳定（两个臂同时在线，各 60Hz，`read_errors=0`）
- 订阅 60Hz 直通（订阅频率 ≥54Hz 时不按时间抽帧），帧间隔均值 16.7ms
- 本机 RPC 往返 <1ms；发送写命令到总线的延迟 ≤2ms（worker 每 2ms 检查一次队列）
- 对比：之前 Python 直连时 macOS 定时器合并会把 60Hz 拖到 20Hz，只能靠自旋补偿

## 排障（含 2026-11-22 事故）

### `Device or resource busy` / 串口 fd 泄漏

事故根因（v0.1.0）：读失败后旧 fd 未释放，TIOCEXCL 使再次 open 永远 EBUSY；断线后 worker 还
100% CPU 自旋。**v0.2.0 已修**：`Link` 保证 open 前先 close、读失败 3 次先释放 fd 再重连、断线不死循环。
现场处置顺序：

```bash
lsof /dev/cu.usbmodem*        # 确认没有其它进程（zeropose 直连 / 旧 armd）占用
python3 clients/python/armd_client.py --socket /tmp/armd.sock reconnect koch_v1_1:follower
# 仍 busy：插拔 USB；最后才重启 armd（不要删锁文件）
launchctl kickstart -k gui/$(id -u)/com.armd
```

fd 不变量：connected 期间 `stats.opens - stats.closes == 1`（恰好一个 live 句柄），断线释放后相等；
差值 >1 才是泄漏。可用 `arm.sim.inject kind="transport_error"` 在 sim 臂上验证释放路径。

### 舵机 latched `hardware error 0x80`（hardware_alert）

顶住限位/堵转后舵机 latch 的错误，普通重连不清除：

```bash
python3 clients/python/armd_client.py --socket /tmp/armd.sock safety koch_v1_1:follower
# motor_errors["3"].code == 128
python3 clients/python/armd_client.py --socket /tmp/armd.sock reboot koch_v1_1:follower --motors 3
# 或 arm.clear_error 只清 armd 标记；hardware_error_status 仍非 0 时必须 reboot/断电
```

### 收到 `event:"stall"` / `state.stalls`

堵转保护触发了：该电机已 torque OFF + goal 回退。先排除机械卡死/限位/负载，再给安全新目标
（写入即清除该电机 stall 标记）并 `arm.torque(enable=true)`。夹爪夹住物体时 Goal-Present 长期
有偏差属正常——shipped configs 已用 `stall_exclude_motors: [6]` 排除，不要靠关 `stall_protection` 解决。

### 单个舵机不动，其它正常

v0.2.0 起单电机故障被隔离（`connected` 仍 true）：看 `arm.list[].motor_errors` /
`arm.state.motor_errors`。`code=128` 走 `arm.reboot`；`names=["no_response"]` 检查供电/线缆/ID。
完整根因、时间线与证据：[`docs/INCIDENT-2026-11-22.md`](docs/INCIDENT-2026-11-22.md)；
接口语义：[`docs/SAFETY-DESIGN.md`](docs/SAFETY-DESIGN.md)。

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
# 1) 编译：本机交叉编译已验证（aarch64 / armv7 都产出 ELF）：
./scripts/build-arm.sh          # → dist/armd-aarch64、dist/armd-armv7
#    或者直接在板上 cargo build --release
#    CI：.github/workflows/arm.yml 打 tag 时自动产出两个架构的 artifact
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
src/config.rs     配置（含 v0.2.0 安全层字段）
src/safety.rs     安全层纯逻辑（stall 计时/错误位命名/软限位，可单测）
clients/python/   Python 客户端
docs/DESIGN.md    架构、RPC 协议全文、队列与背压、安全层/fd 生命周期/软限位
docs/SAFETY-DESIGN.md  v0.2.0 安全层冻结规范（接口/行为语义）
docs/INCIDENT-2026-11-22.md  2026-11-22 fd 泄漏/0x80 事故记录
```
