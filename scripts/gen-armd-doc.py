#!/usr/bin/env python3
"""从 docs/api.json + 内置说明生成单文件 HTML 文档（docs/armd.html）。

用法：python3 scripts/gen-armd-doc.py
没有第三方依赖；HTML 自包含（内联 CSS/JS），可直接双击或用浏览器打开。
"""
from __future__ import annotations

import html
import json
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
API = json.loads((ROOT / "docs" / "api.json").read_text(encoding="utf-8"))
OUT = ROOT / "docs" / "armd.html"
OUT_MD = ROOT / "docs" / "armd-api.md"


def esc(x) -> str:
    return html.escape(str(x))


def code(x: str, lang: str = "") -> str:
    return f'<pre class="code"><code>{esc(x)}</code></pre>'


def table(headers, rows) -> str:
    head = "".join(f"<th>{esc(h)}</th>" for h in headers)
    body = "".join("<tr>" + "".join(f"<td>{c}</td>" for c in row) + "</tr>" for row in rows)
    return f'<div class="table-wrap"><table><thead><tr>{head}</tr></thead><tbody>{body}</tbody></table></div>'


def params_table(params) -> str:
    if not params:
        return '<p class="muted">无</p>'
    rows = []
    for p in params:
        t = esc(p.get("type", "any"))
        if p.get("optional"):
            t += " · 可选"
        if "default" in p:
            t += f" · 默认 {esc(p['default'])}"
        rows.append((f"<code>{esc(p['name'])}</code>", t))
    return table(["参数", "类型"], rows)


def json_block(obj) -> str:
    return code(json.dumps(obj, ensure_ascii=False, indent=2), "json")


METHOD_ANCHORS = {m["name"]: "m-" + m["name"].replace(".", "-") for m in API["methods"]}

method_sections = []
for m in API["methods"]:
    anchor = METHOD_ANCHORS[m["name"]]
    method_sections.append(f'''
<section class="method" id="{anchor}">
  <h3><code>{esc(m["name"])}</code></h3>
  <p>{esc(m.get("desc", ""))}</p>
  <h4>参数</h4>
  {params_table(m.get("params"))}
  <h4>返回</h4>
  {json_block(m.get("result"))}
</section>''')

methods_index = table(
    ["方法", "说明"],
    [(
        f'<a href="#{METHOD_ANCHORS[m["name"]]}"><code>{esc(m["name"])}</code></a>',
        esc(m.get("desc", "")),
    ) for m in API["methods"]],
)

events_html = "".join(
    f'''<section class="event"><h3><code>{esc(e["event"])}</code></h3>
    <p>{esc(e.get("desc", ""))}</p>{json_block(e.get("fields"))}</section>'''
    for e in API["events"]
)

errors_html = table(
    ["错误码", "含义"], [(f"<code>{esc(e['code'])}</code>", esc(e["meaning"])) for e in API["errors"]]
)

config_html = table(
    ["配置项", "默认值", "说明"],
    [
        (
            f"<code>{esc(c['key'])}</code>",
            f"<code>{esc(c.get('default', '—'))}</code>",
            esc(c.get("desc", "")),
        )
        for c in API["config_fields"]
    ],
)

profile = API["profile_schema"]
profile_fields = table(
    ["字段", "含义"],
    [
        (f"<code>{esc(k)}</code>", f"<code>{esc(v)}</code>" if not isinstance(v, dict)
         else "<br>".join(f"<code>{esc(kk)}</code>：{esc(vv)}" for kk, vv in v.items()))
        for k, v in profile["fields"].items()
    ],
)

PY_EXAMPLE = '''from armd_client import ArmdClient  # clients/python/armd_client.py（stdlib）

c = ArmdClient("/tmp/armd.sock", token=None)      # 下位机：ArmdClient("tcp://arm-board:8771", token=...)
print(c.arms())                                    # 所有机械臂摘要
print(c.state("koch_v1_1:follower")["deg"])        # 标定后的关节角（度）
print(c.read("koch_v1_1:follower")["positions"])   # 立刻 sync_read 一次 raw

# 订阅 60Hz 状态流
import time
frames = []
sub = c.subscribe(lambda f: frames.append(f), arms=["koch_v1_1:follower"], rate_hz=60)
time.sleep(1)
print("frames/s ≈", len(frames))
c.unsubscribe(sub)

# 标定档案
prof = c.profile("koch_v1_1:follower")["profile"]
c.set_profile("koch_v1_1:follower", prof)          # 覆盖前 armd 自动留历史
print(c.profile_history("koch_v1_1:follower")["history"])
c.restore_profile("koch_v1_1:follower", c.profile_history("koch_v1_1:follower")["history"][-1]["name"])
c.close()'''

CURL_EXAMPLE = '''# 带 token 时先 hello；下面是 raw socket 写法（也可以用 socat/nc）
python3 - <<'PY'
import json, socket
s = socket.socket(socket.AF_UNIX); s.connect("/tmp/armd.sock")
s.sendall((json.dumps({"id":1,"method":"hello","params":{"token":"<TOKEN>"}})+"\\n").encode())
print(s.recv(4096).decode().strip())
s.sendall((json.dumps({"id":2,"method":"arm.state","params":{"arm":"koch_v1_1:follower"}})+"\\n").encode())
print(s.recv(65536).decode().strip())
PY'''

TOC = """
<ul class="toc">
  <li><a href="#overview">概览</a></li>
  <li><a href="#quickstart">快速开始</a></li>
  <li><a href="#architecture">架构与队列</a></li>
  <li><a href="#install">安装部署</a></li>
  <li><a href="#config">配置参考</a></li>
  <li><a href="#protocol">传输与协议</a></li>
  <li><a href="#methods">RPC 方法</a></li>
  <li><a href="#events">发布订阅</a></li>
  <li><a href="#profile">标定档案 profile</a></li>
  <li><a href="#clients">客户端与示例</a></li>
  <li><a href="#errors">错误码</a></li>
  <li><a href="#faq">排障 FAQ</a></li>
  <li><a href="#changelog">版本</a></li>
</ul>
"""

HTML = f'''<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>armd — 机械臂串口守护进程 · 开发文档 v{esc(API["version"])}</title>
<style>
  :root {{
    --bg: #0b1220; --panel: #111a2b; --text: #dbe6f3; --muted: #8aa0b8;
    --accent: #42f5e9; --accent2: #7dd3fc; --border: #22304a; --code-bg: #0a1020;
    --warn: #fbbf24; --ok: #34d399; --err: #f87171;
  }}
  * {{ box-sizing: border-box; }}
  html {{ scroll-behavior: smooth; }}
  body {{
    margin: 0; background: var(--bg); color: var(--text);
    font: 15px/1.65 -apple-system, BlinkMacSystemFont, "PingFang SC", "Segoe UI", "Noto Sans SC", sans-serif;
  }}
  .layout {{ display: flex; min-height: 100vh; }}
  aside {{
    width: 260px; flex: 0 0 260px; background: var(--panel); border-right: 1px solid var(--border);
    position: sticky; top: 0; height: 100vh; overflow-y: auto; padding: 22px 18px;
  }}
  aside h1 {{ font-size: 18px; margin: 0 0 4px; color: var(--accent); letter-spacing: .5px; }}
  aside .ver {{ color: var(--muted); font-size: 12px; margin-bottom: 18px; }}
  aside .search {{ width: 100%; padding: 7px 9px; margin-bottom: 14px; border-radius: 7px;
    border: 1px solid var(--border); background: var(--code-bg); color: var(--text); font-size: 13px; }}
  .toc {{ list-style: none; padding: 0; margin: 0; }}
  .toc li {{ margin: 2px 0; }}
  .toc a {{ color: var(--text); text-decoration: none; font-size: 13.5px; display: block;
    padding: 4px 8px; border-radius: 6px; }}
  .toc a:hover, .toc a.active {{ background: rgba(66,245,233,.12); color: var(--accent); }}
  main {{ flex: 1; min-width: 0; padding: 34px 44px 90px; max-width: 1080px; }}
  h1.page {{ font-size: 30px; margin: 0 0 6px; }}
  h1.page small {{ font-size: 14px; color: var(--muted); font-weight: 400; }}
  .meta {{ color: var(--muted); font-size: 13px; margin-bottom: 26px; }}
  section.block {{ margin: 0 0 40px; }}
  h2 {{ font-size: 22px; border-bottom: 1px solid var(--border); padding-bottom: 8px; margin: 36px 0 14px; }}
  h3 {{ font-size: 16px; margin: 22px 0 8px; color: var(--accent2); }}
  h4 {{ font-size: 13.5px; margin: 14px 0 6px; color: var(--muted); text-transform: uppercase; letter-spacing: .04em; }}
  p {{ margin: 8px 0; }}
  a {{ color: var(--accent2); }}
  ul, ol {{ padding-left: 22px; }}
  li {{ margin: 3px 0; }}
  code {{ background: rgba(66,245,233,.08); color: var(--accent); padding: 1px 5px; border-radius: 4px;
    font: 13px/1.5 ui-monospace, SFMono-Regular, Menlo, monospace; }}
  pre.code {{ background: var(--code-bg); border: 1px solid var(--border); border-radius: 8px;
    padding: 12px 14px; overflow-x: auto; margin: 8px 0; }}
  pre.code code {{ background: none; color: #cfe3f7; padding: 0; font-size: 12.5px; }}
  .table-wrap {{ overflow-x: auto; margin: 10px 0; }}
  table {{ border-collapse: collapse; width: 100%; font-size: 13.5px; }}
  th, td {{ border: 1px solid var(--border); padding: 7px 10px; text-align: left; vertical-align: top; }}
  th {{ background: rgba(125,211,252,.08); color: var(--accent2); font-weight: 600; }}
  tr:nth-child(even) td {{ background: rgba(255,255,255,.015); }}
  .muted {{ color: var(--muted); }}
  .note, .warn {{ border-left: 3px solid var(--accent); background: rgba(66,245,233,.06);
    padding: 10px 14px; border-radius: 0 8px 8px 0; margin: 12px 0; }}
  .warn {{ border-left-color: var(--warn); background: rgba(251,191,36,.07); }}
  .grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(230px, 1fr)); gap: 12px; }}
  .card {{ background: var(--panel); border: 1px solid var(--border); border-radius: 10px; padding: 12px 14px; }}
  .card b {{ color: var(--accent); }}
  .method {{ background: var(--panel); border: 1px solid var(--border); border-radius: 10px;
    padding: 8px 16px 14px; margin: 14px 0; }}
  .method h3 {{ margin-top: 10px; }}
  .ascii {{ background: var(--code-bg); border: 1px solid var(--border); border-radius: 8px;
    padding: 12px 14px; overflow-x: auto; font: 12px/1.45 ui-monospace, Menlo, monospace;
    color: #b9d4ee; white-space: pre; }}
  footer {{ color: var(--muted); font-size: 12.5px; border-top: 1px solid var(--border);
    margin-top: 40px; padding-top: 14px; }}
  @media (max-width: 900px) {{
    .layout {{ display: block; }}
    aside {{ position: static; width: auto; height: auto; border-right: 0; border-bottom: 1px solid var(--border); }}
    main {{ padding: 22px 18px 60px; }}
  }}
  @media print {{
    aside {{ display: none; }} body {{ background: #fff; color: #111; }}
    main {{ max-width: none; }} pre.code {{ background: #f5f7fa; }} code {{ color: #0b63b3; }}
  }}
</style>
</head>
<body>
<div class="layout">
<aside>
  <h1>armd</h1>
  <div class="ver">v{esc(API["version"])} · 更新于 {esc(API["updated"])}<br>机械臂串口守护进程</div>
  <input class="search" id="search" placeholder="搜索方法/配置/错误码…" autocomplete="off">
  {TOC}
</aside>
<main>
  <h1 class="page">armd — 机械臂串口守护进程 <small>开发文档</small></h1>
  <div class="meta">{esc(API["summary"])}</div>

  <section class="block" id="overview">
    <h2>概览</h2>
    <p><b>armd</b> 是机械臂的<b>用户态设备驱动</b>：系统里唯一打开 Dynamixel 串口的进程。
    它按固定节拍（默认 60Hz）轮询每条总线上的所有舵机，把状态分发给所有订阅者，
    并通过 RPC 接受读/写/标定操作。上层应用（ZeroPose、遥操作、脚本、下位机上的其他服务）
    <b>从不直接碰串口</b>，只连 armd。</p>
    <div class="grid">
      <div class="card"><b>单进程多臂</b><br>一条总线一个 worker 线程；主臂/从臂可同时在线</div>
      <div class="card"><b>RPC + 发布订阅</b><br>Unix socket / TCP 上的 NDJSON；任意语言可接</div>
      <div class="card"><b>标定档案</b><br>zero/sign/grip 由 armd 持有，自动历史快照可恢复</div>
      <div class="card"><b>ARM 下位机</b><br>交叉编译 aarch64/armv7，systemd + TCP + token</div>
    </div>
    <div class="note">快速判断"该不该用 armd"：只要你的程序需要读/写机械臂舵机，就应该连 armd；
    需要标定后的关节角，直接读 <code>arm.state.deg</code>，不用自己实现换算。</div>
  </section>

  <section class="block" id="quickstart">
    <h2>快速开始</h2>
    <h3>1. 启动守护进程（air.my 已用 LaunchAgent 常驻）</h3>
    {code('''# 真机（配置见 ~/.config/armd/config.json）
~/.local/bin/armd --config ~/.config/armd/config.json
# 无硬件自测
armd --sim --socket /tmp/armd-sim.sock

# LaunchAgent 管理（air.my）
launchctl kickstart -k gui/$(id -u)/com.armd   # 重启
launchctl print    gui/$(id -u)/com.armd       # 状态
tail -f /tmp/armd.log                          # 日志''')}
    <h3>2. 用 Python 客户端调一下</h3>
    {code('''cd ~/nfs/raid/Projects/armd
python3 clients/python/armd_client.py --socket /tmp/armd.sock info
python3 clients/python/armd_client.py --socket /tmp/armd.sock arms
python3 clients/python/armd_client.py --socket /tmp/armd.sock state koch_v1_1:follower''')}
    <h3>3. 网络/下位机场景</h3>
    {code('''armd --config /etc/armd/config.json       # 板上：listen 含 tcp://0.0.0.0:8771 + token
python3 clients/python/armd_client.py --socket tcp://arm-board.my:8771 --token $ARMD_TOKEN arms''')}
  </section>

  <section class="block" id="architecture">
    <h2>架构与队列</h2>
    <div class="ascii">客户端 A（ZeroPose）   客户端 B（遥操作）   客户端 C（脚本/agent）
        │ RPC/订阅             │                     │
        └──────────────┬───────┴─────────────────────┘
                       ▼  Unix socket 或 TCP（NDJSON）
              ┌───────────────────────────┐
              │           armd            │
              │  conn 线程 × N（每客户端） │
              │  writer 线程（响应+推送）  │
              ├───────────────────────────┤
              │  arm worker（每条总线一个）│
              │   urgent 队列 64          │
              │   normal 队列 256         │
              │   每订阅者有界队列 32（丢帧）│
              │   60Hz sync_read 合并       │
              └──────────┬────────────────┘
                         ▼
              /dev/cu.usbmodem*（Dynamixel 总线，1Mbps）</div>
    <ul>
      <li><b>urgent(64)</b>：写寄存器 / 扭矩 / sync_write / 重连，每轮优先全清，写命令到总线延迟 ≤2ms。</li>
      <li><b>normal(256)</b>：读快照、ping、改速率等，每轮最多 16 条，保证轮询不被饿死。</li>
      <li><b>轮询合并</b>：一次 <code>sync_read</code> 读全部电机，结果写状态缓存；所有订阅者共享这次读数。</li>
      <li><b>发布背压</b>：每个订阅者一个容量 32 的有界队列，满了丢帧并计数（<code>frames_dropped</code>），慢客户端不拖累总线。</li>
      <li><b>订阅速率</b>：<code>rate_hz ≥ 54</code> 时直通（每个轮询帧都推）；更低频率按时间抽帧。</li>
      <li><b>自愈</b>：连续 3 次读失败判掉线 → 每 <code>reconnect_sec</code>（默认 2s）重连；连接时 ping 全部电机握手。</li>
      <li><b>单实例</b>：打开串口前先拿 <code>&lt;socket&gt;.lock</code>（File::try_lock）；启动时探测已有 socket，活的拒绝抢占、死的清理。</li>
    </ul>
  </section>

  <section class="block" id="install">
    <h2>安装部署</h2>
    <h3>macOS（air.my）</h3>
    {code('''./scripts/deploy-macos.sh     # 构建 → 安装 → codesign → 重启 LaunchAgent
# macOS 26 坑：拷贝过的二进制会被 taskgated 以 "Code Signature Invalid" SIGKILL，
# 脚本里的 codesign --force --sign - 是必须的。''')}
    <h3>ARM 下位机（Linux）</h3>
    {code('''# 交叉编译（macOS 上已装 zig + cargo-zigbuild + rustup targets）
./scripts/build-arm.sh        # → dist/armd-aarch64, dist/armd-armv7
# 板上安装
sudo install -d /opt/armd /etc/armd
sudo install -m755 armd /opt/armd/armd
sudo install -m600 configs/koch-arm-board.json /etc/armd/config.json
sudo install -m644 deploy/systemd/armd.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now armd
sudo usermod -aG dialout armd
sudo systemctl mask serial-getty@ttyS2   # Armbian 默认串口控制台会占用 UART
sudo cat /run/armd/identity.json''')}
    <div class="warn">只有板上的 armd 能碰串口：下位机上线后，air.my 上不要再起 armd（或清空其 arms）。
    网络场景必须配 <code>token</code>；不要裸奔在局域网。</div>
  </section>

  <section class="block" id="config">
    <h2>配置参考（JSON）</h2>
    {code('''{
  "socket": "/tmp/armd.sock",
  "listen": ["unix:/tmp/armd.sock", "tcp://0.0.0.0:8771"],
  "token": "CHANGE_ME",
  "identity": "/run/armd/identity.json",
  "arms": [
    {
      "id": "koch_v1_1:follower",
      "label": "Koch v1.1 从臂",
      "port": "/dev/ttyACM0",
      "baud": 1000000,
      "poll_hz": 60,
      "motors": [1, 2, 3, 4, 5, 6],
      "monitor": true,
      "joint_names": {"1": "shoulder_pan", "2": "shoulder_lift", "3": "elbow_flex",
                       "4": "wrist_flex", "5": "wrist_roll", "6": "gripper"},
      "profile": "~/.config/armd/profiles/koch_v1_1-follower.json"
    }
  ]
}''')}
    {config_html}
  </section>

  <section class="block" id="protocol">
    <h2>传输与协议</h2>
    <p><b>帧格式</b>：{esc(API["transport"]["framing"])}</p>
    <p><b>Unix socket</b>：{esc(API["transport"]["unix"])}<br>
       <b>TCP</b>：{esc(API["transport"]["tcp"])}<br>
       <b>鉴权</b>：{esc(API["transport"]["auth"])}<br>
       <b>JSON-RPC 2.0</b>：{esc(API["transport"]["jsonrpc"])}</p>
    <h3>请求 / 响应 / 错误</h3>
    {code('''→ ''' + json.dumps(API["request_example"], ensure_ascii=False))}
    {code('''← ''' + json.dumps(API["response_example"], ensure_ascii=False))}
    {code('''← ''' + json.dumps(API["error_example"], ensure_ascii=False))}
    <p class="muted">请求不带 id 也可（单向命令），但写操作建议带 id 等确认。</p>
  </section>

  <section class="block" id="methods">
    <h2>RPC 方法</h2>
    {methods_index}
    {''.join(method_sections)}
  </section>

  <section class="block" id="events">
    <h2>发布订阅（事件帧）</h2>
    <p>订阅后服务端按 <code>rate_hz</code> 推送；帧里 <code>sub</code> 是订阅 id。
    <code>event:"state"</code> 为周期状态，<code>event:"arm"</code> 为连接状态变化。</p>
    {events_html}
  </section>

  <section class="block" id="profile">
    <h2>标定档案 profile</h2>
    <p>{esc(profile["description"])}</p>
    {profile_fields}
    <h3>换算公式</h3>
    {code('''# 非夹爪关节
deg = wrap(raw - zero)   # 折到 ±2048（连续关节可整圈，避免 ±180° 跳变）
deg = deg * 360 / 4096 * sign
# 夹爪
deg = clamp((raw - closed) / (open - closed), 0, 1) * (angle 或 137.5)''')}
    <h3>读写 / 历史 / 恢复</h3>
    {code('''c.profile(arm) -> {profile, path, has_profile}
c.set_profile(arm, profile)     # 校验 + 原子写；覆盖前自动快照到 profiles/history/（保留 20 份）
c.profile_history(arm)          # [{name, savedAt}]
c.restore_profile(arm, name)    # 从历史快照恢复
c.calibration_snapshot(arm)     # 采点：raw(按关节名)+deg+health，不占总线''')}
    <div class="note">armd 是标定数据的<b>唯一事实源</b>。ZeroPose 的「保存到真机」通过
    <code>arm.profile.set</code> 写入；本地 <code>~/.config/zeropose/*.json</code> 只是镜像备份。</div>
  </section>

  <section class="block" id="clients">
    <h2>客户端与示例</h2>
    <p><b>Python 客户端</b>：<code>clients/python/armd_client.py</code>（纯标准库，支持 Unix/TCP/token）。</p>
    {code(PY_EXAMPLE)}
    <h3>裸 socket（任意语言）</h3>
    {code(CURL_EXAMPLE)}
    <h3>CLI</h3>
    {code('''python3 clients/python/armd_client.py --socket /tmp/armd.sock info
python3 clients/python/armd_client.py --socket /tmp/armd.sock arms
python3 clients/python/armd_client.py --socket /tmp/armd.sock state   koch_v1_1:follower
python3 clients/python/armd_client.py --socket /tmp/armd.sock read    koch_v1_1:follower
python3 clients/python/armd_client.py --socket /tmp/armd.sock stats   koch_v1_1:follower
python3 clients/python/armd_client.py --socket /tmp/armd.sock watch   koch_v1_1:follower --rate 10
python3 clients/python/armd_client.py --socket /tmp/armd.sock torque  koch_v1_1:follower 0
python3 clients/python/armd_client.py --socket /tmp/armd.sock profile koch_v1_1:follower
python3 clients/python/armd_client.py --socket /tmp/armd.sock profile koch_v1_1:follower --history
python3 clients/python/armd_client.py --socket /tmp/armd.sock snapshot koch_v1_1:follower''')}
  </section>

  <section class="block" id="errors">
    <h2>错误码</h2>
    {errors_html}
  </section>

  <section class="block" id="faq">
    <h2>排障 FAQ</h2>
    <h3>启动失败：另一个 armd 已经持有 …lock</h3>
    <p>单实例锁 <code>&lt;socket&gt;.lock</code> 被占用。先确认没有在跑的 armd（<code>pgrep -fl armd</code>）；
    锁文件本身不需要删除（进程退出内核自动释放）。</p>
    <h3>macOS：进程刚起就被 SIGKILL（Code Signature Invalid）</h3>
    <p>拷贝过的二进制需要重签：<code>codesign --force --sign - ~/.local/bin/armd</code>；用 <code>scripts/deploy-macos.sh</code> 可避免。</p>
    <h3>Linux 板子：舵机全都不响应</h3>
    <p>99% 是串口被别的程序占用：<code>lsof /dev/ttyACM0</code> 或 <code>fuser -v /dev/ttyACM0</code>；
    Armbian 常见 <code>serial-getty@ttyS2</code>，<code>sudo systemctl mask serial-getty@ttyS2</code>。</p>
    <h3>连接被拒 / 全部 unauthorized</h3>
    <p>配置了 <code>token</code>：客户端连接后必须先发 <code>hello</code>（Python 客户端传 <code>token=</code> 或环境变量 <code>ARMD_TOKEN</code> 会自动发）。</p>
    <h3>订阅帧数比 60 少</h3>
    <p><code>rate_hz &lt; 54</code> 会按时间抽帧；要 60Hz 直通就传 <code>rate_hz: 60</code>。
    也可能是客户端读得慢：看 <code>arm.stats.frames_dropped</code>。</p>
    <h3>网页/应用一直显示旧数据</h3>
    <p>检查订阅连接是否中断：状态帧 <code>ts</code> 超过 1 秒没更新应视为掉线并重连（ZeroPose 已内置）。
    armd 重启后客户端需要重新 subscribe。</p>
    <h3>谁在跑？什么版本？监听在哪？</h3>
    <p><code>cat /tmp/armd-identity.json</code>（Linux 下是 <code>/run/armd/identity.json</code>）。</p>
  </section>

  <section class="block" id="changelog">
    <h2>版本与兼容</h2>
    <ul>
      <li><b>v{esc(API["version"])}</b>（{esc(API["updated"])}）：JSON-RPC 2.0 兼容、token 鉴权、Unix+TCP 双传输、
      标定档案 profile + 历史快照/恢复、电压/温度监控、ARM 交叉编译产物。</li>
      <li>协议稳定性：方法名/字段只增不改；新增可选字段老客户端可忽略。</li>
      <li>机器可读规范：<code>docs/api.json</code>（本页由它生成：<code>python3 scripts/gen-armd-doc.py</code>）。</li>
    </ul>
    <footer>armd v{esc(API["version"])} · 生成于 {time.strftime("%Y-%m-%d %H:%M")} ·
    仓库 Gogs <code>adam/armd</code> / GitHub <code>ColdZoo/armd</code></footer>
  </section>
</main>
</div>
<script type="application/json" id="armd-api">{json.dumps(API, ensure_ascii=False)}</script>
<script>
  // 侧边栏 TOC 高亮
  const links = [...document.querySelectorAll('.toc a')];
  const targets = links.map(a => document.querySelector(a.getAttribute('href'))).filter(Boolean);
  const io = new IntersectionObserver((entries) => {{
    entries.forEach(en => {{
      if (en.isIntersecting) {{
        links.forEach(l => l.classList.remove('active'));
        const link = links.find(l => l.getAttribute('href') === '#' + en.target.id);
        if (link) link.classList.add('active');
      }}
    }});
  }}, {{ rootMargin: '-20% 0px -70% 0px' }});
  targets.forEach(t => io.observe(t));

  // 搜索：过滤方法/事件/错误码/配置表行，并隐藏无匹配的区块标题
  const search = document.getElementById('search');
  search.addEventListener('input', () => {{
    const q = search.value.trim().toLowerCase();
    document.querySelectorAll('section.method, section.event, tbody tr').forEach(el => {{
      el.style.display = !q || el.textContent.toLowerCase().includes(q) ? '' : 'none';
    }});
    document.querySelectorAll('section.method').forEach(el => {{
      const h3 = el.querySelector('h3');
      if (h3 && el.style.display === 'none') h3.dataset.hidden = '1';
    }});
  }});
</script>
</body>
</html>
'''

def render_markdown() -> str:
    """KB 友好版（纯 Markdown）：OpenViking 语义检索/其他 agent 直接读。"""
    lines = [
        f"# armd API 参考（v{API['version']}，更新 {API['updated']}）",
        "",
        API["summary"],
        "",
        "> 完整 HTML 文档（含架构/安装/排障/示例）：`viking://resources/armd/armd.html`；",
        "> 本机文档站 http://127.0.0.1:8790/ ｜局域网 http://air.my:8790/ ｜机器可读规范 `viking://resources/armd/api.json`",
        "",
        "## 传输",
        "",
        f"- Unix socket：{API['transport']['unix']}",
        f"- TCP：{API['transport']['tcp']}",
        f"- 帧格式：{API['transport']['framing']}",
        f"- 鉴权：{API['transport']['auth']}",
        f"- JSON-RPC：{API['transport']['jsonrpc']}",
        "",
        "## 请求 / 响应示例",
        "",
        "```json",
        json.dumps(API["request_example"], ensure_ascii=False),
        "```",
        "",
        "```json",
        json.dumps(API["response_example"], ensure_ascii=False),
        "```",
        "",
        "错误响应：",
        "",
        "```json",
        json.dumps(API["error_example"], ensure_ascii=False),
        "```",
        "",
        "## 方法总览",
        "",
        "| 方法 | 说明 |",
        "|---|---|",
    ]
    for m in API["methods"]:
        lines.append(f"| `{m['name']}` | {m.get('desc', '')} |")
    lines += ["", "## 方法详情", ""]
    for m in API["methods"]:
        lines += [f"### `{m['name']}`", "", m.get("desc", ""), ""]
        params = m.get("params") or []
        if params:
            lines += ["| 参数 | 类型 |", "|---|---|"]
            for p in params:
                t = p.get("type", "any")
                if p.get("optional"):
                    t += "（可选）"
                if "default" in p:
                    t += f"，默认 {p['default']}"
                lines.append(f"| `{p['name']}` | {t} |")
            lines.append("")
        lines += ["返回：", "", "```json", json.dumps(m.get("result"), ensure_ascii=False, indent=2), "```", ""]
    lines += ["## 发布订阅事件", ""]
    for e in API["events"]:
        lines += [f"### `{e['event']}`", "", e.get("desc", ""), "", "```json",
                  json.dumps(e.get("fields"), ensure_ascii=False, indent=2), "```", ""]
    lines += ["## 错误码", "", "| 错误码 | 含义 |", "|---|---|"]
    for e in API["errors"]:
        lines.append(f"| `{e['code']}` | {e['meaning']} |")
    prof = API["profile_schema"]
    lines += ["", "## 标定档案 profile", "", prof["description"], "", "| 字段 | 含义 |", "|---|---|"]
    for k, v in prof["fields"].items():
        if isinstance(v, dict):
            v = "；".join(f"{kk}: {vv}" for kk, vv in v.items())
        lines.append(f"| `{k}` | {v} |")
    lines += ["", "换算公式：", "", "```", prof["formula"], "```", ""]
    lines += ["## 配置项", "", "| 配置项 | 默认值 | 说明 |", "|---|---|---|"]
    for c in API["config_fields"]:
        lines.append(f"| `{c['key']}` | {c.get('default', '—')} | {c.get('desc', '')} |")
    lines += ["", f"仓库：{API['packages']['repo']}；客户端：{API['packages']['python_client']}", ""]
    return "\n".join(lines)


if __name__ == "__main__":
    OUT.write_text(HTML, encoding="utf-8")
    md = render_markdown()
    OUT_MD.write_text(md, encoding="utf-8")
    print(f"wrote {OUT} ({len(HTML.encode())} bytes)")
    print(f"wrote {OUT_MD} ({len(md.encode())} bytes, KB 友好版)")
