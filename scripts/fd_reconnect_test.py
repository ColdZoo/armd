#!/usr/bin/env python3
"""armd 串口 fd / 重连耐久测试（Python 标准库，无第三方依赖）。

背景（见 docs/SAFETY-DESIGN.md §2 根因 C）：
    串口是独占资源（TIOCEXCL）。v0.1.0 的 reopen 失败路径没释放旧 fd，导致
    "自己占着 fd，又永远 open 不了"（lsof 看到 fd 挂在已消失的设备节点上，
    open 永远 `Device or resource busy`）。本脚本对**运行中的 armd** 做 N 次
    `arm.reconnect`，每次前后用 lsof 统计 armd 进程里每个机械臂串口设备的 fd 数量，
    断言：连着的臂每设备恒为 **1** 个 fd；断开的臂为 0（没有遗留 fd）；
    RPC 不能出现 `Device or resource busy`。

安全性：
    - 只调用只读/运维 RPC：`info` / `arm.list` / `arm.stats` / `arm.reconnect`，
      **绝不写电机寄存器**（脚本内部用 ALLOWED_METHODS 白名单强制）；
    - 不改配置、不重启 daemon、不动 bit / 其它进程；pid 只用于 `lsof -p`。

注意：物理拔插 USB 串口是另一套**人工**流程（本脚本不模拟）：
    1) 保持订阅不断开，手动拔掉 follower 串口线，观察 armd 日志出现 reconnect 循环；
    2) `lsof -p <pid> | grep usbmodem` 应只剩 0 个 fd（旧 handle 已释放）；
    3) 插回后 arm.list 自动 connected=true 且每设备 1 个 fd；
    4) 若设备节点换名（macOS 常见），需改 configs 并重启 armd —— 这属于人工步骤。

用法：
    python3 scripts/fd_reconnect_test.py                      # 默认 /tmp/armd.sock，20 次
    python3 scripts/fd_reconnect_test.py --cycles 20 --interval 0.2
    python3 scripts/fd_reconnect_test.py --socket /run/armd.sock --pid 1234
    python3 scripts/fd_reconnect_test.py --arm koch_v1_1:follower --verbose

退出码：
    0  通过（fd 不泄漏、无 busy、无 RPC 错误）
    1  发现 fd 泄漏 / busy / RPC 错误 / 断言失败
    2  用法或环境错误（socket 连不上、找不到 armd pid、没有 lsof 等）
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import socket
import subprocess
import sys
import time
from typing import Any, Dict, List, Optional, Tuple

DEFAULT_SOCKET = "/tmp/armd.sock"
#: armd 默认 identity 文件（Linux 优先 /run/armd，macOS 退回 /tmp）
IDENTITY_CANDIDATES = ("/run/armd/identity.json", "/tmp/armd-identity.json")

#: 本脚本只允许这些 RPC（安全白名单：绝不做电机写操作）
ALLOWED_METHODS = frozenset({"info", "arm.list", "arm.stats", "arm.reconnect"})


class SetupError(Exception):
    """环境/用法问题（退出码 2）。"""


class RpcError(Exception):
    """armd 返回的 JSON-RPC 错误（退出码 1）。"""

    def __init__(self, code: str, message: str):
        super().__init__(f"{code}: {message}")
        self.code = code
        self.message = message


# ---------------------------------------------------------------- RPC 客户端


class RpcClient:
    """极简 NDJSON 客户端：一行一个 JSON，按 id 配对，推送帧（带 event）自动跳过。"""

    def __init__(self, path: str, timeout: float = 5.0):
        self.path = path
        self.timeout = float(timeout)
        self._sock: Optional[socket.socket] = None
        self._buf = b""
        self._next_id = 0

    def connect(self) -> "RpcClient":
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.settimeout(self.timeout)
        try:
            sock.connect(self.path)
        except OSError as exc:
            sock.close()
            raise SetupError(f"连接 armd socket 失败 {self.path}: {exc}") from exc
        self._sock = sock
        return self

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None
                self._buf = b""

    def request(self, method: str, params: Optional[Dict[str, Any]] = None) -> Any:
        if method not in ALLOWED_METHODS:
            raise SetupError(f"脚本安全白名单禁止调用 {method}（只读运维 RPC）")
        if self._sock is None:
            raise SetupError("RPC 连接已关闭")
        self._next_id += 1
        request_id = self._next_id
        payload = json.dumps(
            {"id": request_id, "method": method, "params": params or {}},
            ensure_ascii=False,
        )
        try:
            self._sock.sendall(payload.encode("utf-8") + b"\n")
        except OSError as exc:
            raise RpcError("closed", f"写 {method} 失败: {exc}") from exc

        deadline = time.monotonic() + self.timeout
        while True:
            line = self._readline(deadline)
            if line is None:
                raise RpcError("timeout", f"{method} 未在 {self.timeout:.1f}s 内返回")
            try:
                frame = json.loads(line)
            except json.JSONDecodeError:
                continue
            if not isinstance(frame, dict):
                continue
            if frame.get("id") != request_id:
                continue  # 订阅推送 / 其它帧
            error = frame.get("error")
            if isinstance(error, dict):
                raise RpcError(str(error.get("code", "?")), str(error.get("message", "")))
            return frame.get("result")

    def _readline(self, deadline: float) -> Optional[str]:
        assert self._sock is not None
        while b"\n" not in self._buf:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            self._sock.settimeout(min(0.5, max(0.01, remaining)))
            try:
                chunk = self._sock.recv(65536)
            except socket.timeout:
                continue
            except OSError as exc:
                raise RpcError("closed", f"读 socket 失败: {exc}") from exc
            if not chunk:
                raise RpcError("closed", "armd 关闭了连接")
            self._buf += chunk
        line, _sep, self._buf = self._buf.partition(b"\n")
        return line.decode("utf-8", "replace").strip()


# ---------------------------------------------------------------- 进程 / lsof


def _run(cmd: List[str], timeout: float = 5.0) -> subprocess.CompletedProcess:
    try:
        return subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
            check=False,
        )
    except FileNotFoundError as exc:
        raise SetupError(f"命令不存在: {cmd[0]}") from exc
    except subprocess.TimeoutExpired as exc:
        raise SetupError(f"命令超时: {' '.join(cmd)}") from exc


def read_identity(path: str) -> Optional[Dict[str, Any]]:
    try:
        with open(path, "r", encoding="utf-8") as fh:
            data = json.load(fh)
    except (OSError, json.JSONDecodeError):
        return None
    return data if isinstance(data, dict) else None


def pid_alive(pid: int) -> bool:
    if pid <= 0:
        return False
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def pid_from_lsof(lsof: str, socket_path: str) -> Optional[int]:
    proc = _run([lsof, "-t", "--", socket_path], timeout=5.0)
    if proc.returncode not in (0, 1):  # 1 = 没有匹配，正常
        raise SetupError(f"lsof 查询 socket pid 失败: {proc.stderr.strip()}")
    candidates: List[int] = []
    for line in proc.stdout.splitlines():
        line = line.strip()
        if line.isdigit():
            candidates.append(int(line))
    matching: List[int] = []
    for pid in candidates:
        ps = _run(["ps", "-p", str(pid), "-o", "comm="], timeout=5.0)
        command = (ps.stdout or "").strip()
        if "armd" in os.path.basename(command) or command.endswith("armd"):
            matching.append(pid)
    unique = sorted(set(matching))
    if len(unique) == 1:
        return unique[0]
    if len(unique) > 1:
        raise SetupError(
            f"lsof 找到多个疑似 armd 进程持有 {socket_path}: {unique}；请用 --pid 指定"
        )
    return None


def discover_pid(args: argparse.Namespace, lsof: Optional[str]) -> Tuple[int, str]:
    """定位 armd 进程 pid：--pid > identity 文件 > lsof socket。"""
    if args.pid:
        pid = int(args.pid)
        if not pid_alive(pid):
            raise SetupError(f"--pid {pid} 进程不存在（或无权查看）")
        return pid, "--pid"
    candidates: List[str] = []
    if args.identity:
        candidates.append(args.identity)
    else:
        candidates.extend(IDENTITY_CANDIDATES)
    for path in candidates:
        data = read_identity(path)
        if not data or data.get("service") != "armd":
            continue
        listen = data.get("listen") or []
        socket_listens = {str(item) for item in listen} | {f"unix:{args.socket}"}
        if args.socket not in socket_listens:
            # identity 描述的可能是别的 daemon（比如生产 /tmp/armd.sock）
            continue
        pid = int(data.get("pid") or 0)
        if pid_alive(pid):
            return pid, path
    if lsof:
        pid = pid_from_lsof(lsof, args.socket)
        if pid:
            return pid, f"lsof ({args.socket})"
    raise SetupError(
        "找不到 armd 进程 pid（identity 文件不匹配或不存在）。"
        "请用 --pid <pid> / --identity <path> 指定"
    )


def lsof_fd_rows(pid: int, lsof: str) -> List[Dict[str, str]]:
    """解析 `lsof -nP -p <pid>`：返回 [{fd, type, name}]。"""
    proc = _run([lsof, "-n", "-P", "-p", str(pid)], timeout=10.0)
    if proc.returncode not in (0, 1):
        raise SetupError(f"lsof -p {pid} 失败: {proc.stderr.strip()}")
    rows: List[Dict[str, str]] = []
    for line in proc.stdout.splitlines()[1:]:
        parts = line.split()
        if len(parts) < 9:
            continue
        # COMMAND PID USER FD TYPE DEVICE SIZE/OFF NODE NAME(params 可能带空格)
        rows.append(
            {
                "command": parts[0],
                "pid": parts[1],
                "fd": parts[3],
                "type": parts[4],
                "name": " ".join(parts[8:]).strip(),
            }
        )
    return rows


def _name_matches_port(name: str, port: str) -> bool:
    name = name.strip()
    if name == port:
        return True
    # lsof 有时显示 "port->..."、"(deleted)" 等修饰
    return name.startswith(port + " ") or name.startswith(port + "->") or name.startswith(port + "(deleted)")


def count_serial_fds(rows: List[Dict[str, str]], port: str) -> List[str]:
    """返回该串口设备上的 fd 描述列表（fd 号+name）。"""
    matches = [f"{row['fd']}:{row['name']}" for row in rows if _name_matches_port(row["name"], port)]
    return matches


def unassigned_serial_fds(rows: List[Dict[str, str]], ports: List[str]) -> List[str]:
    """armd 进程里持有、但不属于任何已配置臂串口的设备 fd（可能是泄漏/未知设备）。"""
    known = [p for p in ports if p]
    out: List[str] = []
    for row in rows:
        name = row["name"].strip()
        if row["type"] != "CHR":
            continue
        if not (name.startswith("/dev/cu.") or name.startswith("/dev/ttyUSB") or name.startswith("/dev/ttyACM")):
            continue
        if any(_name_matches_port(name, port) for port in known):
            continue
        out.append(f"{row['fd']}:{name}")
    return out


# ---------------------------------------------------------------- arm 状态辅助


def get_arms(client: RpcClient) -> List[Dict[str, Any]]:
    arms = client.request("arm.list")
    if not isinstance(arms, list):
        raise RpcError("bad_result", f"arm.list 应返回数组，实际: {arms!r}")
    return [arm for arm in arms if isinstance(arm, dict) and arm.get("id")]


def wait_all_connected(
    client: RpcClient, arm_ids: List[str], timeout: float
) -> Tuple[Dict[str, Dict[str, Any]], List[str]]:
    """轮询 arm.list，直到所有目标臂 connected；返回 (最新臂表, 仍未连上的 id)。"""
    deadline = time.monotonic() + timeout
    latest: Dict[str, Dict[str, Any]] = {}
    while True:
        arms = get_arms(client)
        latest = {str(arm.get("id")): arm for arm in arms}
        missing = [aid for aid in arm_ids if not latest.get(aid, {}).get("connected")]
        if not missing or time.monotonic() >= deadline:
            return latest, missing
        time.sleep(0.05)


def select_target_arms(arms: List[Dict[str, Any]], wanted: List[str]) -> List[Dict[str, Any]]:
    if wanted:
        by_id = {str(arm.get("id")): arm for arm in arms}
        missing = [aid for aid in wanted if aid not in by_id]
        if missing:
            raise SetupError(f"--arm 指定的臂不存在: {missing}；可用: {sorted(by_id)}")
        return [by_id[aid] for aid in wanted]
    serial_arms = [arm for arm in arms if arm.get("port")]
    # 没有真机臂（比如 --sim 测试 daemon）时也 reconnect 一遍 sim 臂，验证 RPC 链路
    return serial_arms if serial_arms else list(arms)


def stats_delta(before: Dict[str, Any], after: Dict[str, Any], key: str) -> int:
    try:
        return int(after.get(key) or 0) - int(before.get(key) or 0)
    except (TypeError, ValueError):
        return 0


# ---------------------------------------------------------------- 主流程


def parse_args(argv: Optional[List[str]]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="armd 串口 fd 泄漏 / 重连耐久测试（只读 RPC；不写电机）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("注意：物理拔插")[0],
    )
    parser.add_argument("--socket", default=DEFAULT_SOCKET, help=f"armd Unix socket（默认 {DEFAULT_SOCKET}）")
    parser.add_argument("--cycles", type=int, default=20, help="arm.reconnect 次数（默认 20）")
    parser.add_argument("--interval", type=float, default=0.2, help="每次 reconnect 后等待秒数（默认 0.2）")
    parser.add_argument("--arm", action="append", default=[], metavar="ARM_ID", help="只测试指定臂（可重复；默认自动选真机臂）")
    parser.add_argument("--pid", type=int, default=None, help="armd 进程 pid（跳过 identity/lsof 探测）")
    parser.add_argument("--identity", default=None, help="armd identity JSON 路径（默认 /run/armd 或 /tmp）")
    parser.add_argument("--timeout", type=float, default=5.0, help="单次 RPC / 重连收敛超时秒数（默认 5）")
    parser.add_argument("--settle-timeout", type=float, default=None, help="每次 reconnect 后等待 connected 的秒数（默认 = --timeout）")
    fd_group = parser.add_mutually_exclusive_group()
    fd_group.add_argument("--lsof", action="store_true", help="强制用 lsof 做 fd 校验（默认自动探测 lsof）")
    fd_group.add_argument("--no-lsof", action="store_true", help="跳过 lsof fd 校验（只跑 RPC 重连）")
    parser.add_argument("--lsof-path", default=os.environ.get("LSOF", "lsof"), help="lsof 可执行文件路径（默认环境变量 LSOF 或 lsof）")
    parser.add_argument("--verbose", action="store_true", help="打印每次 reconnect 的耗时与 fd 明细")
    args = parser.parse_args(argv)
    if args.cycles < 1:
        parser.error("--cycles 必须 >= 1")
    if args.interval < 0:
        parser.error("--interval 不能为负")
    if args.settle_timeout is None:
        args.settle_timeout = args.timeout
    return args


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)

    # ---- lsof 可用性
    lsof_path: Optional[str] = None
    if args.no_lsof:
        print("[fd] 已按 --no-lsof 跳过 lsof fd 校验（只验证重连 RPC/状态流）")
    else:
        found = shutil.which(args.lsof_path)
        if found:
            lsof_path = found
        elif args.lsof:
            print(f"错误: --lsof 要求可用的 lsof（找不到 {args.lsof_path!r}）", file=sys.stderr)
            return 2
        else:
            print("[fd] 警告: 没找到 lsof，跳过 fd 校验（可用 --lsof-path 指定）", file=sys.stderr)

    client = RpcClient(args.socket, timeout=args.timeout)
    failures: List[str] = []
    try:
        client.connect()
        info = client.request("info")
        print(f"[fd] armd {info.get('version','?')} uptime={info.get('uptime_sec')}s socket={info.get('socket')}")

        arms = get_arms(client)
        targets = select_target_arms(arms, args.arm)
        if not targets:
            raise SetupError("arm.list 里没有任何机械臂")
        target_ids = [str(arm.get("id")) for arm in targets]
        target_ports = {str(arm.get("id")): str(arm.get("port") or "") for arm in targets}
        real_ports = [port for port in target_ports.values() if port]
        print(f"[fd] 目标臂: {', '.join(target_ids)}"
              + (f"（串口: {', '.join(sorted(set(real_ports)))}）" if real_ports else "（全部为 sim 臂，跳过串口 fd 断言）"))

        pid: Optional[int] = None
        pid_source = ""
        if lsof_path and real_ports:
            pid, pid_source = discover_pid(args, lsof_path)
            print(f"[fd] armd pid={pid}（来源: {pid_source}）")

        def check_stray_fds(rows: List[Dict[str, str]], label: str) -> None:
            """armd 持有、但不属于任何配置臂的串口设备 fd：设备换名/旧配置残留/泄漏。"""
            stray = unassigned_serial_fds(rows, list(target_ports.values()))
            for item in stray:
                failures.append(
                    f"{label} armd 持有未配置串口设备的 fd {item}"
                    "（设备换名/旧配置残留/泄漏；若刚拔插过请人工确认）"
                )

        def snapshot_fds() -> Tuple[Dict[str, List[str]], List[Dict[str, str]]]:
            if not (lsof_path and real_ports and pid is not None):
                return {}, []
            rows = lsof_fd_rows(pid, lsof_path)
            counts = {port: count_serial_fds(rows, port) for port in real_ports}
            return counts, rows

        def check_fd_expectations(
            cycle_label: str,
            arms_map: Dict[str, Dict[str, Any]],
            fds: Dict[str, List[str]],
        ) -> None:
            for arm_id in target_ids:
                port = target_ports[arm_id]
                if not port:
                    continue
                connected = bool(arms_map.get(arm_id, {}).get("connected"))
                expected = 1 if connected else 0
                got = fds.get(port, [])
                if len(got) != expected:
                    state = "connected" if connected else "disconnected"
                    failures.append(
                        f"{cycle_label} 臂 {arm_id}（{state}）串口 {port} 期望 {expected} 个 fd，"
                        f"实际 {len(got)}: {got}（疑似 fd 泄漏/未释放）"
                    )
                elif args.verbose:
                    print(f"[fd] {cycle_label} 臂 {arm_id} {port}: {len(got)} 个 fd {got}")

        # ---- 起始基线：连着的臂必须恰好 1 个 fd
        arms_before, missing = wait_all_connected(client, target_ids, args.settle_timeout)
        if missing:
            # 起始就不 connected：仍记录，但给出明确提示（可能是真机未插/被占用）
            failures.append(f"初始状态臂未 connected: {missing}（先检查硬件/daemon）")
        before_fds, rows_before = snapshot_fds()
        check_fd_expectations("cycle 0(基线)", arms_before, before_fds)
        check_stray_fds(rows_before, "初始基线")

        stats_before = {
            arm_id: client.request("arm.stats", {"arm": arm_id}) for arm_id in target_ids
        }

        header = f"{'cycle':>5} | {'arm':<24} | {'fd before':>9} | {'fd after':>8} | {'expect':>6} | {'reconnect':>9} | connected"
        print("\n" + header)
        print("-" * len(header))

        prev_fds = before_fds
        prev_arms = arms_before
        for cycle in range(1, args.cycles + 1):
            cycle_before_fds = prev_fds
            # 本轮 reconnect 之前的状态/fd（上轮 settle 后）
            check_fd_expectations(f"cycle {cycle}(before)", prev_arms, cycle_before_fds)
            reconnect_ms: Dict[str, float] = {}
            reconnect_ok: Dict[str, bool] = {}
            reconnect_error: Dict[str, str] = {}
            for arm in targets:
                arm_id = str(arm.get("id"))
                started = time.monotonic()
                try:
                    result = client.request("arm.reconnect", {"arm": arm_id})
                    reconnect_ok[arm_id] = bool(result) if not isinstance(result, dict) else bool(result.get("ok", True))
                    if isinstance(result, dict) and result.get("ok") is False:
                        reconnect_ok[arm_id] = False
                        reconnect_error[arm_id] = f"ok=false: {result}"
                except RpcError as exc:
                    reconnect_ok[arm_id] = False
                    reconnect_error[arm_id] = f"{exc.code}: {exc.message}"
                    low = f"{exc.code} {exc.message}".lower()
                    if "busy" in low or "resource" in low:
                        failures.append(
                            f"cycle {cycle} 臂 {arm_id} reconnect 报 busy/资源占用: {exc} "
                            f"（串口旧 fd 可能未释放）"
                        )
                    else:
                        failures.append(f"cycle {cycle} 臂 {arm_id} reconnect RPC 错误: {exc}")
                reconnect_ms[arm_id] = (time.monotonic() - started) * 1000.0
            if args.interval > 0:
                time.sleep(args.interval)

            arms_after, missing = wait_all_connected(client, target_ids, args.settle_timeout)
            if missing:
                failures.append(
                    f"cycle {cycle} reconnect 后 {args.settle_timeout:.1f}s 内仍未 connected: {missing}"
                    "（可能是串口 busy / 旧 fd 未释放）"
                )
            after_fds, rows_after = snapshot_fds()
            check_stray_fds(rows_after, f"cycle {cycle}")
            check_fd_expectations(f"cycle {cycle}(after)", arms_after, after_fds)
            prev_fds, prev_arms = after_fds, arms_after

            for arm_id in target_ids:
                port = target_ports[arm_id]
                connected = bool(arms_after.get(arm_id, {}).get("connected"))
                if port:
                    before_n = len(cycle_before_fds.get(port, []))
                    after_n = len(after_fds.get(port, []))
                    fd_cols = (str(before_n), str(after_n), "1" if connected else "0")
                else:
                    fd_cols = ("n/a", "n/a", "sim")
                status = "ok" if reconnect_ok.get(arm_id) else f"FAIL({reconnect_error.get(arm_id, '?')})"
                print(
                    f"{cycle:>5} | {arm_id:<24} | {fd_cols[0]:>9} | {fd_cols[1]:>8} | {fd_cols[2]:>6} | "
                    f"{status + ' ' + str(round(reconnect_ms.get(arm_id, 0.0), 1)) + 'ms':>9} | {connected}"
                )

        # ---- 汇总 stats delta（reconnects/opens/closes）
        print("\n[fd] arm.stats 增量（N 次 reconnect 后）：")
        for arm_id in target_ids:
            after = client.request("arm.stats", {"arm": arm_id})
            before = stats_before.get(arm_id, {})
            delta = {key: stats_delta(before, after, key) for key in ("reconnects", "opens", "closes")}
            print(
                f"  - {arm_id}: reconnects +{delta['reconnects']}, "
                f"opens +{delta['opens']}, closes +{delta['closes']} "
                f"(当前 opens={after.get('opens')}, closes={after.get('closes')})"
            )
            opens, closes = after.get("opens"), after.get("closes")
            if isinstance(opens, int) and isinstance(closes, int) and abs(opens - closes) > 1:
                failures.append(
                    f"臂 {arm_id} stats opens={opens} closes={closes} 差值 >1 "
                    "（稳态最多 1 个 live 串口 handle，疑似 fd 泄漏）"
                )
            if delta["opens"] < args.cycles:
                failures.append(
                    f"臂 {arm_id} 做了 {args.cycles} 次 reconnect，但 stats.opens 只增加 "
                    f"{delta['opens']}（重连可能没真正 reopen）"
                )

        if failures:
            print("\n[fd] FAIL — 发现问题：")
            for item in failures:
                print(f"  ✗ {item}")
            return 1
        print(f"\n[fd] PASS — {args.cycles} 次 reconnect，串口 fd 无泄漏、无 busy、无 RPC 错误")
        return 0
    except SetupError as exc:
        print(f"[fd] 环境错误: {exc}", file=sys.stderr)
        return 2
    except RpcError as exc:
        print(f"[fd] RPC 错误: {exc}", file=sys.stderr)
        return 1
    finally:
        client.close()


if __name__ == "__main__":
    sys.exit(main())
