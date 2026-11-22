#!/usr/bin/env python3
"""armd v0.2.0 仿真实测验收（PR evidence 用，Python 标准库，无第三方依赖）。

这个脚本不依赖仓库里的临时目录约定：它自己在临时目录里生成 sim 臂配置
（socket / identity / profile / `<socket>.lock` 全部落在临时目录内），
拉起 `armd --config <tmp>/config.json`，然后通过 Unix socket 的 NDJSON RPC +
订阅跑完整验收流程，最后杀进程并清理临时目录。**不会碰 /tmp/armd.sock 或生产 daemon。**

覆盖的验收场景（与 docs/SAFETY-DESIGN.md §7 对齐）：
  1. sim 臂连接：`arm.state.connected=true`、joints/velocity 有数据；
  2. 堵转：`arm.sim.inject kind=stall` → ≤1.5s 收到 `event:"stall"`，
     该电机 torque OFF + `stalls["3"]` + `read_register(64)==0`，其他电机/状态流正常、无硬件错误；
  3. 单电机硬件错误：`kind=hardware_error` → `motor_errors["3"].code==128` 且整臂仍 connected，
     `arm.read` 只在该电机返回 errors、无 positions；`arm.reboot` 清除后 torque=false 且可重新使能；
  4. 软限位 clamp：Goal=5000 → `clamped["1"].applied <= 4085` + warnings，`state.goals[1]` 同步；
  5. 20 次 `arm.reconnect`：订阅不断流、`state.seq` 继续增长、`stats.opens - stats.closes == 1`（恰好 1 个 live handle）；
  6. 传输失败释放 fd：`kind=transport_error` → connected=false 时 `stats.opens == stats.closes`
     （旧 handle 已释放，不占着串口），自动重连后恢复差值 1 且订阅流恢复；
  7. 独立 reject 模式 daemon：Goal=5000 → RPC 错误码 `soft_limit`。

用法：
    python3 scripts/sim_acceptance.py
    python3 scripts/sim_acceptance.py --binary /tmp/armd-target/debug/armd --keep --verbose

退出码：0 = 全部通过；1 = 验收失败；2 = 环境错误（找不到 armd 二进制等）。
"""
from __future__ import annotations

import argparse
import json
import os
import queue
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

ARM = "sim"

#: armd 二进制候选路径（相对仓库根目录的 target 目录 + 环境变量）
BIN_CANDIDATES = (
    "/tmp/armd-target/debug/armd",
    "target/debug/armd",
    "/tmp/armd-target/release/armd",
    "target/release/armd",
)

RECONNECT_CYCLES = 20


class SimAcceptError(Exception):
    """环境错误（退出码 2）。"""


class RpcError(Exception):
    def __init__(self, code: str, message: str):
        super().__init__(f"{code}: {message}")
        self.code = code
        self.message = message


def check(condition: Any, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def phase(name: str) -> None:
    print(f"\n=== {name} ===")


# ---------------------------------------------------------------- RPC 客户端


class RpcClient:
    def __init__(self, path: str, timeout: float = 5.0):
        self.path = path
        self.timeout = timeout
        self._sock: Optional[socket.socket] = None
        self._buf = b""
        self._next_id = 0

    def connect(self, retries: int = 100) -> "RpcClient":
        last: Optional[OSError] = None
        for _ in range(retries):
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            sock.settimeout(self.timeout)
            try:
                sock.connect(self.path)
            except OSError as exc:
                sock.close()
                last = exc
                time.sleep(0.05)
                continue
            self._sock = sock
            return self
        raise SimAcceptError(f"连接 sim daemon socket 失败 {self.path}: {last}")

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None

    def request(self, method: str, params: Optional[Dict[str, Any]] = None) -> Any:
        if self._sock is None:
            raise RpcError("closed", "RPC 连接已关闭")
        self._next_id += 1
        request_id = self._next_id
        payload = json.dumps(
            {"id": request_id, "method": method, "params": params or {}}, ensure_ascii=False
        )
        self._sock.sendall(payload.encode("utf-8") + b"\n")
        deadline = time.monotonic() + self.timeout
        while True:
            line = self._readline(deadline)
            if line is None:
                raise RpcError("timeout", f"{method} 未在 {self.timeout:.1f}s 内返回")
            try:
                frame = json.loads(line)
            except json.JSONDecodeError:
                continue
            if not isinstance(frame, dict) or frame.get("id") != request_id:
                continue
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
                raise RpcError("closed", "sim daemon 关闭了连接")
            self._buf += chunk
        line, _sep, self._buf = self._buf.partition(b"\n")
        return line.decode("utf-8", "replace").strip()


# ---------------------------------------------------------------- 订阅


class Subscriber:
    """独立连接上的 sub.subscribe；后台线程收帧进队列，测试侧做谓词等待。"""

    def __init__(self, path: str, arms: List[str], rate_hz: float = 60.0, timeout: float = 5.0):
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        sock.connect(path)
        self._sock = sock
        self._timeout = timeout
        self._buf = b""
        request = json.dumps(
            {"id": 1, "method": "sub.subscribe", "params": {"arms": arms, "rate_hz": rate_hz}}
        )
        sock.sendall(request.encode() + b"\n")
        frame = None
        deadline = time.monotonic() + timeout
        while frame is None:
            line = self._readline(deadline)
            if line is None:
                raise SimAcceptError("sub.subscribe 超时")
            try:
                candidate = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(candidate, dict) and candidate.get("id") == 1:
                frame = candidate
        if "error" in frame:
            error = frame["error"]
            raise RpcError(str(error.get("code", "?")), str(error.get("message", "")))
        self.sub_id = int(frame.get("result", {}).get("sub_id", 0))
        self._queue: "queue.Queue[Dict[str, Any]]" = queue.Queue()
        self._thread = threading.Thread(target=self._reader_loop, name="armd-sim-sub", daemon=True)
        self._thread.start()

    def _readline(self, deadline: float) -> Optional[str]:
        while b"\n" not in self._buf:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            self._sock.settimeout(min(0.5, max(0.01, remaining)))
            try:
                chunk = self._sock.recv(65536)
            except socket.timeout:
                continue
            except OSError:
                return None
            if not chunk:
                return None
            self._buf += chunk
        line, _sep, self._buf = self._buf.partition(b"\n")
        return line.decode("utf-8", "replace").strip()

    def _reader_loop(self) -> None:
        self._sock.settimeout(0.5)
        while True:
            try:
                chunk = self._sock.recv(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break
            self._buf += chunk
            while b"\n" in self._buf:
                line, _sep, self._buf = self._buf.partition(b"\n")
                text = line.decode("utf-8", "replace").strip()
                if not text:
                    continue
                try:
                    frame = json.loads(text)
                except json.JSONDecodeError:
                    continue
                if isinstance(frame, dict):
                    self._queue.put(frame)

    def wait_for(
        self, pred: Callable[[Dict[str, Any]], bool], timeout: float
    ) -> Optional[Dict[str, Any]]:
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            try:
                frame = self._queue.get(timeout=min(0.2, remaining))
            except queue.Empty:
                continue
            if pred(frame):
                return frame

    def drain(self) -> None:
        while True:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                return

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass


# ---------------------------------------------------------------- sim daemon


def sim_arm_config(mode: str, reconnect_sec: float) -> Dict[str, Any]:
    return {
        "id": ARM,
        "label": "sim arm",
        "sim": True,
        "port": "",
        "poll_hz": 60,
        "read_timeout_ms": 30,
        "motors": [1, 2, 3, 4, 5, 6],
        "reconnect_sec": reconnect_sec,
        "stall_protection": True,
        "stall_timeout_ms": 250,
        "stall_tolerance": 60,
        "stall_velocity_epsilon": 2,
        "stall_exclude_motors": [],
        "auto_torque_off": True,
        "stall_recover_margin": 20,
        "soft_limits": True,
        "soft_limit_mode": mode,
        "soft_limit_margin": 10,
        "joint_names": {
            "1": "shoulder_pan",
            "2": "shoulder_lift",
            "3": "elbow_flex",
            "4": "wrist_flex",
            "5": "wrist_roll",
            "6": "gripper",
        },
    }


class SimDaemon:
    """临时目录里的 sim armd（socket/identity/profile/lock 全部隔离）。"""

    def __init__(self, binary: str, mode: str = "clamp", reconnect_sec: float = 2.0, keep: bool = False):
        self.binary = binary
        self.mode = mode
        self.keep = keep
        self.dir = tempfile.mkdtemp(prefix=f"armd-sim-accept-{mode}-")
        self.socket_path = os.path.join(self.dir, "armd.sock")
        self.identity_path = os.path.join(self.dir, "identity.json")
        self.log_path = os.path.join(self.dir, "daemon.log")
        self.config_path = os.path.join(self.dir, "config.json")
        self.profile_path = os.path.join(self.dir, "profile.json")
        config = {
            "socket": self.socket_path,
            "identity": self.identity_path,
            "arms": [sim_arm_config(mode, reconnect_sec)],
        }
        config["arms"][0]["profile"] = self.profile_path
        with open(self.config_path, "w", encoding="utf-8") as fh:
            json.dump(config, fh, ensure_ascii=False, indent=2)
        self._log = open(self.log_path, "w", encoding="utf-8")
        try:
            self.child = subprocess.Popen(
                [binary, "--config", self.config_path],
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=self._log,
            )
        except OSError as exc:
            self._log.close()
            raise SimAcceptError(f"启动 armd 失败 {binary}: {exc}") from exc
        self._wait_socket()

    def _wait_socket(self) -> None:
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if self.child.poll() is not None:
                raise SimAcceptError(
                    f"armd 启动即退出（mode={self.mode}, code={self.child.returncode}）:\n"
                    + self.log_tail()
                )
            probe = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            probe.settimeout(0.2)
            try:
                probe.connect(self.socket_path)
            except OSError:
                probe.close()
                time.sleep(0.05)
                continue
            probe.close()
            return
        raise SimAcceptError(f"armd socket 10s 内未就绪:\n{self.log_tail()}")

    def connect(self) -> RpcClient:
        return RpcClient(self.socket_path).connect()

    def subscribe(self, rate_hz: float = 60.0) -> Subscriber:
        return Subscriber(self.socket_path, [ARM], rate_hz=rate_hz)

    def log_tail(self, lines: int = 40) -> str:
        try:
            self._log.flush()
        except (OSError, ValueError):
            pass
        try:
            with open(self.log_path, "r", encoding="utf-8", errors="replace") as fh:
                content = fh.read().splitlines()
        except OSError:
            return "<no log>"
        return "\n".join(content[-lines:])

    def stop(self) -> None:
        if getattr(self, "child", None) is not None and self.child.poll() is None:
            self.child.terminate()
            try:
                self.child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.child.kill()
                self.child.wait(timeout=5)
        try:
            self._log.close()
        except (OSError, ValueError):
            pass
        if self.keep:
            print(f"[sim-accept] 保留临时目录（含 daemon.log）: {self.dir}")
        else:
            shutil.rmtree(self.dir, ignore_errors=True)


# ---------------------------------------------------------------- 状态辅助


def wait_state(
    client: RpcClient, pred: Callable[[Dict[str, Any]], bool], timeout: float, what: str
) -> Dict[str, Any]:
    deadline = time.monotonic() + timeout
    last: Dict[str, Any] = {}
    while time.monotonic() < deadline:
        last = client.request("arm.state", {"arm": ARM})
        if pred(last):
            return last
        time.sleep(0.02)
    raise AssertionError(f"等待状态条件超时（{what}，{timeout:.1f}s）；最后一次 arm.state: {last}")


def wait_connected(client: RpcClient, timeout: float = 5.0) -> Dict[str, Any]:
    return wait_state(
        client,
        lambda s: s.get("connected") is True and bool(s.get("joints")),
        timeout,
        "connected=true 且已有 joints",
    )


def motor_map(state: Dict[str, Any], field: str) -> Dict[str, Any]:
    value = state.get(field) or {}
    return value if isinstance(value, dict) else {}


# ---------------------------------------------------------------- 验收流程


def run_clamp_acceptance(daemon: SimDaemon, verbose: bool) -> None:
    client = daemon.connect()
    sub = daemon.subscribe(60.0)
    try:
        phase("0. 连接与状态")
        info = client.request("info")
        check(ARM in (info.get("arms") or []), f"info.arms 应包含 {ARM}: {info}")
        state = wait_connected(client)
        check(state.get("seq", 0) > 0, f"state.seq 应 > 0: {state}")
        joints = motor_map(state, "joints")
        for motor in ("1", "2", "3", "4", "5", "6"):
            check(motor in joints, f"joints 缺少电机 {motor}: {state}")
        print(f"PASS 连接: seq={state['seq']} hz={state.get('hz')} joints={sorted(joints)}")

        # ---- 1. 堵转保护
        phase("1. 堵转保护（stall）")
        client.request("arm.torque", {"arm": ARM, "enable": True, "motors": [3]})
        wait_state(
            client,
            lambda s: motor_map(s, "torque").get("3") is True,
            2.0,
            "torque[3]=true",
        )
        present = int(motor_map(client.request("arm.state", {"arm": ARM}), "joints")["3"])
        goal = present + 800
        ack = client.request(
            "arm.sync_write",
            {"arm": ARM, "address": 116, "length": 4, "values": {"3": goal}},
        )
        check(ack.get("ok") is True, f"写 goal 应成功: {ack}")
        sub.drain()
        t0 = time.monotonic()
        client.request("arm.sim.inject", {"arm": ARM, "kind": "stall", "motors": [3]})
        event = sub.wait_for(
            lambda f: f.get("event") == "stall" and f.get("motor") == 3, 1.5
        )
        elapsed_ms = (time.monotonic() - t0) * 1000.0
        check(event is not None, "注入 stall 后 1.5s 内没有收到 event:\"stall\"")
        check(event.get("torque_off") is True, f"stall 事件 torque_off 应为 true: {event}")
        for key in ("reason", "position", "goal", "stall_ms"):
            check(key in event, f"stall 事件缺少 {key}: {event}")
        stalled = wait_state(
            client,
            lambda s: "3" in motor_map(s, "stalls")
            and motor_map(s, "torque").get("3") is False,
            2.0,
            "stalls[3] 出现且 torque[3]=false",
        )
        check(stalled.get("connected") is True, f"单电机堵转不应断开整臂: {stalled}")
        check(
            not motor_map(stalled, "motor_errors"),
            f"堵转不应产生 hardware_error: {stalled.get('motor_errors')}",
        )
        for motor in ("1", "2", "4", "5", "6"):
            check(
                motor in motor_map(stalled, "joints"),
                f"电机 {motor} 的 joints 不应受电机 3 堵转影响: {stalled}",
            )
        reg = client.request(
            "arm.read_register", {"arm": ARM, "motor": 3, "address": 64, "length": 1}
        )
        check(reg.get("value") == 0, f"stall 后 Torque_Enable(64) 应为 0: {reg}")
        seq_before = stalled.get("seq", 0)
        client.request("arm.sim.inject", {"arm": ARM, "kind": "clear", "motors": [3]})
        wait_state(client, lambda s: not motor_map(s, "motor_errors"), 1.0, "clear 后无 motor_errors")
        time.sleep(0.25)
        after = client.request("arm.state", {"arm": ARM})
        check(after.get("seq", 0) > seq_before, f"堵转后状态流应继续: {after.get('seq')}")
        hw = client.request(
            "arm.read_register", {"arm": ARM, "motor": 3, "address": 70, "length": 1}
        )
        check(hw.get("value") == 0, f"stall 后不应留下 Hardware_Error_Status: {hw}")
        print(
            f"PASS 堵转: 事件延迟 {elapsed_ms:.0f}ms，stall_ms={event.get('stall_ms')}，"
            f"torque_off 已生效，无硬件错误，其他电机正常"
        )

        # ---- 2. 单电机硬件错误隔离 + reboot
        phase("2. 单电机硬件错误隔离（hardware_error + reboot）")
        sub.drain()
        t0 = time.monotonic()
        client.request("arm.sim.inject", {"arm": ARM, "kind": "hardware_error", "motors": [3]})
        event = sub.wait_for(
            lambda f: f.get("event") == "motor_error" and f.get("motor") == 3, 1.0
        )
        check(event is not None, "注入 hardware_error 后 1s 内没有收到 event:\"motor_error\"")
        check(event.get("code") == 128, f"motor_error 事件 code 应为 128: {event}")
        check(
            "hardware_alert" in (event.get("names") or []),
            f"motor_error 事件 names 应含 hardware_alert: {event}",
        )
        err_state = wait_state(
            client,
            lambda s: (motor_map(s, "motor_errors").get("3") or {}).get("code") == 128,
            1.0,
            "motor_errors[3].code==128",
        )
        check(err_state.get("connected") is True, f"单电机错误不应断开整臂: {err_state}")
        check(
            set(motor_map(err_state, "motor_errors")) == {"3"},
            f"motor_errors 只应包含电机 3: {err_state.get('motor_errors')}",
        )
        for motor in ("1", "2", "4", "5", "6"):
            check(
                motor in motor_map(err_state, "joints"),
                f"电机 {motor} 应继续更新: {err_state}",
            )
        read = client.request(
            "arm.read", {"arm": ARM, "motors": [1, 2, 3, 4, 5, 6], "timeout_ms": 100}
        )
        check("3" not in (read.get("positions") or {}), f"hardware_error 电机不应有 data: {read}")
        check((read.get("errors") or {}).get("3") == 128, f"arm.read.errors[3] 应为 128: {read}")
        reboot = client.request("arm.reboot", {"arm": ARM, "motors": [3]})
        check(reboot.get("ok") is True, f"arm.reboot 应成功: {reboot}")
        check(3 in (reboot.get("rebooted") or []), f"rebooted 应包含 3: {reboot}")
        after_reboot = wait_state(
            client,
            lambda s: not motor_map(s, "motor_errors")
            and motor_map(s, "torque").get("3") is False
            and s.get("connected") is True,
            3.0,
            "reboot 后 motor_errors 清空 + torque[3]=false + connected",
        )
        client.request("arm.torque", {"arm": ARM, "enable": True, "motors": [3]})
        wait_state(
            client,
            lambda s: motor_map(s, "torque").get("3") is True,
            2.0,
            "reboot 后 torque 可重新使能",
        )
        print(
            f"PASS 硬件错误隔离: 仅 motor 3 error(0x80)，connected={after_reboot.get('connected')}，"
            f"reboot 后清除且可重新使能"
        )

        # ---- 3. 软限位 clamp
        phase("3. 软限位 clamp（Goal=5000 → 4085）")
        limit_state = wait_state(
            client, lambda s: "1" in motor_map(s, "limits"), 3.0, "limits[1] 已读到"
        )
        limit = motor_map(limit_state, "limits")["1"]
        check(
            int(limit.get("max", 0)) == 4095,
            f"sim 默认 EEPROM max 应为 4095: {limit}",
        )
        result = client.request(
            "arm.sync_write",
            {"arm": ARM, "address": 116, "length": 4, "values": {"1": 5000}},
        )
        check(result.get("ok") is True, f"clamp 模式写 Goal=5000 应成功: {result}")
        applied = None
        clamped = (result.get("clamped") or {}).get("1") or {}
        if isinstance(clamped, dict):
            applied = clamped.get("applied")
        if applied is None:
            applied = result.get("applied")
        check(
            isinstance(applied, int) and applied <= 4085,
            f"clamp 后 applied 应 <= 4085: {result}",
        )
        if result.get("warnings"):
            print(f"  clamp warnings: {result['warnings']}")
        wait_state(
            client,
            lambda s: motor_map(s, "goals").get("1") == applied,
            2.0,
            f"state.goals[1]=={applied}",
        )
        print(f"PASS 软限位 clamp: 5000 → {applied}（soft_limit_margin=10）")

        # ---- 4. 20 次 reconnect
        phase(f"4. 重连 ×{RECONNECT_CYCLES}（订阅不断流、句柄不泄漏）")
        sub.drain()
        stats_before = client.request("arm.stats", {"arm": ARM})
        seq_before = client.request("arm.state", {"arm": ARM}).get("seq", 0)
        for index in range(RECONNECT_CYCLES):
            ack = client.request("arm.reconnect", {"arm": ARM})
            check(ack.get("ok") is True, f"第 {index} 次 reconnect 应成功: {ack}")
        recovered = wait_state(
            client,
            lambda s: s.get("connected") is True and s.get("seq", 0) > seq_before,
            5.0,
            f"{RECONNECT_CYCLES} 次 reconnect 后 connected 且 seq 前进",
        )
        frame = sub.wait_for(
            lambda f: f.get("event") == "state" and f.get("seq", 0) > recovered.get("seq", 0),
            2.0,
        )
        check(frame is not None, "reconnect 后订阅没有新的 state 帧（流中断）")
        stats = client.request("arm.stats", {"arm": ARM})
        opens, closes = stats.get("opens"), stats.get("closes")
        check(isinstance(opens, int) and isinstance(closes, int), f"stats 缺 opens/closes: {stats}")
        check(
            opens - closes == 1,
            f"稳态应恰好 1 个 live handle（opens-closes==1），实际 opens={opens} closes={closes}",
        )
        check(
            opens - int(stats_before.get("opens", 0)) >= RECONNECT_CYCLES,
            f"{RECONNECT_CYCLES} 次 reconnect 后 opens 增量不足: {stats_before.get('opens')}→{opens}",
        )
        print(
            f"PASS reconnect: opens={opens} closes={closes}（差值 1），"
            f"stats.reconnects={stats.get('reconnects')}，订阅流恢复"
        )

        # ---- 5. 传输失败 → 释放 fd → 自动重连
        phase("5. 传输失败（transport_error）→ 旧 handle 先释放再重连")
        seq_before_transport = client.request("arm.state", {"arm": ARM}).get("seq", 0)
        client.request("arm.sim.inject", {"arm": ARM, "kind": "transport_error"})
        disconnected = wait_state(
            client, lambda s: s.get("connected") is False, 3.0, "transport_error 后 connected=false"
        )
        check(disconnected.get("connected") is False, f"应已断线: {disconnected}")
        stats_down = client.request("arm.stats", {"arm": ARM})
        check(
            stats_down.get("opens") == stats_down.get("closes"),
            "掉线时旧 handle 必须已释放（opens==closes），"
            f"实际 opens={stats_down.get('opens')} closes={stats_down.get('closes')}",
        )
        try:
            client.request("arm.sim.inject", {"arm": ARM, "kind": "clear"})
        except RpcError as exc:
            # 串口已关闭时 clear 返回错误是预期的：重连会开一个无故障的新 SimPort
            print(f"  （掉线期间 clear 返回 {exc.code}，符合预期：串口已关闭）")
        reconnected = wait_state(
            client,
            lambda s: s.get("connected") is True and s.get("seq", 0) > seq_before_transport,
            8.0,
            "transport_error 后自动重连并恢复状态流",
        )
        stats_after = client.request("arm.stats", {"arm": ARM})
        check(
            (stats_after.get("opens", 0) or 0) - (stats_after.get("closes", 0) or 0) == 1,
            f"重连后应恢复为 1 个 live handle: {stats_after}",
        )
        frame = sub.wait_for(
            lambda f: f.get("event") == "state"
            and f.get("seq", 0) > reconnected.get("seq", 0),
            2.0,
        )
        check(frame is not None, "传输恢复后订阅没有新 state 帧")
        print(
            f"PASS 传输失败 fd 释放: 掉线时 opens={stats_down.get('opens')}=closes={stats_down.get('closes')}，"
            f"重连后差值 1，订阅流恢复"
        )
    finally:
        sub.close()
        client.close()


def run_reject_acceptance(daemon: SimDaemon) -> None:
    client = daemon.connect()
    try:
        phase("6. 软限位 reject 模式（Goal=5000 → soft_limit）")
        wait_connected(client)
        wait_state(client, lambda s: "1" in motor_map(s, "limits"), 3.0, "limits[1] 已读到")
        try:
            client.request(
                "arm.sync_write",
                {"arm": ARM, "address": 116, "length": 4, "values": {"1": 5000}},
            )
        except RpcError as exc:
            check(exc.code == "soft_limit", f"reject 模式应返回 soft_limit，实际 {exc.code}: {exc}")
            print(f"PASS 软限位 reject: {exc}")
            return
        raise AssertionError("reject 模式下写 Goal=5000 应失败（soft_limit），但成功了")
    finally:
        client.close()


# ---------------------------------------------------------------- 入口


def find_binary(explicit: Optional[str]) -> str:
    if explicit:
        if not os.path.isfile(explicit):
            raise SimAcceptError(f"--binary 不存在: {explicit}")
        return explicit
    env_bin = os.environ.get("ARMD_BIN")
    if env_bin:
        if not os.path.isfile(env_bin):
            raise SimAcceptError(f"ARMD_BIN 不存在: {env_bin}")
        return env_bin
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    for candidate in BIN_CANDIDATES:
        path = candidate if os.path.isabs(candidate) else os.path.join(repo_root, candidate)
        if os.path.isfile(path) and os.access(path, os.X_OK):
            return path
    raise SimAcceptError(
        "找不到 armd 二进制；先 `cargo build`，或用 --binary / ARMD_BIN 指定"
    )


def parse_args(argv: Optional[List[str]]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="armd v0.2.0 仿真验收（PR evidence；临时目录内起 sim daemon，跑完清理）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--binary", default=None, help="armd 二进制路径（默认 ARMD_BIN 或 /tmp/armd-target/debug/armd）")
    parser.add_argument("--keep", action="store_true", help="保留临时目录与 daemon.log（排查用）")
    parser.add_argument("--verbose", action="store_true", help="打印更多细节")
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    try:
        binary = find_binary(args.binary)
    except SimAcceptError as exc:
        print(f"[sim-accept] 环境错误: {exc}", file=sys.stderr)
        return 2
    print(
        f"[sim-accept] 二进制: {binary}\n"
        f"[sim-accept] 临时目录隔离（不碰 /tmp/armd.sock / /tmp/armd-identity.json）"
    )
    started = time.monotonic()
    daemon: Optional[SimDaemon] = None
    reject_daemon: Optional[SimDaemon] = None
    try:
        daemon = SimDaemon(binary, mode="clamp", reconnect_sec=2.0, keep=args.keep)
        run_clamp_acceptance(daemon, args.verbose)
        daemon.stop()
        daemon = None

        reject_daemon = SimDaemon(binary, mode="reject", reconnect_sec=1.0, keep=args.keep)
        run_reject_acceptance(reject_daemon)
        reject_daemon.stop()
        reject_daemon = None
    except (AssertionError, RpcError) as exc:
        log = ""
        for handle in (daemon, reject_daemon):
            if handle is not None:
                log += f"\n--- armd 日志（{handle.mode}）---\n{handle.log_tail()}"
        print(f"\n[sim-accept] FAIL: {exc}{log}", file=sys.stderr)
        return 1
    except (SimAcceptError, OSError) as exc:
        print(f"\n[sim-accept] 环境错误: {exc}", file=sys.stderr)
        return 2
    finally:
        for handle in (daemon, reject_daemon):
            if handle is not None:
                handle.stop()
    elapsed = time.monotonic() - started
    print(f"\n[sim-accept] PASS — 全部 {7} 组仿真验收通过（{elapsed:.1f}s）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
