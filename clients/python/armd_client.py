"""armd RPC 客户端（Python 标准库实现，无第三方依赖）。

用法：
    from armd_client import ArmdClient
    c = ArmdClient("/tmp/armd.sock")
    print(c.arms())
    print(c.state("koch_v1_1:follower"))
    pos = c.read("koch_v1_1:follower")["positions"]
    sub = c.subscribe(lambda f: print(f), arms=["koch_v1_1:follower"], rate_hz=60)
    ...
    c.unsubscribe(sub); c.close()

协议：Unix socket 上的 NDJSON，详见 armd/docs/DESIGN.md。
"""
from __future__ import annotations

import itertools
import json
import os
import socket
import threading
from typing import Any, Callable, Dict, Iterable, List, Optional

DEFAULT_SOCKET = "/tmp/armd.sock"


class ArmdError(Exception):
    """服务端返回的错误（code/message）。"""

    def __init__(self, code: str, message: str):
        super().__init__(f"{code}: {message}")
        self.code = code
        self.message = message


class ArmdClient:
    def __init__(self, path: str = DEFAULT_SOCKET, default_timeout: float = 2.0,
                 token: Optional[str] = None):
        """path 支持 `/tmp/armd.sock`（本机 Unix socket）或 `tcp://host:8771`（ARM 下位机等网络场景）。"""
        self.path = path
        self.default_timeout = default_timeout
        self.token = token or os.environ.get("ARMD_TOKEN") or None
        if path.startswith("tcp://"):
            host, _, port = path[len("tcp://"):].rpartition(":")
            if not host or not port.isdigit():
                raise ArmdError("bad_request", f"非法 TCP 地址 '{path}'（应为 tcp://host:port）")
            self._sock = socket.create_connection((host, int(port)), timeout=10.0)
            self._sock.settimeout(None)
            try:
                self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except OSError:
                pass
        else:
            self._sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self._sock.connect(path)
        self._reader_f = self._sock.makefile("rb")
        self._writer_lock = threading.Lock()
        self._pending: Dict[int, list] = {}
        self._pending_lock = threading.Lock()
        self._ids = itertools.count(1)
        self._subs: Dict[int, Callable[[dict], None]] = {}
        self._subs_lock = threading.Lock()
        self._closed = False
        self._reader = threading.Thread(target=self._read_loop, name="armd-reader", daemon=True)
        self._reader.start()
        if self.token:
            self.request("hello", token=self.token)

    # ------------------------------------------------------------ 底层
    def _read_loop(self) -> None:
        while not self._closed:
            try:
                line = self._reader_f.readline()
            except OSError:
                break
            if not line:
                break
            try:
                msg = json.loads(line)
            except json.JSONDecodeError:
                continue
            if "event" in msg:
                sub = msg.get("sub")
                with self._subs_lock:
                    cb = self._subs.get(sub)
                if cb is not None:
                    try:
                        cb(msg)
                    except Exception:  # 订阅回调异常不能影响读线程
                        pass
                continue
            rid = msg.get("id")
            with self._pending_lock:
                slot = self._pending.pop(rid, None)
            if slot is not None:
                result, error, event = slot
                if msg.get("error"):
                    error["error"] = msg["error"]
                else:
                    result["result"] = msg.get("result")
                event.set()
        # 连接断了：唤醒所有等待者
        with self._pending_lock:
            pend = list(self._pending.values())
            self._pending.clear()
        for _, _err, ev in pend:
            ev.set()

    def _send(self, msg: dict) -> None:
        data = (json.dumps(msg, separators=(",", ":")) + "\n").encode()
        with self._writer_lock:
            self._sock.sendall(data)

    def request(self, method: str, timeout: Optional[float] = None, **params: Any) -> Any:
        if self._closed:
            raise ArmdError("closed", "client closed")
        rid = next(self._ids)
        ev = threading.Event()
        result: dict = {}
        error: dict = {}
        with self._pending_lock:
            self._pending[rid] = [result, error, ev]
        self._send({"id": rid, "method": method, "params": params})
        if not ev.wait(self.default_timeout if timeout is None else timeout):
            with self._pending_lock:
                self._pending.pop(rid, None)
            raise ArmdError("timeout", f"{method} 超时")
        if "error" in error and error["error"]:
            e = error["error"]
            raise ArmdError(e.get("code", "error"), e.get("message", ""))
        return result.get("result")

    # ------------------------------------------------------------ 机械臂 API
    def ping(self) -> dict:
        return self.request("ping")

    def info(self) -> dict:
        return self.request("info")

    def arms(self) -> List[dict]:
        return self.request("arm.list")

    def state(self, arm: str) -> dict:
        return self.request("arm.state", arm=arm)

    def profile(self, arm: str) -> dict:
        """读取标定档案 {profile, path, has_profile}。"""
        return self.request("arm.profile.get", arm=arm)

    def set_profile(self, arm: str, profile: dict) -> dict:
        """保存标定档案（由 armd 校验并持久化）。"""
        return self.request("arm.profile.set", arm=arm, profile=profile)

    def profile_history(self, arm: str) -> dict:
        """标定历史快照列表 {history:[{name,savedAt}], dir}。"""
        return self.request("arm.profile.history", arm=arm)

    def restore_profile(self, arm: str, name: str) -> dict:
        """从历史快照恢复标定（armd 校验后覆盖当前 profile）。"""
        return self.request("arm.profile.restore", arm=arm, name=name)

    def delete_profile(self, arm: str) -> dict:
        return self.request("arm.profile.delete", arm=arm)

    def calibration_snapshot(self, arm: str) -> dict:
        """一次取当前 raw（按关节名）/角度/健康，用于标定采点。"""
        return self.request("arm.calibration.snapshot", arm=arm)

    def degrees(self, arm: str) -> Optional[dict]:
        """当前关节角（度）；没有 profile 时为 None。"""
        st = self.state(arm)
        return st.get("deg") or None

    def read(self, arm: str, motors: Optional[Iterable[int]] = None, timeout_ms: int = 50) -> dict:
        params: dict = {"arm": arm, "timeout_ms": timeout_ms}
        if motors is not None:
            params["motors"] = list(motors)
        return self.request("arm.read", **params)

    def read_register(self, arm: str, motor: int, address: int, length: int) -> dict:
        return self.request("arm.read_register", arm=arm, motor=motor, address=address, length=length)

    def write_register(self, arm: str, motor: int, address: int, length: int, value: int) -> dict:
        return self.request(
            "arm.write_register", arm=arm, motor=motor, address=address, length=length, value=value
        )

    def sync_write(self, arm: str, address: int, values: Dict[int, int], length: int = 4) -> dict:
        return self.request(
            "arm.sync_write", arm=arm, address=address, length=length,
            values={str(k): v for k, v in values.items()},
        )

    def torque(self, arm: str, enable: bool, motors: Optional[Iterable[int]] = None) -> dict:
        params: dict = {"arm": arm, "enable": enable}
        if motors is not None:
            params["motors"] = list(motors)
        return self.request("arm.torque", **params)

    def ping_motor(self, arm: str, motor: int) -> dict:
        return self.request("arm.ping", arm=arm, motor=motor)

    def set_rate(self, arm: str, hz: float) -> dict:
        return self.request("arm.set_rate", arm=arm, hz=hz)

    def reconnect(self, arm: str) -> dict:
        return self.request("arm.reconnect", arm=arm)

    def stats(self, arm: str) -> dict:
        return self.request("arm.stats", arm=arm)

    # ------------------------------------------------------------ 发布订阅
    def subscribe(
        self,
        callback: Callable[[dict], None],
        arms: Optional[Iterable[str]] = None,
        rate_hz: float = 60.0,
    ) -> int:
        params: dict = {"rate_hz": rate_hz}
        if arms is not None:
            params["arms"] = list(arms)
        res = self.request("sub.subscribe", **params)
        sub_id = res["sub_id"]
        with self._subs_lock:
            self._subs[sub_id] = callback
        return sub_id

    def unsubscribe(self, sub_id: int) -> dict:
        with self._subs_lock:
            self._subs.pop(sub_id, None)
        return self.request("sub.unsubscribe", sub_id=sub_id)

    def set_sub_rate(self, sub_id: int, rate_hz: float) -> dict:
        return self.request("sub.rate", sub_id=sub_id, rate_hz=rate_hz)

    def close(self) -> None:
        self._closed = True
        try:
            self._sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            self._sock.close()
        except OSError:
            pass


class StateStream:
    """订阅多个臂，保存最新状态快照（线程安全），供轮询式代码使用。

    stream = StateStream(client, ["koch_v1_1:follower"], rate_hz=60)
    stream.latest("koch_v1_1:follower")  -> {"joints": {...}, "seq": ...} | None
    stream.close()
    """

    def __init__(self, client: ArmdClient, arms: Iterable[str], rate_hz: float = 60.0):
        self.client = client
        self._lock = threading.Lock()
        self._latest: Dict[str, dict] = {}
        self._counts: Dict[str, int] = {}
        self.sub_id = client.subscribe(self._on_frame, arms=list(arms), rate_hz=rate_hz)

    def _on_frame(self, frame: dict) -> None:
        if frame.get("event") != "state":
            return
        arm = frame.get("arm")
        with self._lock:
            self._latest[arm] = frame
            self._counts[arm] = self._counts.get(arm, 0) + 1

    def latest(self, arm: str) -> Optional[dict]:
        with self._lock:
            return self._latest.get(arm)

    def frames(self, arm: str) -> int:
        with self._lock:
            return self._counts.get(arm, 0)

    def close(self) -> None:
        try:
            self.client.unsubscribe(self.sub_id)
        except ArmdError:
            pass


# ------------------------------------------------------------------ CLI
def _main() -> None:
    import argparse

    ap = argparse.ArgumentParser(description="armd RPC 客户端")
    ap.add_argument("--socket", default=DEFAULT_SOCKET,
                    help="Unix socket 路径，或 tcp://host:port")
    ap.add_argument("--token", default=None, help="访问令牌（也可用环境变量 ARMD_TOKEN）")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("info")
    sub.add_parser("arms")
    p = sub.add_parser("state"); p.add_argument("arm")
    p = sub.add_parser("read"); p.add_argument("arm")
    p = sub.add_parser("stats"); p.add_argument("arm")
    p = sub.add_parser("profile"); p.add_argument("arm")
    p.add_argument("--set", default=None, help="profile JSON 文件路径")
    p.add_argument("--delete", action="store_true")
    p.add_argument("--history", action="store_true")
    p.add_argument("--restore", default=None, help="历史快照文件名")
    p = sub.add_parser("snapshot"); p.add_argument("arm")
    p = sub.add_parser("watch"); p.add_argument("arm"); p.add_argument("--rate", type=float, default=10.0)
    p = sub.add_parser("torque"); p.add_argument("arm"); p.add_argument("on", type=int)
    p = sub.add_parser("write"); p.add_argument("arm"); p.add_argument("motor", type=int); p.add_argument("address", type=int); p.add_argument("value", type=int); p.add_argument("--len", type=int, default=4)
    args = ap.parse_args()
    c = ArmdClient(args.socket, token=args.token)
    try:
        if args.cmd == "info":
            print(json.dumps(c.info(), ensure_ascii=False, indent=2))
        elif args.cmd == "arms":
            print(json.dumps(c.arms(), ensure_ascii=False, indent=2))
        elif args.cmd == "state":
            print(json.dumps(c.state(args.arm), ensure_ascii=False, indent=2))
        elif args.cmd == "read":
            print(json.dumps(c.read(args.arm), ensure_ascii=False, indent=2))
        elif args.cmd == "snapshot":
            print(json.dumps(c.calibration_snapshot(args.arm), ensure_ascii=False, indent=2))
        elif args.cmd == "profile":
            if args.delete:
                print(json.dumps(c.delete_profile(args.arm)))
            elif args.history:
                print(json.dumps(c.profile_history(args.arm), ensure_ascii=False, indent=2))
            elif args.restore:
                print(json.dumps(c.restore_profile(args.arm, args.restore), ensure_ascii=False, indent=2))
            elif args.set:
                with open(args.set, encoding="utf-8") as fp:
                    out = c.set_profile(args.arm, json.load(fp))
                print(json.dumps(out, ensure_ascii=False, indent=2))
            else:
                print(json.dumps(c.profile(args.arm), ensure_ascii=False, indent=2))
        elif args.cmd == "stats":
            print(json.dumps(c.stats(args.arm), ensure_ascii=False, indent=2))
        elif args.cmd == "torque":
            print(c.torque(args.arm, bool(args.on)))
        elif args.cmd == "write":
            print(c.write_register(args.arm, args.motor, args.address, args.len, args.value))
        elif args.cmd == "watch":
            stop = threading.Event()
            def on_frame(f):
                print(json.dumps(f, ensure_ascii=False))
            sub_id = c.subscribe(on_frame, arms=[args.arm], rate_hz=args.rate)
            try:
                while not stop.wait(1.0):
                    pass
            except KeyboardInterrupt:
                pass
            finally:
                c.unsubscribe(sub_id)
    finally:
        c.close()


if __name__ == "__main__":
    _main()
