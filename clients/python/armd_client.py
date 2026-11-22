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

安全层（v0.2.0）：
    c.reboot(arm, motors=[3])               # Dynamixel Reboot(0x08)，清 latched 硬件错误
    c.clear_error(arm)                      # 清 armd 侧 error/stall 标记（读 Hardware_Error_Status）
    c.torque_off(arm, motors=[3])           # 便捷 torque OFF（等价 arm.torque enable=false）
    c.sim_inject(arm, "stall", motors=[3])  # 仅 sim 臂：stall/hardware_error/no_response/clear
    c.safety(arm)                           # -> {"stalls","motor_errors","torque","limits"}
    c.stalled(arm) / c.stalled(arm, 3)      # 是否（该电机）处于 stall 状态
    这些方法在旧版 armd（0.1.0）上会返回 ArmdError("unknown_method")；safety() 对缺失字段退化为 {}。

自动重连（默认开启）：
    c = ArmdClient("/tmp/armd.sock", auto_reconnect=True, reconnect_interval=1.0,
                   on_disconnect=lambda reason: print("断线", reason),
                   on_reconnect=lambda: print("已重连"))
    - daemon 断开后后台重连（指数退避，上限约 5s）；
    - 自动重订阅：subscribe 时记住 arms/rate_hz，重连后重新注册，回调保持不变；
    - 断线瞬间在途的 request() 在重连成功后自动重试一次（读/写 RPC 均如此，注意写 RPC 可能被重复执行）；
    - 显式 close() 后不再重连；auto_reconnect=False 时断线直接抛 ArmdError("closed")。
    注意：Unix socket 的 connect 失败（daemon 未启动）在构造时仍会抛 OSError，与旧版一致。

协议：Unix socket 上的 NDJSON，详见 armd/docs/DESIGN.md。
"""
from __future__ import annotations

import itertools
import json
import os
import socket
import threading
import time
from typing import Any, Callable, Dict, Iterable, List, Optional

DEFAULT_SOCKET = "/tmp/armd.sock"

#: 自动重连的退避上限（秒）
RECONNECT_MAX_INTERVAL = 5.0

#: 等待重连时可被关闭/重连事件唤醒的最长轮询间隔（秒）
_WAIT_SLICE = 0.05


class ArmdError(Exception):
    """服务端返回的错误（code/message），或客户端连接已关闭（code="closed"）。"""

    def __init__(self, code: str, message: str):
        super().__init__(f"{code}: {message}")
        self.code = code
        self.message = message


class _ConnectionLost(OSError):
    """内部异常：底层 socket 断开/未连接。request() 负责转成重连+重试或 ArmdError。"""


class ArmdClient:
    def __init__(self, path: str = DEFAULT_SOCKET, default_timeout: float = 2.0,
                 token: Optional[str] = None,
                 auto_reconnect: bool = True,
                 reconnect_interval: float = 1.0,
                 on_disconnect: Optional[Callable[..., Any]] = None,
                 on_reconnect: Optional[Callable[..., Any]] = None):
        """path 支持 `/tmp/armd.sock`（本机 Unix socket）或 `tcp://host:8771`（ARM 下位机等网络场景）。

        auto_reconnect=True（默认）时，连接断开后后台自动重连并重订阅，详见模块 docstring。
        """
        self.path = path
        self.default_timeout = default_timeout
        self.token = token or os.environ.get("ARMD_TOKEN") or None
        self.auto_reconnect = bool(auto_reconnect)
        self.reconnect_interval = max(0.01, float(reconnect_interval))
        self.on_disconnect = on_disconnect
        self.on_reconnect = on_reconnect
        self._host: Optional[str] = None
        self._port: Optional[int] = None
        if path.startswith("tcp://"):
            host, _, port = path[len("tcp://"):].rpartition(":")
            if not host or not port.isdigit():
                raise ArmdError("bad_request", f"非法 TCP 地址 '{path}'（应为 tcp://host:port）")
            self._host, self._port = host, int(port)

        # 连接状态（_conn_lock 保护 _sock/_reader_f/_gen/_closed/_connected_ev）
        self._sock: Optional[socket.socket] = None
        self._reader_f: Optional[Any] = None
        self._reader: Optional[threading.Thread] = None
        self._gen = 0                      # 连接代号：每次成功/失效连接++，用于丢弃过期读线程事件
        self._closed = False
        self._closed_ev = threading.Event()
        self._connected_ev = threading.Event()
        self._conn_lock = threading.RLock()
        self._write_lock = threading.Lock()
        self._initializing = False

        # 请求/响应
        self._pending: Dict[int, dict] = {}
        self._pending_lock = threading.Lock()
        self._ids = itertools.count(1)

        # 订阅：_subs 是「服务端 sub id -> callback」（读线程用，重订阅时更新）；
        # _sub_specs 是「本地稳定句柄 -> {callback, arms, rate_hz, server_id}」。
        self._subs: Dict[int, Callable[[dict], None]] = {}
        self._sub_by_server: Dict[int, int] = {}
        self._sub_specs: Dict[int, dict] = {}
        self._local_sub_ids = itertools.count(1)
        self._subs_lock = threading.RLock()

        # 重连线程
        self._reconnect_thread: Optional[threading.Thread] = None
        self._reconnect_lock = threading.Lock()

        self._initializing = True
        try:
            self._establish_connection(initial=True)
        finally:
            self._initializing = False
        if not self._connected_ev.is_set() and not self._closed:
            self._ensure_reconnect()

    # ------------------------------------------------------------ 连接管理
    def _open_socket(self) -> tuple:
        """新建一条底层连接（不安装到 self 上）。失败抛 OSError。"""
        if self._host is not None:
            sock = socket.create_connection((self._host, self._port), timeout=10.0)
            sock.settimeout(None)
            try:
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except OSError:
                pass
        else:
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            try:
                sock.connect(self.path)
            except BaseException:
                try:
                    sock.close()
                except OSError:
                    pass
                raise
        try:
            reader = sock.makefile("rb")
        except BaseException:
            try:
                sock.close()
            except OSError:
                pass
            raise
        return sock, reader

    @staticmethod
    def _teardown(sock: Optional[socket.socket], reader: Optional[Any]) -> None:
        """关闭一条连接（先 shutdown 唤醒阻塞读，再关文件对象/socket）。幂等。"""
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        if reader is not None:
            try:
                reader.close()
            except Exception:
                pass
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

    def _establish_connection(self, initial: bool = False) -> None:
        """建立/恢复一条连接，可抛 OSError/ArmdError。失败时保证不残留旧 handle。"""
        if self._closed:
            raise ArmdError("closed", "client closed")
        sock: Optional[socket.socket] = None
        reader: Optional[Any] = None
        gen: Optional[int] = None
        try:
            sock, reader = self._open_socket()
            with self._conn_lock:
                if self._closed:
                    raise ArmdError("closed", "client closed")
                self._gen += 1
                gen = self._gen
                self._sock = sock
                self._reader_f = reader
            self._reader = threading.Thread(
                target=self._read_loop, args=(reader, gen), name="armd-reader", daemon=True
            )
            self._reader.start()
            if self.token:
                self._request_raw("hello", {"token": self.token}, self.default_timeout)
            self._resubscribe_all()
            with self._conn_lock:
                if self._closed or gen != self._gen:
                    raise _ConnectionLost("连接在建立期间被关闭/替换")
                self._connected_ev.set()
        except BaseException:
            if gen is not None:
                self._drop_connection(gen, "连接建立失败", notify=False, schedule_reconnect=False)
            else:
                self._teardown(sock, reader)
            raise
        if not initial:
            self._call_cb(self.on_reconnect)

    def _drop_connection(self, gen: int, reason: str, notify: bool = True,
                         schedule_reconnect: bool = True) -> bool:
        """把当前连接标记为失效：清 pending、关 handle、起重连线程、通知 on_disconnect。

        只有 gen 仍是当前连接代号才动作（幂等/防重复处理）；返回是否真的做了处理。
        """
        with self._conn_lock:
            if gen != self._gen:
                return False
            self._gen += 1
            sock, reader = self._sock, self._reader_f
            self._sock = None
            self._reader_f = None
            self._connected_ev.clear()
        with self._subs_lock:
            self._subs.clear()
            self._sub_by_server.clear()
        self._fail_pending_lost()
        self._teardown(sock, reader)
        if schedule_reconnect:
            self._ensure_reconnect()
        if notify:
            self._call_cb(self.on_disconnect, reason)
        return True

    def _ensure_reconnect(self) -> None:
        if self._closed or not self.auto_reconnect:
            return
        with self._reconnect_lock:
            t = self._reconnect_thread
            if t is not None and t.is_alive():
                return
            t = threading.Thread(target=self._reconnect_loop, name="armd-reconnect", daemon=True)
            self._reconnect_thread = t
        t.start()

    def _reconnect_loop(self) -> None:
        attempt = 0
        try:
            while True:
                if self._closed or not self.auto_reconnect:
                    return
                cap = max(RECONNECT_MAX_INTERVAL, self.reconnect_interval)
                delay = min(self.reconnect_interval * (2.0 ** min(attempt, 8)), cap)
                if self._closed_ev.wait(delay):
                    return
                if self._closed or not self.auto_reconnect:
                    return
                try:
                    self._establish_connection(initial=False)
                    return
                except Exception:
                    attempt += 1
        finally:
            with self._reconnect_lock:
                self._reconnect_thread = None

    @staticmethod
    def _call_cb(callback: Optional[Callable[..., Any]], *args: Any) -> None:
        """安全调用用户回调（回调异常不能影响客户端内部状态）。"""
        if callback is None:
            return
        try:
            callback(*args)
        except TypeError:
            # 兼容无参回调：先按带参调用，TypeError 时退回无参
            try:
                callback()
            except Exception:
                pass
        except Exception:
            pass

    # ------------------------------------------------------------ 底层收发
    def _read_loop(self, reader_f: Any, gen: int) -> None:
        try:
            while True:
                try:
                    line = reader_f.readline()
                except (OSError, ValueError):
                    break
                if not line:
                    break
                try:
                    msg = json.loads(line)
                except (json.JSONDecodeError, UnicodeDecodeError):
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
                if slot is None:
                    continue
                if msg.get("error"):
                    slot["error"] = msg["error"]
                else:
                    slot["result"] = msg.get("result")
                slot["ev"].set()
        finally:
            self._drop_connection(gen, "连接断开")

    def _send(self, msg: dict) -> None:
        data = (json.dumps(msg, separators=(",", ":")) + "\n").encode()
        with self._write_lock:
            with self._conn_lock:
                sock = self._sock
            if sock is None:
                raise _ConnectionLost("未连接")
            sock.sendall(data)

    def _new_slot(self) -> dict:
        return {"result": None, "error": None, "ev": threading.Event(), "lost": False}

    def _fail_pending_lost(self) -> None:
        """连接失效：唤醒所有等待者并标记 lost（request() 据此决定重试/报错）。"""
        with self._pending_lock:
            slots = list(self._pending.values())
            self._pending.clear()
        for slot in slots:
            slot["lost"] = True
            slot["ev"].set()

    def _request_raw(self, method: str, params: dict, timeout: float) -> Any:
        """单次请求，不做重连/重试（重连线程内部用：hello / 重订阅）。"""
        if self._closed:
            raise ArmdError("closed", "client closed")
        rid = next(self._ids)
        slot = self._new_slot()
        with self._pending_lock:
            self._pending[rid] = slot
        try:
            self._send({"id": rid, "method": method, "params": params})
        except OSError as e:
            with self._pending_lock:
                self._pending.pop(rid, None)
            raise _ConnectionLost(str(e)) from e
        if not slot["ev"].wait(timeout):
            with self._pending_lock:
                self._pending.pop(rid, None)
            raise ArmdError("timeout", f"{method} 超时")
        if slot["lost"]:
            raise _ConnectionLost("连接断开")
        if slot["error"]:
            e = slot["error"]
            raise ArmdError(e.get("code", "error"), e.get("message", ""))
        return slot["result"]

    def _await_connected(self, deadline: float) -> bool:
        """等待连接可用（死线前）。返回 False 表示超时/已关闭/不会重连。"""
        while True:
            if self._closed:
                return False
            if self._connected_ev.is_set():
                return True
            if not self.auto_reconnect:
                return False
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            self._connected_ev.wait(min(remaining, _WAIT_SLICE))

    def _attempt_request(self, method: str, params: dict, timeout: float) -> Any:
        """一次请求尝试；连接断开时抛 _ConnectionLost。"""
        rid = next(self._ids)
        slot = self._new_slot()
        with self._pending_lock:
            self._pending[rid] = slot
        sock = None
        gen = None
        try:
            data = (json.dumps({"id": rid, "method": method, "params": params},
                               separators=(",", ":")) + "\n").encode()
            with self._write_lock:
                with self._conn_lock:
                    sock = self._sock
                    gen = self._gen
                if sock is None:
                    raise _ConnectionLost("未连接")
                sock.sendall(data)
        except OSError as e:
            with self._pending_lock:
                self._pending.pop(rid, None)
            if sock is not None and gen is not None:
                self._drop_connection(gen, f"发送失败：{e}")
            raise _ConnectionLost(str(e)) from e
        if not slot["ev"].wait(timeout):
            with self._pending_lock:
                self._pending.pop(rid, None)
            raise ArmdError("timeout", f"{method} 超时")
        if slot["lost"]:
            raise _ConnectionLost("连接断开")
        if slot["error"]:
            e = slot["error"]
            raise ArmdError(e.get("code", "error"), e.get("message", ""))
        return slot["result"]

    def request(self, method: str, timeout: Optional[float] = None, **params: Any) -> Any:
        """发一条 RPC 请求。

        断线时：auto_reconnect=False -> ArmdError("closed")；否则等待后台重连，重连成功后
        在途/待发请求自动重发一次（最多一次），仍失败则 ArmdError("closed")。
        """
        timeout = float(self.default_timeout if timeout is None else timeout)
        if self._closed:
            raise ArmdError("closed", "client closed")
        if not self._connected_ev.is_set():
            if not self._await_connected(time.monotonic() + timeout):
                raise ArmdError("closed", "连接已断开（等待重连超时）")
        try:
            return self._attempt_request(method, params, timeout)
        except _ConnectionLost as e:
            if not self.auto_reconnect or self._closed:
                raise ArmdError("closed", f"连接已断开：{e}") from e
            if not self._await_connected(time.monotonic() + timeout):
                raise ArmdError("closed", f"连接已断开且重连失败：{e}") from e
            try:
                return self._attempt_request(method, params, timeout)
            except _ConnectionLost as e2:
                raise ArmdError("closed", f"连接已断开：{e2}") from e2

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

    # ------------------------------------------------------------ 安全层 API（v0.2.0）
    @staticmethod
    def _motors(motors: Optional[Iterable[int]]) -> Optional[List[int]]:
        return None if motors is None else [int(m) for m in motors]

    def reboot(self, arm: str, motors: Optional[Iterable[int]] = None) -> dict:
        """Dynamixel Reboot(0x08)，清除 latched 硬件错误（如 0x80）。默认全部电机。

        返回 {ok, rebooted, failed, errors, torque, limits, warnings}；服务端等待最长 5s。
        """
        params: dict = {"arm": arm}
        m = self._motors(motors)
        if m is not None:
            params["motors"] = m
        return self.request("arm.reboot", timeout=8.0, **params)

    def clear_error(self, arm: str, motors: Optional[Iterable[int]] = None) -> dict:
        """清 armd 侧 motor_errors/stalls 标记，并读 Hardware_Error_Status(70)。

        返回 {ok, cleared, hardware_error_status, warnings}；latched 错误仍需 reboot()。
        """
        params: dict = {"arm": arm}
        m = self._motors(motors)
        if m is not None:
            params["motors"] = m
        return self.request("arm.clear_error", timeout=5.0, **params)

    def torque_off(self, arm: str, motors: Optional[Iterable[int]] = None) -> dict:
        """按电机 torque OFF 的便捷方法（等价 torque(arm, False, motors)）。"""
        params: dict = {"arm": arm}
        m = self._motors(motors)
        if m is not None:
            params["motors"] = m
        return self.request("arm.torque_off", **params)

    def sim_inject(self, arm: str, kind: str, motors: Optional[Iterable[int]] = None,
                   enable: bool = True) -> dict:
        """仿真故障注入（仅 sim 臂，否则 ArmdError("not_sim")）。

        kind: "stall" / "hardware_error" / "no_response" / "clear"。
        """
        params: dict = {"arm": arm, "kind": kind, "enable": bool(enable)}
        m = self._motors(motors)
        if m is not None:
            params["motors"] = m
        return self.request("arm.sim.inject", **params)

    def safety(self, arm: str) -> dict:
        """安全层状态便捷视图（数据来自 arm.state；旧版 daemon 缺失字段时为 {}）。

        返回 {"stalls": {id: {...}}, "motor_errors": {id: {...}},
              "torque": {id: bool}, "limits": {id: {"min","max"}}}
        """
        st = self.state(arm) or {}
        return {
            "stalls": st.get("stalls") or {},
            "motor_errors": st.get("motor_errors") or {},
            "torque": st.get("torque") or {},
            "limits": st.get("limits") or {},
        }

    def stalled(self, arm: str, motor: Optional[int] = None) -> bool:
        """是否存在 stall 标记；给定 motor 时只判断该电机（0.2.0 之前恒为 False）。"""
        stalls = self.safety(arm)["stalls"]
        if motor is None:
            return bool(stalls)
        return bool(stalls.get(str(motor), stalls.get(motor)))

    # ------------------------------------------------------------ 发布订阅
    def subscribe(
        self,
        callback: Callable[[dict], None],
        arms: Optional[Iterable[str]] = None,
        rate_hz: float = 60.0,
    ) -> int:
        """订阅状态流，返回本地稳定订阅句柄（重连后不变）。

        arms/rate_hz 会被记住；连接断开自动重连后会用同样的参数重新订阅，
        推送帧仍会送到同一个 callback。
        """
        arm_list = None if arms is None else [str(a) for a in arms]
        local_id = next(self._local_sub_ids)
        spec = {"callback": callback, "arms": arm_list, "rate_hz": float(rate_hz),
                "server_id": None}
        with self._subs_lock:
            self._sub_specs[local_id] = spec
        with self._conn_lock:
            gen = self._gen
        try:
            params: dict = {"rate_hz": rate_hz}
            if arm_list is not None:
                params["arms"] = arm_list
            res = self.request("sub.subscribe", **params)
        except BaseException:
            with self._subs_lock:
                self._sub_specs.pop(local_id, None)
            raise
        server_id = res.get("sub_id")
        with self._conn_lock:
            same_connection = gen == self._gen and self._connected_ev.is_set()
        if same_connection:
            with self._subs_lock:
                if local_id in self._sub_specs:
                    spec["server_id"] = server_id
                    self._subs[server_id] = callback
                    self._sub_by_server[server_id] = local_id
        # 若在注册期间断线：spec 已存在，重连线程的 _resubscribe_all 会补注册
        return local_id

    def unsubscribe(self, sub_id: int) -> dict:
        """取消订阅。sub_id 用 subscribe() 返回的本地句柄（兼容旧版服务端 sub id）。"""
        target = sub_id
        with self._subs_lock:
            spec = self._sub_specs.pop(sub_id, None)
            if spec is not None:
                server_id = spec.get("server_id")
                if server_id is not None:
                    self._subs.pop(server_id, None)
                    self._sub_by_server.pop(server_id, None)
                    target = server_id
            else:
                local = self._sub_by_server.pop(sub_id, None)
                if local is not None:
                    self._sub_specs.pop(local, None)
                    self._subs.pop(sub_id, None)
                    target = sub_id
        return self.request("sub.unsubscribe", sub_id=target)

    def set_sub_rate(self, sub_id: int, rate_hz: float) -> dict:
        with self._subs_lock:
            spec = self._sub_specs.get(sub_id)
            if spec is not None:
                spec["rate_hz"] = float(rate_hz)
        if spec is None:
            return self.request("sub.rate", sub_id=sub_id, rate_hz=rate_hz)
        # 重连期间 server_id 可能暂时未知：等重连+重订阅完成后再改
        deadline = time.monotonic() + self.default_timeout
        target = None
        while target is None:
            with self._subs_lock:
                target = spec.get("server_id")
            if target is not None and self._connected_ev.is_set():
                break
            target = None
            if not self._await_connected(deadline):
                raise ArmdError("closed", "连接已断开（等待重连超时）")
        return self.request("sub.rate", sub_id=target, rate_hz=rate_hz)

    def _resubscribe_all(self) -> None:
        """重连成功后把保存的订阅全部重新注册到新连接上（读线程已启动）。"""
        with self._subs_lock:
            specs = list(self._sub_specs.items())
        for local_id, spec in specs:
            params: dict = {"rate_hz": spec["rate_hz"]}
            if spec["arms"] is not None:
                params["arms"] = list(spec["arms"])
            try:
                res = self._request_raw("sub.subscribe", params, self.default_timeout)
            except ArmdError:
                continue  # 单个订阅失败不影响其他订阅
            server_id = res.get("sub_id")
            with self._subs_lock:
                if local_id not in self._sub_specs:
                    continue  # 重连期间已被 unsubscribe
                old_server = spec.get("server_id")
                if old_server is not None:
                    self._subs.pop(old_server, None)
                    self._sub_by_server.pop(old_server, None)
                spec["server_id"] = server_id
                self._subs[server_id] = spec["callback"]
                self._sub_by_server[server_id] = local_id

    def close(self) -> None:
        """显式关闭：禁用重连、唤醒等待者、关闭 socket。可重复调用。"""
        with self._conn_lock:
            if self._closed:
                return
            self._closed = True
            self._closed_ev.set()
            self._gen += 1
            sock, reader = self._sock, self._reader_f
            self._sock = None
            self._reader_f = None
            self._connected_ev.clear()
        with self._subs_lock:
            self._subs.clear()
            self._sub_by_server.clear()
        self._fail_pending_lost()
        self._teardown(sock, reader)


class StateStream:
    """订阅多个臂，保存最新状态快照（线程安全），供轮询式代码使用。

    stream = StateStream(client, ["koch_v1_1:follower"], rate_hz=60)
    stream.latest("koch_v1_1:follower")  -> {"joints": {...}, "seq": ...} | None
    stream.close()

    ArmdClient 自动重连时订阅会自动恢复，latest()/frames() 无需重建。
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
def _parse_motors(text: Optional[str]) -> Optional[List[int]]:
    """'3,4' -> [3, 4]；None/空 -> None。"""
    if not text:
        return None
    return [int(x) for x in text.replace(" ", "").split(",") if x]


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
    # 安全层（v0.2.0）
    p = sub.add_parser("reboot", help="Dynamixel Reboot 清 latched 硬件错误")
    p.add_argument("arm"); p.add_argument("--motors", default=None, help="逗号分隔电机 id，如 3,4")
    p = sub.add_parser("clear-error", help="清 armd 侧 error/stall 标记")
    p.add_argument("arm"); p.add_argument("--motors", default=None, help="逗号分隔电机 id，如 3,4")
    p = sub.add_parser("torque-off", help="按电机 torque OFF")
    p.add_argument("arm"); p.add_argument("--motors", default=None, help="逗号分隔电机 id，如 3,4")
    p = sub.add_parser("safety", help="查看 stalls/motor_errors/torque/limits")
    p.add_argument("arm")
    p = sub.add_parser("sim-inject", help="仿真故障注入（仅 sim 臂）")
    p.add_argument("arm"); p.add_argument("kind", help="stall/hardware_error/no_response/clear")
    p.add_argument("--motors", default=None, help="逗号分隔电机 id，如 3,4")
    p.add_argument("--disable", action="store_true", help="enable=false（清除注入）")
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
        elif args.cmd == "reboot":
            print(json.dumps(c.reboot(args.arm, _parse_motors(args.motors)), ensure_ascii=False, indent=2))
        elif args.cmd == "clear-error":
            print(json.dumps(c.clear_error(args.arm, _parse_motors(args.motors)), ensure_ascii=False, indent=2))
        elif args.cmd == "torque-off":
            print(json.dumps(c.torque_off(args.arm, _parse_motors(args.motors)), ensure_ascii=False, indent=2))
        elif args.cmd == "safety":
            print(json.dumps(c.safety(args.arm), ensure_ascii=False, indent=2))
        elif args.cmd == "sim-inject":
            print(json.dumps(c.sim_inject(args.arm, args.kind, _parse_motors(args.motors),
                                          enable=not args.disable), ensure_ascii=False, indent=2))
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
