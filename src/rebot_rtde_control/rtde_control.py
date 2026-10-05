from __future__ import annotations

import math
import threading
from collections.abc import Sequence

from ._exceptions import RTDEConnectionError
from ._network_client import DEFAULT_CONTROLLER_PORT, RebotNetworkClient


DEFAULT_SPEED_RAD_S = 0.1
DEFAULT_LEASE_MS = 1000
DEFAULT_STATE_TIMEOUT_S = 3.0


class RTDEControl:
    """reBot机械臂控制接口。"""

    # 初始化控制接口
    def __init__(
        self,
        *,
        default_speed: float = DEFAULT_SPEED_RAD_S,
        state_timeout: float = DEFAULT_STATE_TIMEOUT_S,
    ) -> None:
        self._default_speed = self._positive_number(
            "default_speed",
            default_speed,
        )
        self._state_timeout = self._positive_number(
            "state_timeout",
            state_timeout,
        )
        self._lock = threading.RLock()
        self._client: RebotNetworkClient | None = None
        self._endpoint: tuple[str, int] | None = None

    # 连接控制箱并启动控制会话
    def connect(
        self,
        ip: str,
        port: int = DEFAULT_CONTROLLER_PORT,
    ) -> bool:
        with self._lock:
            endpoint = (ip, port)
            current = self._client
            if current is not None and current.connected:
                if self._endpoint != endpoint:
                    raise RTDEConnectionError(
                        "already connected to another control box"
                    )
                if current.started and current.has_control:
                    return True

            self._close_client()
            client = RebotNetworkClient(ip, port, client_name="rebot-rtde")
            self._client = client
            self._endpoint = endpoint

            try:
                client.connect()
                client.setup_outputs()
                client.setup_inputs()
                client.acquire_control(
                    requested_lease_ms=DEFAULT_LEASE_MS,
                )
                client.start()
                client.wait_for_state(timeout_s=self._state_timeout)
            except BaseException:
                self._close_client()
                raise
            return True

    # 返回当前连接状态
    def isConnected(self) -> bool:
        with self._lock:
            return self._client is not None and self._client.connected

    # 发送一帧servoJ目标
    def servoJ(
        self,
        q: Sequence[float],
        speed: float | Sequence[float] | None = None,
    ) -> bool:
        with self._lock:
            client = self._require_active_client()
            speed_limits = self._default_speed if speed is None else speed
            client.send_servoj(q, speed_limits)
            return True

    # 暂停控制并保持当前位置
    def pause(self) -> bool:
        with self._lock:
            client = self._client
            if client is None or not client.connected:
                return False
            if client.started:
                client.pause()
            return True

    # 安全释放控制并断开连接
    def disconnect(self) -> None:
        with self._lock:
            self._close_client()

    # 返回已经启动的内部客户端
    def _require_active_client(self) -> RebotNetworkClient:
        client = self._client
        if client is None or not client.connected:
            raise RTDEConnectionError("control box is not connected")
        if not client.started or not client.has_control:
            raise RTDEConnectionError("control session is not active")
        return client

    # 关闭内部客户端
    def _close_client(self) -> None:
        client = self._client
        self._client = None
        self._endpoint = None
        if client is None:
            return

        try:
            if client.connected and client.started:
                client.pause()
        except Exception:
            pass

        try:
            if client.connected and client.has_control:
                client.release_control()
        except Exception:
            pass

        client.close()

    # 校验有限正数
    @staticmethod
    def _positive_number(name: str, value: float) -> float:
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            or float(value) <= 0.0
        ):
            raise ValueError(f"{name} must be finite and positive")
        return float(value)
