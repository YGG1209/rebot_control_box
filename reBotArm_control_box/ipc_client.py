from __future__ import annotations

import math
import socket
import threading
from collections.abc import Sequence
from typing import Any, Mapping

from reBotArm_control_box.ipc_protocol import (
    DEFAULT_IPC_SOCKET_PATH,
    IPC_MAX_PACKET_SIZE,
    IPCMessageType,
    IPCProtocolError,
    IPCSnapshot,
    decode_packet,
    encode_packet,
    make_servoj_payload,
    parse_snapshot_payload,
    require_fields,
    validate_uint,
)


class IPCClientError(RuntimeError):
    pass


class IPCRemoteError(IPCClientError):
    # 初始化远端错误
    def __init__(self, code: str, message: str) -> None:
        super().__init__(f"{code}: {message}")
        self.code = code
        self.message = message


class ControlIPCClient:
    # 初始化持久IPC客户端
    def __init__(
        self,
        socket_path: str = DEFAULT_IPC_SOCKET_PATH,
        *,
        timeout_s: float = 1.0,
    ) -> None:
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError("timeout_s must be finite and positive")
        self._socket_path = socket_path
        self._timeout_s = float(timeout_s)
        self._socket: socket.socket | None = None
        self._request_id = 1
        self._lock = threading.Lock()

    # 返回IPC连接状态
    @property
    def connected(self) -> bool:
        return self._socket is not None

    # 连接控制守护进程
    def connect(self) -> None:
        with self._lock:
            if self._socket is not None:
                return
            connection = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            connection.settimeout(self._timeout_s)
            try:
                connection.connect(self._socket_path)
            except OSError as error:
                connection.close()
                raise IPCClientError(
                    f"cannot connect to IPC server: {self._socket_path}"
                ) from error
            self._socket = connection

    # 关闭IPC连接
    def close(self) -> None:
        with self._lock:
            connection = self._socket
            self._socket = None
            if connection is not None:
                connection.close()

    # 进入上下文并连接
    def __enter__(self) -> ControlIPCClient:
        self.connect()
        return self

    # 退出上下文并关闭连接
    def __exit__(self, *_: object) -> None:
        self.close()

    # 检查IPC服务是否响应
    def ping(self) -> int:
        response = self._request(
            IPCMessageType.PING,
            {},
            IPCMessageType.PONG,
        )
        require_fields(response, {"server_time_ns"})
        value = response["server_time_ns"]
        return validate_uint("server_time_ns", value, 64)

    # 提交最新servoJ命令
    def submit_servoj(
        self,
        sequence: int,
        target_q: Sequence[float],
        speed_limits: float | Sequence[float],
    ) -> bool:
        response = self._request(
            IPCMessageType.SUBMIT_SERVOJ,
            make_servoj_payload(sequence, target_q, speed_limits),
            IPCMessageType.COMMAND_ACK,
        )
        require_fields(response, {"accepted"})
        if not isinstance(response["accepted"], bool):
            raise IPCProtocolError("accepted must be a boolean")
        return response["accepted"]

    # 暂停并保持实际关节位置
    def pause(self) -> None:
        response = self._request(
            IPCMessageType.PAUSE,
            {},
            IPCMessageType.PAUSE_ACK,
        )
        require_fields(response, {"accepted"})
        if response["accepted"] is not True:
            raise IPCProtocolError("pause acknowledgement was not accepted")

    # 重置控制会话和命令序号
    def reset_session(self) -> None:
        response = self._request(
            IPCMessageType.RESET_SESSION,
            {},
            IPCMessageType.RESET_SESSION_ACK,
        )
        require_fields(response, {"accepted"})
        if response["accepted"] is not True:
            raise IPCProtocolError("reset acknowledgement was not accepted")

    # 读取控制守护进程快照
    def get_snapshot(self) -> IPCSnapshot:
        response = self._request(
            IPCMessageType.GET_SNAPSHOT,
            {},
            IPCMessageType.SNAPSHOT,
        )
        return parse_snapshot_payload(response)

    # 完成一次串行请求响应
    def _request(
        self,
        message_type: IPCMessageType,
        payload: Mapping[str, Any],
        expected_type: IPCMessageType,
    ) -> dict[str, Any]:
        with self._lock:
            connection = self._require_socket()
            request_id = self._next_request_id()
            packet = encode_packet(message_type, request_id, payload)

            try:
                sent = connection.send(packet)
                if sent != len(packet):
                    raise OSError("IPC packet was only partially sent")
                response_data = connection.recv(IPC_MAX_PACKET_SIZE)
                if not response_data:
                    raise OSError("IPC server closed the connection")
            except OSError as error:
                self._socket = None
                connection.close()
                raise IPCClientError("IPC request failed") from error

            response = decode_packet(response_data)
            if response.request_id != request_id:
                raise IPCProtocolError(
                    "IPC response request_id does not match the request"
                )
            if response.message_type is IPCMessageType.ERROR:
                require_fields(response.payload, {"code", "message"})
                raise IPCRemoteError(
                    str(response.payload["code"]),
                    str(response.payload["message"]),
                )
            if response.message_type is not expected_type:
                raise IPCProtocolError(
                    f"expected {expected_type.name}, got "
                    f"{response.message_type.name}"
                )
            return response.payload

    # 返回已连接Socket
    def _require_socket(self) -> socket.socket:
        if self._socket is None:
            raise IPCClientError("IPC client is not connected")
        return self._socket

    # 分配并回绕请求编号
    def _next_request_id(self) -> int:
        request_id = self._request_id
        self._request_id = (self._request_id + 1) & 0xFFFFFFFF
        return request_id
