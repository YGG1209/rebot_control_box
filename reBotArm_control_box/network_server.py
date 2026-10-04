from __future__ import annotations

import argparse
import errno
import logging
import math
import secrets
import socket
import threading
import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Any

from reBotArm_control_box.ipc_client import (
    ControlIPCClient,
    IPCClientError,
    IPCRemoteError,
)
from reBotArm_control_box.ipc_protocol import (
    DEFAULT_IPC_SOCKET_PATH,
    IPCProtocolError,
)
from reBotArm_control_box.protocol import (
    PROTOCOL_VERSION,
    SUPPORTED_VERSIONS,
    ControlMode,
    Frame,
    FrameDecoder,
    INPUT_FIELDS,
    MessageType,
    OUTPUT_FIELDS,
    ProtocolError,
    Recipe,
    RobotMode,
    SafetyState,
    decode_json_payload,
    encode_frame,
    encode_json_frame,
    is_newer_sequence,
    pack_recipe_payload,
    recipe_types,
    resolve_recipe,
    unpack_recipe_payload,
)


LOGGER = logging.getLogger("rebot-network-server")

DEFAULT_SERVER_HOST = "192.168.100.2"
DEFAULT_SERVER_PORT = 5000
DEFAULT_HANDSHAKE_TIMEOUT_S = 5.0
DEFAULT_CLIENT_IDLE_TIMEOUT_S = 1.0
RECEIVE_CHUNK_SIZE = 64 * 1024
MAX_CLIENT_FRAMES_PER_SECOND = 240
THREAD_JOIN_TIMEOUT_S = 3.0
MIN_RECIPE_FREQUENCY_HZ = 1.0
MAX_RECIPE_FREQUENCY_HZ = 60.0
INPUT_RECIPE_ID = 1
OUTPUT_RECIPE_ID = 2
MIN_CONTROL_LEASE_MS = 250
MAX_CONTROL_LEASE_MS = 5000
DEFAULT_CONTROL_LEASE_MS = 1000
OUTPUT_THREAD_JOIN_TIMEOUT_S = 1.0

SUPPORTED_INPUT_FIELD_NAMES = frozenset(
    {
        "control_mode",
        "target_q",
        "speed_limits",
    }
)
SUPPORTED_OUTPUT_FIELD_NAMES = frozenset(
    {
        "controller_time_ns",
        "actual_q",
        "actual_qd",
        "actual_tau",
        "target_q",
        "robot_mode",
        "safety_state",
        "fault_code",
        "last_input_sequence",
    }
)

CLIENT_RESPONSE_TYPES = {
    MessageType.HELLO_ACK,
    MessageType.SETUP_INPUTS_ACK,
    MessageType.SETUP_OUTPUTS_ACK,
    MessageType.ACQUIRE_CONTROL_ACK,
    MessageType.RELEASE_CONTROL_ACK,
    MessageType.START_ACK,
    MessageType.PAUSE_ACK,
    MessageType.OUTPUT_DATA,
    MessageType.HEARTBEAT_ACK,
    MessageType.ERROR,
}


class SessionState(Enum):
    WAIT_HELLO = auto()
    NEGOTIATED = auto()
    CLOSED = auto()


class FrameResult(Enum):
    ACCEPTED = auto()
    REJECTED = auto()
    CLOSE = auto()


@dataclass(frozen=True, slots=True)
class ConfiguredRecipe:
    recipe_id: int
    frequency_hz: float
    recipe: Recipe


class SessionError(ProtocolError):
    # 初始化带协议错误码的会话异常
    def __init__(
        self,
        code: str,
        message: str,
        *,
        close_connection: bool = True,
    ) -> None:
        super().__init__(message)
        self.code = code
        self.close_connection = close_connection


class NetworkSession:
    # 初始化单个上层主机协议会话
    def __init__(
        self,
        connection: socket.socket,
        address: tuple[str, int],
        ipc_client: Any,
        stop_event: threading.Event,
        *,
        handshake_timeout_s: float = DEFAULT_HANDSHAKE_TIMEOUT_S,
        client_idle_timeout_s: float = DEFAULT_CLIENT_IDLE_TIMEOUT_S,
    ) -> None:
        self._connection = connection
        self._address = address
        self._ipc_client = ipc_client
        self._stop_event = stop_event
        self._handshake_timeout_s = handshake_timeout_s
        self._client_idle_timeout_s = client_idle_timeout_s
        self._decoder = FrameDecoder()
        self._state = SessionState.WAIT_HELLO
        self._last_rx_sequence: int | None = None
        self._selected_version: int | None = None
        self._client_name = ""
        self._negotiated = False
        self._next_tx_sequence = 0
        self._rate_window_started_at = time.monotonic()
        self._rate_window_frame_count = 0
        self._input_recipe: ConfiguredRecipe | None = None
        self._output_recipe: ConfiguredRecipe | None = None
        self._send_lock = threading.Lock()
        self._stream_lock = threading.Lock()
        self._output_thread: threading.Thread | None = None
        self._output_stop_event: threading.Event | None = None
        self._session_end_event = threading.Event()
        self._async_error_lock = threading.Lock()
        self._async_error: Exception | None = None
        self._started = False
        self._control_acquired = False
        self._control_session_id: int | None = None
        self._control_lease_s = 0.0
        self._control_lease_deadline = 0.0
        self._last_control_mode: ControlMode | None = None
        self._safe_reset_completed = False

    # 返回当前协议会话状态
    @property
    def state(self) -> SessionState:
        return self._state

    # 返回是否已完成版本协商
    @property
    def negotiated(self) -> bool:
        return self._negotiated

    # 返回本会话是否已经完成安全重置
    @property
    def safe_reset_completed(self) -> bool:
        return self._safe_reset_completed

    # 持续接收并处理一个TCP连接
    def run(self) -> None:
        handshake_deadline = time.monotonic() + self._handshake_timeout_s
        last_valid_frame_at = time.monotonic()

        try:
            while (
                not self._stop_event.is_set()
                and not self._session_end_event.is_set()
            ):
                now = time.monotonic()
                if (
                    self._state is SessionState.WAIT_HELLO
                    and now >= handshake_deadline
                ):
                    self._try_send_error(
                        "HANDSHAKE_TIMEOUT",
                        "HELLO was not received before the deadline",
                        0,
                    )
                    break
                if (
                    self._state is SessionState.NEGOTIATED
                    and now - last_valid_frame_at
                    >= self._client_idle_timeout_s
                ):
                    self._try_send_error(
                        "CONNECTION_TIMEOUT",
                        "no complete valid frame was received before the deadline",
                        self._last_rx_sequence or 0,
                    )
                    break
                if self._state is SessionState.NEGOTIATED:
                    self._expire_control_lease(now)

                try:
                    data = self._connection.recv(RECEIVE_CHUNK_SIZE)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if not data:
                    break

                try:
                    frames = self._decoder.feed(data)
                except ProtocolError as error:
                    self._try_send_error("INVALID_FRAME", str(error), 0)
                    break

                keep_open = True
                for frame in frames:
                    if self._stop_event.is_set():
                        keep_open = False
                        break
                    result = self._process_frame(frame)
                    if result is FrameResult.CLOSE:
                        keep_open = False
                        break
                    if result is FrameResult.ACCEPTED:
                        last_valid_frame_at = time.monotonic()
                        self._renew_control_lease(last_valid_frame_at)
                if not keep_open:
                    break
        finally:
            self._started = False
            self._stop_output_stream()
            if self._decoder.buffered_bytes:
                LOGGER.debug(
                    "TCP连接关闭时仍有%d字节未组成完整帧",
                    self._decoder.buffered_bytes,
                )
            self._state = SessionState.CLOSED

        async_error = self._take_async_error()
        if async_error is not None:
            raise async_error

    # 校验并分发一条完整协议帧
    def _process_frame(self, frame: Frame) -> FrameResult:
        try:
            self._accept_frame_rate()
            self._validate_common_frame_fields(frame)
            self._accept_rx_sequence(frame.sequence)

            if self._state is SessionState.WAIT_HELLO:
                self._handle_hello(frame)
                return FrameResult.ACCEPTED

            if frame.version != self._selected_version:
                raise SessionError(
                    "UNEXPECTED_VERSION",
                    f"session negotiated version {self._selected_version}",
                )
            if frame.message_type is MessageType.HELLO:
                raise SessionError(
                    "INVALID_STATE",
                    "HELLO is only valid as the first message",
                )
            if frame.message_type is MessageType.HEARTBEAT:
                self._handle_heartbeat(frame)
                return FrameResult.ACCEPTED
            if frame.message_type is MessageType.SETUP_INPUTS:
                self._require_not_started("SETUP_INPUTS")
                self._input_recipe = self._handle_setup_recipe(
                    frame,
                    registry=INPUT_FIELDS,
                    supported_names=SUPPORTED_INPUT_FIELD_NAMES,
                    recipe_id=INPUT_RECIPE_ID,
                    ack_type=MessageType.SETUP_INPUTS_ACK,
                )
                return FrameResult.ACCEPTED
            if frame.message_type is MessageType.SETUP_OUTPUTS:
                self._require_not_started("SETUP_OUTPUTS")
                self._output_recipe = self._handle_setup_recipe(
                    frame,
                    registry=OUTPUT_FIELDS,
                    supported_names=SUPPORTED_OUTPUT_FIELD_NAMES,
                    recipe_id=OUTPUT_RECIPE_ID,
                    ack_type=MessageType.SETUP_OUTPUTS_ACK,
                )
                return FrameResult.ACCEPTED
            if frame.message_type is MessageType.ACQUIRE_CONTROL:
                self._handle_acquire_control(frame)
                return FrameResult.ACCEPTED
            if frame.message_type is MessageType.RELEASE_CONTROL:
                self._handle_release_control(frame)
                return FrameResult.ACCEPTED
            if frame.message_type is MessageType.START:
                self._handle_start(frame)
                return FrameResult.ACCEPTED
            if frame.message_type is MessageType.PAUSE:
                self._handle_pause(frame)
                return FrameResult.ACCEPTED
            if frame.message_type is MessageType.INPUT_DATA:
                self._handle_input_data(frame)
                return FrameResult.ACCEPTED
            if frame.message_type in CLIENT_RESPONSE_TYPES:
                raise SessionError(
                    "INVALID_DIRECTION",
                    f"client cannot send {frame.message_type.name}",
                )

            self._send_error(
                "NOT_IMPLEMENTED",
                f"message type {frame.message_type.name} is not implemented yet",
                frame.sequence,
            )
            return FrameResult.REJECTED
        except SessionError as error:
            self._try_send_error(error.code, str(error), frame.sequence)
            if error.close_connection:
                return FrameResult.CLOSE
            return FrameResult.REJECTED
        except IPCRemoteError as error:
            self._try_send_error(
                error.code or "CONTROL_REJECTED",
                error.message,
                frame.sequence,
            )
            return FrameResult.REJECTED
        except (IPCClientError, IPCProtocolError) as error:
            self._ipc_client.close()
            self._try_send_error("IPC_UNAVAILABLE", str(error), frame.sequence)
            return FrameResult.CLOSE

    # 限制客户端每秒发送的完整帧数量
    def _accept_frame_rate(self) -> None:
        now = time.monotonic()
        if now - self._rate_window_started_at >= 1.0:
            self._rate_window_started_at = now
            self._rate_window_frame_count = 0
        self._rate_window_frame_count += 1
        if self._rate_window_frame_count > MAX_CLIENT_FRAMES_PER_SECOND:
            raise SessionError(
                "RATE_LIMIT_EXCEEDED",
                f"client exceeded {MAX_CLIENT_FRAMES_PER_SECOND} frames per second",
            )

    # 检查v1公共帧字段
    @staticmethod
    def _validate_common_frame_fields(frame: Frame) -> None:
        if frame.flags != 0:
            raise SessionError(
                "UNSUPPORTED_FLAGS",
                f"v1 requires flags=0, got {frame.flags}",
            )

    # 接受严格更新的客户端序号
    def _accept_rx_sequence(self, sequence: int) -> None:
        previous = self._last_rx_sequence
        if previous is not None and not is_newer_sequence(sequence, previous):
            raise SessionError(
                "INVALID_SEQUENCE",
                f"sequence {sequence} is not newer than {previous}",
            )
        self._last_rx_sequence = sequence

    # 处理HELLO并协商协议版本
    def _handle_hello(self, frame: Frame) -> None:
        if frame.message_type is not MessageType.HELLO:
            raise SessionError(
                "EXPECTED_HELLO",
                "the first message must be HELLO",
            )
        if frame.version != PROTOCOL_VERSION:
            raise SessionError(
                "UNSUPPORTED_VERSION",
                f"HELLO bootstrap version must be {PROTOCOL_VERSION}",
            )

        try:
            values = decode_json_payload(frame.payload)
        except ProtocolError as error:
            raise SessionError("INVALID_HELLO", str(error)) from error

        expected_fields = {"client_name", "supported_versions"}
        actual_fields = set(values)
        if actual_fields != expected_fields:
            missing = expected_fields - actual_fields
            unknown = actual_fields - expected_fields
            if missing:
                message = f"missing HELLO field: {sorted(missing)[0]}"
            else:
                message = f"unknown HELLO field: {sorted(unknown)[0]}"
            raise SessionError("INVALID_HELLO", message)

        client_name = values["client_name"]
        if (
            not isinstance(client_name, str)
            or not client_name.strip()
            or len(client_name) > 128
        ):
            raise SessionError(
                "INVALID_HELLO",
                "client_name must be a non-empty string up to 128 characters",
            )

        offered_versions = self._parse_supported_versions(
            values["supported_versions"]
        )
        common_versions = sorted(
            set(offered_versions).intersection(SUPPORTED_VERSIONS),
            reverse=True,
        )
        if not common_versions:
            raise SessionError(
                "UNSUPPORTED_VERSION",
                "client and server have no common protocol version",
            )

        selected_version = common_versions[0]
        self._client_name = client_name
        self._selected_version = selected_version
        self._state = SessionState.NEGOTIATED
        self._negotiated = True
        self._send_json(
            MessageType.HELLO_ACK,
            {
                "accepted": True,
                "selected_version": selected_version,
                "controller_name": "rebot-control-box",
                "related_sequence": frame.sequence,
            },
        )

    # 检查客户端支持的协议版本列表
    @staticmethod
    def _parse_supported_versions(values: Any) -> tuple[int, ...]:
        if not isinstance(values, list) or not values:
            raise SessionError(
                "INVALID_HELLO",
                "supported_versions must be a non-empty array",
            )

        versions: list[int] = []
        for value in values:
            if isinstance(value, bool) or not isinstance(value, int):
                raise SessionError(
                    "INVALID_HELLO",
                    "supported_versions must contain integers",
                )
            if value < 0 or value > 255:
                raise SessionError(
                    "INVALID_HELLO",
                    "protocol versions must fit in uint8",
                )
            versions.append(value)
        return tuple(versions)

    # 处理应用层心跳并确认IPC仍可用
    def _handle_heartbeat(self, frame: Frame) -> None:
        if frame.payload:
            raise SessionError(
                "INVALID_PAYLOAD",
                "HEARTBEAT payload must be empty",
            )
        self._ipc_client.ping()
        self._send_json(
            MessageType.HEARTBEAT_ACK,
            {"related_sequence": frame.sequence},
        )

    # 拒绝运行期间修改Recipe
    def _require_not_started(self, operation: str) -> None:
        if self._started:
            raise SessionError(
                "INVALID_STATE",
                f"{operation} is not allowed after START; send PAUSE first",
                close_connection=False,
            )

    # 校验无参数管理消息
    @staticmethod
    def _require_empty_request(frame: Frame, operation: str) -> None:
        if not frame.payload:
            return
        try:
            values = decode_json_payload(frame.payload)
        except ProtocolError as error:
            raise SessionError(
                "INVALID_PAYLOAD",
                f"invalid {operation} payload: {error}",
                close_connection=False,
            ) from error
        if values:
            raise SessionError(
                "INVALID_PAYLOAD",
                f"{operation} payload must be empty",
                close_connection=False,
            )

    # 申请当前TCP会话的独占控制权
    def _handle_acquire_control(self, frame: Frame) -> None:
        self._require_not_started("ACQUIRE_CONTROL")
        if self._input_recipe is None:
            raise SessionError(
                "INPUT_RECIPE_REQUIRED",
                "SETUP_INPUTS must succeed before ACQUIRE_CONTROL",
                close_connection=False,
            )

        try:
            values = decode_json_payload(frame.payload)
        except ProtocolError as error:
            raise SessionError(
                "INVALID_CONTROL_REQUEST",
                str(error),
                close_connection=False,
            ) from error

        expected_fields = {"mode", "requested_lease_ms"}
        actual_fields = set(values)
        if actual_fields != expected_fields:
            missing = expected_fields - actual_fields
            unknown = actual_fields - expected_fields
            detail = (
                f"missing field: {sorted(missing)[0]}"
                if missing
                else f"unknown field: {sorted(unknown)[0]}"
            )
            raise SessionError(
                "INVALID_CONTROL_REQUEST",
                detail,
                close_connection=False,
            )
        if values["mode"] != "exclusive":
            raise SessionError(
                "INVALID_CONTROL_REQUEST",
                "mode must be exclusive",
                close_connection=False,
            )

        requested_lease_ms = values["requested_lease_ms"]
        if (
            isinstance(requested_lease_ms, bool)
            or not isinstance(requested_lease_ms, int)
            or not (
                MIN_CONTROL_LEASE_MS
                <= requested_lease_ms
                <= MAX_CONTROL_LEASE_MS
            )
        ):
            raise SessionError(
                "INVALID_CONTROL_REQUEST",
                "requested_lease_ms must be an integer between "
                f"{MIN_CONTROL_LEASE_MS} and {MAX_CONTROL_LEASE_MS}",
                close_connection=False,
            )

        snapshot = self._ipc_client.get_snapshot()
        if snapshot.daemon_state not in (RobotMode.IDLE, RobotMode.RUNNING):
            raise SessionError(
                "ROBOT_NOT_READY",
                f"robot mode is {snapshot.daemon_state.name}",
                close_connection=False,
            )
        if (
            snapshot.safety_state is SafetyState.CONTROL_FAULT
            or snapshot.fault_code != 0
        ):
            raise SessionError(
                "ROBOT_FAULT",
                snapshot.fault_message or "control daemon reports a fault",
                close_connection=False,
            )

        if not self._control_acquired:
            self._ipc_client.reset_session()
            self._control_session_id = secrets.randbits(32) or 1
        self._control_acquired = True
        self._control_lease_s = requested_lease_ms / 1000.0
        self._control_lease_deadline = time.monotonic() + self._control_lease_s
        self._last_control_mode = None
        self._safe_reset_completed = False

        self._send_json(
            MessageType.ACQUIRE_CONTROL_ACK,
            {
                "accepted": True,
                "control_session_id": self._control_session_id,
                "lease_ms": requested_lease_ms,
                "related_sequence": frame.sequence,
            },
        )

    # 启动实时输入和状态输出
    def _handle_start(self, frame: Frame) -> None:
        self._require_empty_request(frame, "START")
        if self._input_recipe is None and self._output_recipe is None:
            raise SessionError(
                "RECIPE_REQUIRED",
                "configure at least one input or output recipe before START",
                close_connection=False,
            )
        if self._input_recipe is not None and not self._control_acquired:
            raise SessionError(
                "CONTROL_REQUIRED",
                "ACQUIRE_CONTROL must succeed before starting input data",
                close_connection=False,
            )

        if not self._started and self._input_recipe is not None:
            self._ipc_client.reset_session()
            self._last_control_mode = None
            self._safe_reset_completed = False

        self._started = True
        self._send_json(
            MessageType.START_ACK,
            {
                "accepted": True,
                "input_recipe_id": (
                    None
                    if self._input_recipe is None
                    else self._input_recipe.recipe_id
                ),
                "output_recipe_id": (
                    None
                    if self._output_recipe is None
                    else self._output_recipe.recipe_id
                ),
                "related_sequence": frame.sequence,
            },
        )
        self._start_output_stream()

    # 暂停实时流并保持当前关节位置
    def _handle_pause(self, frame: Frame) -> None:
        self._require_empty_request(frame, "PAUSE")
        was_started = self._started
        self._started = False
        self._stop_output_stream()
        if self._control_acquired:
            self._ipc_client.pause()
            self._last_control_mode = ControlMode.IDLE
        self._send_json(
            MessageType.PAUSE_ACK,
            {
                "accepted": True,
                "was_started": was_started,
                "related_sequence": frame.sequence,
            },
        )

    # 释放控制权并重置底层命令会话
    def _handle_release_control(self, frame: Frame) -> None:
        self._require_empty_request(frame, "RELEASE_CONTROL")
        was_acquired = self._control_acquired
        released_session_id = self._control_session_id
        self._started = False
        self._stop_output_stream()

        if was_acquired:
            self._ipc_client.reset_session()
            self._clear_control_state()
            self._safe_reset_completed = True

        self._send_json(
            MessageType.RELEASE_CONTROL_ACK,
            {
                "accepted": True,
                "released": was_acquired,
                "control_session_id": released_session_id,
                "related_sequence": frame.sequence,
            },
        )

    # 处理一帧实时关节控制数据
    def _handle_input_data(self, frame: Frame) -> None:
        if not self._started:
            raise SessionError(
                "INVALID_STATE",
                "INPUT_DATA requires START",
                close_connection=False,
            )
        if not self._control_acquired:
            raise SessionError(
                "CONTROL_REQUIRED",
                "INPUT_DATA requires active control ownership",
                close_connection=False,
            )
        configured = self._input_recipe
        if configured is None:
            raise SessionError(
                "INPUT_RECIPE_REQUIRED",
                "no input recipe is configured",
                close_connection=False,
            )

        try:
            recipe_id, values = unpack_recipe_payload(
                frame.payload,
                configured.recipe,
            )
        except ProtocolError as error:
            raise SessionError(
                "INVALID_INPUT_DATA",
                str(error),
                close_connection=False,
            ) from error
        if recipe_id != configured.recipe_id:
            raise SessionError(
                "INVALID_RECIPE_ID",
                f"expected input recipe {configured.recipe_id}, got {recipe_id}",
                close_connection=False,
            )

        try:
            control_mode = ControlMode(values["control_mode"])
        except ValueError as error:
            raise SessionError(
                "UNSUPPORTED_CONTROL_MODE",
                f"unknown control mode: {values['control_mode']}",
                close_connection=False,
            ) from error

        if control_mode is ControlMode.IDLE:
            if self._last_control_mode is not ControlMode.IDLE:
                self._ipc_client.pause()
            self._last_control_mode = ControlMode.IDLE
            return
        if control_mode is not ControlMode.SERVOJ:
            raise SessionError(
                "UNSUPPORTED_CONTROL_MODE",
                f"control mode {control_mode.name} is not available in v1",
                close_connection=False,
            )

        speed_limits = values["speed_limits"]
        if not all(speed > 0.0 for speed in speed_limits):
            raise SessionError(
                "INVALID_INPUT_DATA",
                "speed_limits values must be positive",
                close_connection=False,
            )
        accepted = self._ipc_client.submit_servoj(
            frame.sequence,
            values["target_q"],
            speed_limits,
        )
        if not accepted:
            raise SessionError(
                "STALE_INPUT",
                "control daemon rejected an old or duplicate input sequence",
                close_connection=False,
            )
        self._last_control_mode = ControlMode.SERVOJ

    # 更新活动控制租约
    def _renew_control_lease(self, now: float | None = None) -> None:
        if not self._control_acquired:
            return
        current = time.monotonic() if now is None else now
        self._control_lease_deadline = current + self._control_lease_s

    # 到期后保持机械臂并撤销控制权
    def _expire_control_lease(self, now: float) -> None:
        if (
            not self._control_acquired
            or now < self._control_lease_deadline
        ):
            return

        expired_session_id = self._control_session_id
        self._started = False
        self._stop_output_stream()
        self._ipc_client.reset_session()
        self._clear_control_state()
        self._safe_reset_completed = True
        self._send_error(
            "CONTROL_LEASE_EXPIRED",
            f"control lease expired: {expired_session_id}",
            self._last_rx_sequence or 0,
        )

    # 清除会话内控制权状态
    def _clear_control_state(self) -> None:
        self._control_acquired = False
        self._control_session_id = None
        self._control_lease_s = 0.0
        self._control_lease_deadline = 0.0
        self._last_control_mode = None

    # 配置输入或输出Recipe并返回固定会话编号
    def _handle_setup_recipe(
        self,
        frame: Frame,
        *,
        registry: dict[str, Any],
        supported_names: frozenset[str],
        recipe_id: int,
        ack_type: MessageType,
    ) -> ConfiguredRecipe:
        try:
            values = decode_json_payload(frame.payload)
            frequency_hz, field_names = self._parse_recipe_request(values)
        except ProtocolError as error:
            raise SessionError(
                "INVALID_RECIPE",
                str(error),
                close_connection=False,
            ) from error

        if recipe_id == INPUT_RECIPE_ID and frequency_hz != 60.0:
            raise SessionError(
                "INVALID_RECIPE",
                "input frequency_hz must be exactly 60",
                close_connection=False,
            )

        unsupported = [
            name
            for name in field_names
            if name in registry and name not in supported_names
        ]
        if unsupported:
            raise SessionError(
                "UNSUPPORTED_RECIPE_FIELD",
                f"field is not available yet: {unsupported[0]}",
                close_connection=False,
            )

        available_registry = {
            name: spec
            for name, spec in registry.items()
            if name in supported_names
        }
        try:
            recipe = resolve_recipe(field_names, available_registry)
        except ProtocolError as error:
            raise SessionError(
                "INVALID_RECIPE",
                str(error),
                close_connection=False,
            ) from error

        if (
            recipe_id == INPUT_RECIPE_ID
            and {name for name, _ in recipe}
            != SUPPORTED_INPUT_FIELD_NAMES
        ):
            raise SessionError(
                "INVALID_RECIPE",
                "input recipe must contain control_mode, target_q, and speed_limits",
                close_connection=False,
            )

        configured = ConfiguredRecipe(recipe_id, frequency_hz, recipe)
        self._send_json(
            ack_type,
            {
                "accepted": True,
                "recipe_id": recipe_id,
                "frequency_hz": frequency_hz,
                "fields": [name for name, _ in recipe],
                "types": recipe_types(recipe),
                "related_sequence": frame.sequence,
            },
        )
        return configured

    # 校验Recipe管理消息
    @staticmethod
    def _parse_recipe_request(
        values: dict[str, Any],
    ) -> tuple[float, list[str]]:
        expected_fields = {"frequency_hz", "fields"}
        actual_fields = set(values)
        if actual_fields != expected_fields:
            missing = expected_fields - actual_fields
            unknown = actual_fields - expected_fields
            if missing:
                raise ProtocolError(
                    f"missing recipe field: {sorted(missing)[0]}"
                )
            raise ProtocolError(
                f"unknown recipe option: {sorted(unknown)[0]}"
            )

        frequency = values["frequency_hz"]
        if (
            isinstance(frequency, bool)
            or not isinstance(frequency, (int, float))
        ):
            raise ProtocolError("frequency_hz must be a number")
        frequency_hz = float(frequency)
        if not math.isfinite(frequency_hz):
            raise ProtocolError("frequency_hz must be finite")
        if not (
            MIN_RECIPE_FREQUENCY_HZ
            <= frequency_hz
            <= MAX_RECIPE_FREQUENCY_HZ
        ):
            raise ProtocolError(
                "frequency_hz must be between "
                f"{MIN_RECIPE_FREQUENCY_HZ:g} and "
                f"{MAX_RECIPE_FREQUENCY_HZ:g}"
            )

        field_names = values["fields"]
        if not isinstance(field_names, list):
            raise ProtocolError("fields must be an array")
        if not all(isinstance(name, str) for name in field_names):
            raise ProtocolError("recipe field names must be strings")
        return frequency_hz, field_names

    # 启动所配置频率的状态发布线程
    def _start_output_stream(self) -> None:
        configured = self._output_recipe
        if configured is None:
            return

        with self._stream_lock:
            if (
                self._output_thread is not None
                and self._output_thread.is_alive()
            ):
                return
            stop_event = threading.Event()
            thread = threading.Thread(
                target=self._output_loop,
                args=(configured, stop_event),
                name=f"rebot-output-{self._address[0]}:{self._address[1]}",
                daemon=True,
            )
            self._output_stop_event = stop_event
            self._output_thread = thread
            thread.start()

    # 停止状态发布并等待发送线程退出
    def _stop_output_stream(self) -> None:
        with self._stream_lock:
            stop_event = self._output_stop_event
            thread = self._output_thread
            if stop_event is not None:
                stop_event.set()

        if (
            thread is not None
            and thread.is_alive()
            and thread is not threading.current_thread()
        ):
            thread.join(timeout=OUTPUT_THREAD_JOIN_TIMEOUT_S)

        with self._stream_lock:
            if self._output_thread is thread:
                self._output_thread = None
                self._output_stop_event = None

    # 定时读取IPC快照并发送二进制OUTPUT_DATA
    def _output_loop(
        self,
        configured: ConfiguredRecipe,
        stop_event: threading.Event,
    ) -> None:
        period_ns = max(
            1,
            int(1_000_000_000 / configured.frequency_hz),
        )
        next_cycle_ns = time.monotonic_ns()

        try:
            while (
                not stop_event.is_set()
                and not self._stop_event.is_set()
                and not self._session_end_event.is_set()
            ):
                snapshot = self._ipc_client.get_snapshot()
                if stop_event.is_set():
                    break
                values = {
                    "controller_time_ns": snapshot.timestamp_ns,
                    "actual_q": snapshot.actual_q,
                    "actual_qd": snapshot.actual_qd,
                    "actual_tau": snapshot.actual_tau,
                    "target_q": snapshot.target_q,
                    "robot_mode": int(snapshot.daemon_state),
                    "safety_state": int(snapshot.safety_state),
                    "fault_code": snapshot.fault_code,
                    "last_input_sequence": snapshot.last_input_sequence,
                }
                payload = pack_recipe_payload(
                    configured.recipe_id,
                    configured.recipe,
                    values,
                )
                self._send_frame(MessageType.OUTPUT_DATA, payload)

                next_cycle_ns += period_ns
                now_ns = time.monotonic_ns()
                if next_cycle_ns <= now_ns:
                    skipped = (now_ns - next_cycle_ns) // period_ns + 1
                    next_cycle_ns += skipped * period_ns
                stop_event.wait(
                    max(0.0, (next_cycle_ns - time.monotonic_ns()) / 1e9)
                )
        except Exception as error:
            if not stop_event.is_set() and not self._stop_event.is_set():
                self._signal_async_failure(error)

    # 记录异步发送故障并唤醒接收线程
    def _signal_async_failure(self, error: Exception) -> None:
        with self._async_error_lock:
            if self._async_error is None:
                self._async_error = error
        self._session_end_event.set()
        try:
            self._connection.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass

    # 取出一次异步错误
    def _take_async_error(self) -> Exception | None:
        with self._async_error_lock:
            error = self._async_error
            self._async_error = None
            return error

    # 串行发送一条二进制协议帧
    def _send_frame(
        self,
        message_type: MessageType,
        payload: bytes = b"",
    ) -> None:
        with self._send_lock:
            self._connection.sendall(
                encode_frame(
                    message_type,
                    payload,
                    sequence=self._take_tx_sequence(),
                    version=self._selected_version or PROTOCOL_VERSION,
                )
            )

    # 发送JSON协议帧
    def _send_json(
        self,
        message_type: MessageType,
        values: dict[str, Any],
    ) -> None:
        with self._send_lock:
            self._connection.sendall(
                encode_json_frame(
                    message_type,
                    values,
                    sequence=self._take_tx_sequence(),
                    version=self._selected_version or PROTOCOL_VERSION,
                )
            )

    # 发送结构化错误帧
    def _send_error(
        self,
        code: str,
        message: str,
        related_sequence: int,
    ) -> None:
        self._send_json(
            MessageType.ERROR,
            {
                "code": code,
                "message": message,
                "related_sequence": related_sequence,
            },
        )

    # 尽力发送错误但不覆盖原始异常
    def _try_send_error(
        self,
        code: str,
        message: str,
        related_sequence: int,
    ) -> None:
        try:
            self._send_error(code, message, related_sequence)
        except OSError:
            pass

    # 分配并回绕服务端发送序号
    def _take_tx_sequence(self) -> int:
        sequence = self._next_tx_sequence
        self._next_tx_sequence = (self._next_tx_sequence + 1) & 0xFFFFFFFF
        return sequence


class NetworkServer:
    # 初始化持久TCP网络服务
    def __init__(
        self,
        ipc_client: Any,
        *,
        host: str = DEFAULT_SERVER_HOST,
        port: int = DEFAULT_SERVER_PORT,
        handshake_timeout_s: float = DEFAULT_HANDSHAKE_TIMEOUT_S,
        client_idle_timeout_s: float = DEFAULT_CLIENT_IDLE_TIMEOUT_S,
    ) -> None:
        if not isinstance(port, int) or isinstance(port, bool):
            raise ValueError("port must be an integer")
        if port < 0 or port > 65535:
            raise ValueError("port must be between 0 and 65535")
        if (
            not math.isfinite(handshake_timeout_s)
            or handshake_timeout_s <= 0.0
        ):
            raise ValueError("handshake_timeout_s must be finite and positive")
        if (
            not math.isfinite(client_idle_timeout_s)
            or client_idle_timeout_s <= 0.0
        ):
            raise ValueError("client_idle_timeout_s must be finite and positive")

        self._ipc_client = ipc_client
        self._host = host
        self._port = port
        self._handshake_timeout_s = float(handshake_timeout_s)
        self._client_idle_timeout_s = float(client_idle_timeout_s)
        self._stop_event = threading.Event()
        self._lifecycle_lock = threading.Lock()
        self._client_lock = threading.Lock()
        self._server_socket: socket.socket | None = None
        self._active_connection: socket.socket | None = None
        self._accept_thread: threading.Thread | None = None
        self._client_thread: threading.Thread | None = None
        self._bound_address: tuple[str, int] | None = None
        self._fatal_error: OSError | None = None

    # 返回服务实际监听地址
    @property
    def bound_address(self) -> tuple[str, int] | None:
        return self._bound_address

    # 返回TCP服务运行状态
    @property
    def running(self) -> bool:
        thread = self._accept_thread
        return thread is not None and thread.is_alive()

    # 返回导致监听线程退出的致命错误
    @property
    def fatal_error(self) -> OSError | None:
        return self._fatal_error

    # 连接IPC并启动TCP监听线程
    def start(self) -> None:
        with self._lifecycle_lock:
            with self._client_lock:
                client_thread_alive = (
                    self._client_thread is not None
                    and self._client_thread.is_alive()
                )
            if (
                self.running
                or client_thread_alive
                or self._server_socket is not None
            ):
                raise RuntimeError("network server is already running")

            self._ipc_client.connect()
            try:
                self._ipc_client.ping()
                server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server.bind((self._host, self._port))
                server.listen(2)
                server.settimeout(0.2)
            except Exception:
                self._ipc_client.close()
                if "server" in locals():
                    server.close()
                raise

            address = server.getsockname()
            self._bound_address = (str(address[0]), int(address[1]))
            self._server_socket = server
            self._fatal_error = None
            self._stop_event.clear()
            self._accept_thread = threading.Thread(
                target=self._accept_loop,
                name="rebot-network-accept",
                daemon=True,
            )
            self._accept_thread.start()

    # 停止TCP服务并关闭IPC客户端
    def stop(self) -> None:
        with self._lifecycle_lock:
            self._stop_event.set()

            server = self._server_socket
            self._server_socket = None
            if server is not None:
                server.close()

            self._close_active_connection()

            accept_thread = self._accept_thread
            if (
                accept_thread is not None
                and accept_thread.is_alive()
                and accept_thread is not threading.current_thread()
            ):
                accept_thread.join(timeout=THREAD_JOIN_TIMEOUT_S)

            with self._client_lock:
                client_thread = self._client_thread
            if (
                client_thread is not None
                and client_thread.is_alive()
                and client_thread is not threading.current_thread()
            ):
                client_thread.join(timeout=THREAD_JOIN_TIMEOUT_S)

            if client_thread is not None and client_thread.is_alive():
                self._ipc_client.close()
                client_thread.join(timeout=THREAD_JOIN_TIMEOUT_S)

            self._ipc_client.close()
            self._bound_address = None

            alive_threads = []
            if accept_thread is not None and accept_thread.is_alive():
                alive_threads.append("accept")
            if client_thread is not None and client_thread.is_alive():
                alive_threads.append("client")
            if alive_threads:
                raise RuntimeError(
                    "network server threads did not stop: "
                    + ", ".join(alive_threads)
                )

            self._accept_thread = None
            with self._client_lock:
                self._client_thread = None
                self._active_connection = None

    # 接收连接并确保只有一个活跃上层主机
    def _accept_loop(self) -> None:
        while not self._stop_event.is_set():
            server = self._server_socket
            if server is None:
                break
            try:
                connection, address = server.accept()
            except socket.timeout:
                continue
            except OSError as error:
                if self._stop_event.is_set():
                    break
                if error.errno in (errno.EINTR, errno.ECONNABORTED):
                    continue
                LOGGER.exception("TCP accept failed")
                self._mark_fatal(error)
                break

            if self._stop_event.is_set():
                self._close_connection(connection)
                break

            try:
                self._configure_connection(connection)
            except OSError:
                LOGGER.exception("TCP connection setup failed")
                self._close_connection(connection)
                continue
            with self._client_lock:
                if self._active_connection is not None:
                    busy = True
                else:
                    busy = False
                    self._active_connection = connection
                    thread = threading.Thread(
                        target=self._run_client,
                        args=(connection, address),
                        name=f"rebot-network-client-{address[0]}:{address[1]}",
                        daemon=True,
                    )
                    self._client_thread = thread

            if busy:
                self._reject_busy_connection(connection)
                continue

            LOGGER.info("上层主机已连接: %s:%s", address[0], address[1])
            try:
                thread.start()
            except Exception:
                LOGGER.exception("TCP client thread failed to start")
                with self._client_lock:
                    if self._active_connection is connection:
                        self._active_connection = None
                    if self._client_thread is thread:
                        self._client_thread = None
                self._close_connection(connection)

    # 设置低延迟和连接保活选项
    @staticmethod
    def _configure_connection(connection: socket.socket) -> None:
        connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        connection.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        connection.settimeout(0.2)

    # 运行一个持久协议会话
    def _run_client(
        self,
        connection: socket.socket,
        address: tuple[str, int],
    ) -> None:
        session = NetworkSession(
            connection,
            address,
            self._ipc_client,
            self._stop_event,
            handshake_timeout_s=self._handshake_timeout_s,
            client_idle_timeout_s=self._client_idle_timeout_s,
        )
        try:
            self._ensure_ipc_available()
            session.run()
        except (IPCClientError, IPCProtocolError) as error:
            self._ipc_client.close()
            try:
                connection.sendall(
                    encode_json_frame(
                        MessageType.ERROR,
                        {
                            "code": "IPC_UNAVAILABLE",
                            "message": str(error),
                            "related_sequence": 0,
                        },
                        sequence=0,
                    )
                )
            except OSError:
                pass
        except OSError:
            pass
        except Exception:
            LOGGER.exception("unexpected network session failure")
        finally:
            if (
                session.negotiated
                and not session.safe_reset_completed
                and not self._stop_event.is_set()
                and self._ipc_is_connected()
            ):
                try:
                    self._ipc_client.reset_session()
                except Exception as error:
                    LOGGER.warning("重置IPC控制会话失败: %s", error)
                    self._ipc_client.close()

            self._close_connection(connection)
            with self._client_lock:
                if self._active_connection is connection:
                    self._active_connection = None
                if self._client_thread is threading.current_thread():
                    self._client_thread = None
            LOGGER.info("上层主机已断开: %s:%s", address[0], address[1])

    # 确保持久IPC连接可用
    def _ensure_ipc_available(self) -> None:
        self._ipc_client.connect()
        self._ipc_client.ping()

    # 返回IPC客户端报告的连接状态
    def _ipc_is_connected(self) -> bool:
        return bool(getattr(self._ipc_client, "connected", True))

    # 标记监听服务故障并关闭控制通道
    def _mark_fatal(self, error: OSError) -> None:
        self._fatal_error = error
        self._stop_event.set()
        server = self._server_socket
        if server is not None:
            server.close()
        self._close_active_connection()
        self._ipc_client.close()

    # 拒绝额外的并发控制连接
    @staticmethod
    def _reject_busy_connection(connection: socket.socket) -> None:
        try:
            connection.sendall(
                encode_json_frame(
                    MessageType.ERROR,
                    {
                        "code": "SERVER_BUSY",
                        "message": "another host connection is already active",
                        "related_sequence": 0,
                    },
                    sequence=0,
                )
            )
        except OSError:
            pass
        finally:
            NetworkServer._close_connection(connection)

    # 关闭当前活跃连接以唤醒接收线程
    def _close_active_connection(self) -> None:
        with self._client_lock:
            connection = self._active_connection
        if connection is not None:
            self._close_connection(connection)

    # 安全关闭单个TCP连接
    @staticmethod
    def _close_connection(connection: socket.socket) -> None:
        try:
            connection.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        connection.close()


# 启动持久TCP网络服务
def main() -> None:
    parser = argparse.ArgumentParser(description="reBot persistent TCP server")
    parser.add_argument("--host", default=DEFAULT_SERVER_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_SERVER_PORT)
    parser.add_argument(
        "--ipc-socket",
        default=DEFAULT_IPC_SOCKET_PATH,
    )
    parser.add_argument(
        "--handshake-timeout",
        type=float,
        default=DEFAULT_HANDSHAKE_TIMEOUT_S,
    )
    parser.add_argument(
        "--client-idle-timeout",
        type=float,
        default=DEFAULT_CLIENT_IDLE_TIMEOUT_S,
    )
    arguments = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    ipc_client = ControlIPCClient(arguments.ipc_socket)
    server = NetworkServer(
        ipc_client,
        host=arguments.host,
        port=arguments.port,
        handshake_timeout_s=arguments.handshake_timeout,
        client_idle_timeout_s=arguments.client_idle_timeout,
    )

    try:
        server.start()
        address = server.bound_address
        if address is None:
            raise RuntimeError("network server did not bind an address")
        print(f"网络服务已启动: {address[0]}:{address[1]}")
        print("按 Ctrl+C 停止")
        while True:
            if server.fatal_error is not None:
                raise RuntimeError("network accept loop failed") from server.fatal_error
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n网络服务正在停止")
    finally:
        server.stop()


if __name__ == "__main__":
    main()
