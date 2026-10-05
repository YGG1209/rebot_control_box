from __future__ import annotations

import logging
import math
import socket
import threading
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any

from ._exceptions import (
    RTDEConnectionError as NetworkConnectionError,
    RTDEControlError as RemoteProtocolError,
    RTDEError as NetworkClientError,
    RTDETimeoutError as NetworkTimeoutError,
)
from ._models import ControlLease, RobotState
from ._protocol import (
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
    decode_json_payload,
    encode_frame,
    encode_json_payload,
    is_newer_sequence,
    pack_recipe_payload,
    recipe_types,
    resolve_recipe,
    unpack_recipe_payload,
)


LOGGER = logging.getLogger("rebot-rtde")

DEFAULT_CONTROLLER_HOST = "192.168.100.2"
DEFAULT_CONTROLLER_PORT = 5000
DEFAULT_REQUEST_TIMEOUT_S = 2.0
DEFAULT_HEARTBEAT_INTERVAL_S = 0.25
RECEIVE_CHUNK_SIZE = 64 * 1024

DEFAULT_INPUT_FIELDS = (
    "control_mode",
    "target_q",
    "speed_limits",
)
DEFAULT_OUTPUT_FIELDS = (
    "controller_time_ns",
    "actual_q",
    "actual_qd",
    "actual_tau",
    "target_q",
    "robot_mode",
    "safety_state",
    "fault_code",
    "last_input_sequence",
)


@dataclass(slots=True)
class _PendingRequest:
    """等待中的请求。"""

    expected_type: MessageType
    event: threading.Event
    response: dict[str, Any] | None = None
    error: BaseException | None = None


class RebotNetworkClient:
    """内部持久TCP客户端。"""

    # 初始化上层主机持久TCP客户端
    def __init__(
        self,
        host: str = DEFAULT_CONTROLLER_HOST,
        port: int = DEFAULT_CONTROLLER_PORT,
        *,
        client_name: str = "rebot-python-client",
        connect_timeout_s: float = 3.0,
        request_timeout_s: float = DEFAULT_REQUEST_TIMEOUT_S,
        heartbeat_interval_s: float = DEFAULT_HEARTBEAT_INTERVAL_S,
    ) -> None:
        if not isinstance(host, str) or not host:
            raise ValueError("host must be a non-empty string")
        if isinstance(port, bool) or not isinstance(port, int):
            raise ValueError("port must be an integer")
        if port < 1 or port > 65535:
            raise ValueError("port must be between 1 and 65535")
        if (
            not isinstance(client_name, str)
            or not client_name.strip()
            or len(client_name) > 128
        ):
            raise ValueError(
                "client_name must be a non-empty string up to 128 characters"
            )
        self._validate_positive_timeout("connect_timeout_s", connect_timeout_s)
        self._validate_positive_timeout("request_timeout_s", request_timeout_s)
        self._validate_positive_timeout(
            "heartbeat_interval_s",
            heartbeat_interval_s,
        )

        self._host = host
        self._port = port
        self._client_name = client_name
        self._connect_timeout_s = float(connect_timeout_s)
        self._request_timeout_s = float(request_timeout_s)
        self._heartbeat_interval_s = float(heartbeat_interval_s)

        self._lifecycle_lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._pending_lock = threading.Lock()
        self._state_condition = threading.Condition()
        self._stop_event = threading.Event()

        self._socket: socket.socket | None = None
        self._receiver_thread: threading.Thread | None = None
        self._heartbeat_thread: threading.Thread | None = None
        self._decoder = FrameDecoder()
        self._pending: dict[int, _PendingRequest] = {}
        self._next_tx_sequence = 0
        self._last_rx_sequence: int | None = None
        self._last_tx_at = 0.0
        self._selected_version = PROTOCOL_VERSION
        self._negotiated = False
        self._failure: BaseException | None = None

        self._input_recipe_id: int | None = None
        self._input_recipe: Recipe | None = None
        self._output_recipe_id: int | None = None
        self._output_recipe: Recipe | None = None
        self._latest_state: RobotState | None = None
        self._lease: ControlLease | None = None
        self._started = False

    # 返回TCP协议会话是否可用
    @property
    def connected(self) -> bool:
        return self._socket is not None and self._negotiated

    # 返回是否已获得写控制权
    @property
    def has_control(self) -> bool:
        return self._lease is not None

    # 返回数据流是否已经启动
    @property
    def started(self) -> bool:
        return self._started

    # 返回最近一次输出状态
    @property
    def latest_state(self) -> RobotState | None:
        with self._state_condition:
            return self._latest_state

    # 建立TCP连接并完成版本协商
    def connect(self) -> None:
        with self._lifecycle_lock:
            if self._socket is not None:
                if self._negotiated:
                    return
                raise NetworkClientError("client connection is already starting")

            try:
                connection = socket.create_connection(
                    (self._host, self._port),
                    timeout=self._connect_timeout_s,
                )
            except OSError as error:
                raise NetworkConnectionError(
                    f"cannot connect to {self._host}:{self._port}"
                ) from error

            try:
                connection.setsockopt(
                    socket.IPPROTO_TCP,
                    socket.TCP_NODELAY,
                    1,
                )
                connection.setsockopt(
                    socket.SOL_SOCKET,
                    socket.SO_KEEPALIVE,
                    1,
                )
                connection.settimeout(0.2)
            except OSError as error:
                connection.close()
                raise NetworkConnectionError(
                    "failed to configure the controller connection"
                ) from error

            self._reset_session_state()
            self._socket = connection
            self._receiver_thread = threading.Thread(
                target=self._receive_loop,
                name="rebot-host-receiver",
                daemon=True,
            )
            self._receiver_thread.start()

        try:
            response = self._request_json(
                MessageType.HELLO,
                {
                    "client_name": self._client_name,
                    "supported_versions": list(SUPPORTED_VERSIONS),
                },
                MessageType.HELLO_ACK,
            )
            self._validate_accepted(response, "HELLO")
            selected_version = self._require_uint(
                response,
                "selected_version",
                8,
            )
            if selected_version not in SUPPORTED_VERSIONS:
                raise ProtocolError(
                    f"server selected unsupported version {selected_version}"
                )
            self._selected_version = selected_version
            self._negotiated = True
            self._heartbeat_thread = threading.Thread(
                target=self._heartbeat_loop,
                name="rebot-host-heartbeat",
                daemon=True,
            )
            self._heartbeat_thread.start()
        except Exception:
            self.close()
            raise

    # 配置控制箱到主机的状态输出
    def setup_outputs(
        self,
        fields: Sequence[str] = DEFAULT_OUTPUT_FIELDS,
        *,
        frequency_hz: float = 60.0,
    ) -> int:
        self._require_negotiated()
        self._require_not_started()
        recipe = self._resolve_fields(fields, OUTPUT_FIELDS)
        frequency = self._validate_frequency(frequency_hz)
        response = self._request_json(
            MessageType.SETUP_OUTPUTS,
            {
                "frequency_hz": frequency,
                "fields": [name for name, _ in recipe],
            },
            MessageType.SETUP_OUTPUTS_ACK,
        )
        recipe_id = self._validate_recipe_ack(
            response,
            recipe,
            frequency,
            "SETUP_OUTPUTS",
        )
        self._output_recipe_id = recipe_id
        self._output_recipe = recipe
        with self._state_condition:
            self._latest_state = None
        return recipe_id

    # 配置主机到控制箱的servoJ输入
    def setup_inputs(
        self,
        fields: Sequence[str] = DEFAULT_INPUT_FIELDS,
        *,
        frequency_hz: float = 60.0,
    ) -> int:
        self._require_negotiated()
        self._require_not_started()
        recipe = self._resolve_fields(fields, INPUT_FIELDS)
        frequency = self._validate_frequency(frequency_hz)
        response = self._request_json(
            MessageType.SETUP_INPUTS,
            {
                "frequency_hz": frequency,
                "fields": [name for name, _ in recipe],
            },
            MessageType.SETUP_INPUTS_ACK,
        )
        recipe_id = self._validate_recipe_ack(
            response,
            recipe,
            frequency,
            "SETUP_INPUTS",
        )
        self._input_recipe_id = recipe_id
        self._input_recipe = recipe
        return recipe_id

    # 申请当前连接的独占控制权
    def acquire_control(
        self,
        *,
        requested_lease_ms: int = 1000,
    ) -> ControlLease:
        self._require_negotiated()
        self._require_not_started()
        if self._input_recipe is None:
            raise NetworkClientError("setup_inputs must be called first")
        if isinstance(requested_lease_ms, bool) or not isinstance(
            requested_lease_ms,
            int,
        ):
            raise ValueError("requested_lease_ms must be an integer")
        if requested_lease_ms <= 0:
            raise ValueError("requested_lease_ms must be positive")

        response = self._request_json(
            MessageType.ACQUIRE_CONTROL,
            {
                "mode": "exclusive",
                "requested_lease_ms": requested_lease_ms,
            },
            MessageType.ACQUIRE_CONTROL_ACK,
        )
        self._validate_accepted(response, "ACQUIRE_CONTROL")
        session_id = response.get("control_session_id")
        if (
            isinstance(session_id, bool)
            or not isinstance(session_id, (int, str))
            or isinstance(session_id, str)
            and not session_id
        ):
            raise ProtocolError("invalid control_session_id in acknowledgement")
        lease_ms = self._require_uint(response, "lease_ms", 32)
        if lease_ms == 0:
            raise ProtocolError("lease_ms must be positive")
        lease = ControlLease(session_id, lease_ms)
        self._lease = lease
        return lease

    # 启动已经配置的实时数据流
    def start(self) -> None:
        self._require_negotiated()
        if self._started:
            return
        if self._output_recipe is None or self._output_recipe_id is None:
            raise NetworkClientError("setup_outputs must be called first")
        if self._input_recipe is not None and self._lease is None:
            raise NetworkClientError("acquire_control must be called first")

        response = self._request_empty(MessageType.START, MessageType.START_ACK)
        self._validate_accepted(response, "START")
        output_id = self._require_uint(response, "output_recipe_id", 8)
        if output_id != self._output_recipe_id:
            raise ProtocolError("START_ACK output_recipe_id does not match")
        input_id = response.get("input_recipe_id")
        if self._input_recipe_id is not None:
            if input_id != self._input_recipe_id:
                raise ProtocolError("START_ACK input_recipe_id does not match")
        elif input_id is not None:
            raise ProtocolError("START_ACK contains an unexpected input recipe")
        self._started = True

    # 发送一帧servoJ目标且不等待逐帧ACK
    def send_servoj(
        self,
        target_q: Sequence[float],
        speed_limits: float | Sequence[float],
    ) -> int:
        self._require_negotiated()
        if not self._started:
            raise NetworkClientError("START must be accepted before INPUT_DATA")
        if self._lease is None:
            raise NetworkClientError("control ownership is required")
        if self._input_recipe is None or self._input_recipe_id is None:
            raise NetworkClientError("input recipe is not configured")

        speeds = self._as_six_values(
            speed_limits,
            "speed_limits",
            allow_scalar=True,
            require_positive=True,
        )
        targets = self._as_six_values(target_q, "target_q")
        payload = pack_recipe_payload(
            self._input_recipe_id,
            self._input_recipe,
            {
                "control_mode": int(ControlMode.SERVOJ),
                "target_q": targets,
                "speed_limits": speeds,
            },
        )
        return self._send_oneway(MessageType.INPUT_DATA, payload)

    # 等待任意状态或指定服务端帧之后的新状态
    def wait_for_state(
        self,
        timeout_s: float = 2.0,
        *,
        after_sequence: int | None = None,
        predicate: Callable[[RobotState], bool] | None = None,
    ) -> RobotState:
        self._validate_positive_timeout("timeout_s", timeout_s)
        if after_sequence is not None:
            self._validate_uint_value("after_sequence", after_sequence, 32)
        deadline = time.monotonic() + timeout_s

        with self._state_condition:
            while True:
                state = self._latest_state
                is_new = (
                    state is not None
                    and (
                        after_sequence is None
                        or is_newer_sequence(
                            state.frame_sequence,
                            after_sequence,
                        )
                    )
                )
                if is_new and (predicate is None or predicate(state)):
                    return state
                self._raise_connection_failure()
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise NetworkTimeoutError("timed out waiting for robot state")
                self._state_condition.wait(remaining)

    # 等待控制守护进程确认已经应用指定输入帧
    def wait_until_applied(
        self,
        input_sequence: int,
        timeout_s: float = 1.0,
    ) -> RobotState:
        self._validate_uint_value("input_sequence", input_sequence, 32)
        recipe = self._output_recipe
        if recipe is None or "last_input_sequence" not in {
            name for name, _ in recipe
        }:
            raise NetworkClientError(
                "output recipe must contain last_input_sequence"
            )

        # 判断指定输入帧是否已经生效
        def was_applied(state: RobotState) -> bool:
            current = state.values.get("last_input_sequence")
            return isinstance(current, int) and (
                current == input_sequence
                or is_newer_sequence(current, input_sequence)
            )

        return self.wait_for_state(timeout_s, predicate=was_applied)

    # 暂停数据流并让控制箱保持当前位置
    def pause(self) -> None:
        self._require_negotiated()
        if not self._started:
            return
        response = self._request_empty(MessageType.PAUSE, MessageType.PAUSE_ACK)
        self._validate_accepted(response, "PAUSE")
        self._started = False

    # 释放当前连接持有的控制权
    def release_control(self) -> None:
        self._require_negotiated()
        if self._lease is None:
            return
        if self._started:
            raise NetworkClientError("pause must be called before release_control")
        response = self._request_empty(
            MessageType.RELEASE_CONTROL,
            MessageType.RELEASE_CONTROL_ACK,
        )
        self._validate_accepted(response, "RELEASE_CONTROL")
        self._lease = None

    # 关闭连接并停止后台线程
    def close(self) -> None:
        failure = NetworkConnectionError("client connection is closed")
        self._abort_connection(failure)

        current = threading.current_thread()
        for thread in (self._heartbeat_thread, self._receiver_thread):
            if thread is not None and thread is not current and thread.is_alive():
                thread.join(timeout=2.0)
        self._heartbeat_thread = None
        self._receiver_thread = None

    # 进入上下文并连接控制箱
    def __enter__(self) -> RebotNetworkClient:
        self.connect()
        return self

    # 退出上下文并关闭连接
    def __exit__(self, *_: object) -> None:
        self.close()

    # 清空一次TCP会话的全部状态
    def _reset_session_state(self) -> None:
        self._stop_event.clear()
        self._decoder.reset()
        with self._pending_lock:
            self._pending.clear()
        self._next_tx_sequence = 0
        self._last_rx_sequence = None
        self._last_tx_at = time.monotonic()
        self._selected_version = PROTOCOL_VERSION
        self._negotiated = False
        self._failure = None
        self._input_recipe_id = None
        self._input_recipe = None
        self._output_recipe_id = None
        self._output_recipe = None
        self._lease = None
        self._started = False
        with self._state_condition:
            self._latest_state = None

    # 在单一线程中接收和拆分全部TCP帧
    def _receive_loop(self) -> None:
        try:
            while not self._stop_event.is_set():
                connection = self._socket
                if connection is None:
                    return
                try:
                    data = connection.recv(RECEIVE_CHUNK_SIZE)
                except socket.timeout:
                    continue
                if not data:
                    raise NetworkConnectionError("controller closed the connection")

                for frame in self._decoder.feed(data):
                    self._validate_incoming_frame(frame)
                    self._dispatch_incoming_frame(frame)
        except (OSError, ProtocolError, NetworkClientError) as error:
            if not self._stop_event.is_set():
                wrapped = (
                    error
                    if isinstance(error, NetworkClientError)
                    else NetworkConnectionError(str(error))
                )
                self._abort_connection(wrapped)
        except Exception as error:
            if not self._stop_event.is_set():
                LOGGER.exception("unexpected network receiver failure")
                self._abort_connection(NetworkConnectionError(str(error)))

    # 校验服务端版本、标志和严格递增序号
    def _validate_incoming_frame(self, frame: Frame) -> None:
        if frame.flags != 0:
            raise ProtocolError(f"server frame has unsupported flags {frame.flags}")
        if self._negotiated and frame.version != self._selected_version:
            raise ProtocolError(
                f"server frame version changed to {frame.version}"
            )
        previous = self._last_rx_sequence
        if previous is not None and not is_newer_sequence(
            frame.sequence,
            previous,
        ):
            raise ProtocolError("server frame sequence is not strictly newer")
        self._last_rx_sequence = frame.sequence

    # 分发响应帧和异步状态帧
    def _dispatch_incoming_frame(self, frame: Frame) -> None:
        if frame.message_type is MessageType.OUTPUT_DATA:
            self._handle_output_data(frame)
            return
        if frame.message_type is MessageType.TEXT_MESSAGE:
            values = decode_json_payload(frame.payload)
            LOGGER.info("控制箱消息: %s", values)
            return

        response_types = {
            MessageType.HELLO_ACK,
            MessageType.SETUP_INPUTS_ACK,
            MessageType.SETUP_OUTPUTS_ACK,
            MessageType.ACQUIRE_CONTROL_ACK,
            MessageType.RELEASE_CONTROL_ACK,
            MessageType.START_ACK,
            MessageType.PAUSE_ACK,
            MessageType.HEARTBEAT_ACK,
            MessageType.ERROR,
        }
        if frame.message_type not in response_types:
            raise ProtocolError(
                f"unexpected server message {frame.message_type.name}"
            )

        values = decode_json_payload(frame.payload)
        related_sequence = self._require_uint(
            values,
            "related_sequence",
            32,
        )
        with self._pending_lock:
            pending = self._pending.get(related_sequence)
        if pending is None:
            if frame.message_type is MessageType.ERROR:
                raise self._remote_error(values, related_sequence)
            raise ProtocolError(
                f"response refers to unknown sequence {related_sequence}"
            )

        if frame.message_type is MessageType.ERROR:
            pending.error = self._remote_error(values, related_sequence)
        elif frame.message_type is not pending.expected_type:
            pending.error = ProtocolError(
                f"expected {pending.expected_type.name}, got "
                f"{frame.message_type.name}"
            )
        else:
            pending.response = values
        pending.event.set()

    # 解码异步输出Recipe并发布最新状态
    def _handle_output_data(self, frame: Frame) -> None:
        recipe = self._output_recipe
        recipe_id = self._output_recipe_id
        if recipe is None or recipe_id is None:
            raise ProtocolError("received OUTPUT_DATA before output setup")
        decoded_id, values = unpack_recipe_payload(frame.payload, recipe)
        if decoded_id != recipe_id:
            raise ProtocolError(
                f"OUTPUT_DATA recipe_id is {decoded_id}, expected {recipe_id}"
            )
        state = RobotState(
            frame_sequence=frame.sequence,
            controller_timestamp_ns=frame.timestamp_ns,
            received_at_ns=time.monotonic_ns(),
            values=MappingProxyType(values),
        )
        with self._state_condition:
            self._latest_state = state
            self._state_condition.notify_all()

    # 空闲时发送心跳但不重发运动目标
    def _heartbeat_loop(self) -> None:
        poll_interval = min(self._heartbeat_interval_s / 2.0, 0.1)
        while not self._stop_event.wait(poll_interval):
            with self._send_lock:
                idle_s = time.monotonic() - self._last_tx_at
            if idle_s < self._heartbeat_interval_s:
                continue
            try:
                self._request_empty(
                    MessageType.HEARTBEAT,
                    MessageType.HEARTBEAT_ACK,
                )
            except NetworkClientError as error:
                if not self._stop_event.is_set():
                    self._abort_connection(error)
                return

    # 发送JSON请求并等待对应响应
    def _request_json(
        self,
        message_type: MessageType,
        values: Mapping[str, Any],
        expected_type: MessageType,
    ) -> dict[str, Any]:
        return self._request(
            message_type,
            encode_json_payload(values),
            expected_type,
        )

    # 发送空负载请求并等待对应响应
    def _request_empty(
        self,
        message_type: MessageType,
        expected_type: MessageType,
    ) -> dict[str, Any]:
        return self._request(message_type, b"", expected_type)

    # 按related_sequence完成一次请求响应
    def _request(
        self,
        message_type: MessageType,
        payload: bytes,
        expected_type: MessageType,
    ) -> dict[str, Any]:
        pending = _PendingRequest(expected_type, threading.Event())
        sequence = self._send_payload(message_type, payload, pending)
        if not pending.event.wait(self._request_timeout_s):
            with self._pending_lock:
                self._pending.pop(sequence, None)
            error = NetworkTimeoutError(
                f"timed out waiting for {expected_type.name}"
            )
            self._abort_connection(error)
            raise error

        with self._pending_lock:
            self._pending.pop(sequence, None)
        if pending.error is not None:
            raise pending.error
        if pending.response is None:
            raise ProtocolError("response completed without a payload")
        return pending.response

    # 发送无需ACK的数据帧并返回其序号
    def _send_oneway(self, message_type: MessageType, payload: bytes) -> int:
        return self._send_payload(message_type, payload, None)

    # 串行分配序号并发送完整协议帧
    def _send_payload(
        self,
        message_type: MessageType,
        payload: bytes,
        pending: _PendingRequest | None,
    ) -> int:
        send_error: OSError | None = None
        sequence = 0
        with self._send_lock:
            connection = self._require_socket()
            sequence = self._next_tx_sequence
            self._next_tx_sequence = (sequence + 1) & 0xFFFFFFFF
            if pending is not None:
                with self._pending_lock:
                    if sequence in self._pending:
                        raise NetworkClientError(
                            "request sequence wrapped while still pending"
                        )
                    self._pending[sequence] = pending
            frame = encode_frame(
                message_type,
                payload,
                sequence=sequence,
                version=self._selected_version,
            )
            try:
                connection.sendall(frame)
                self._last_tx_at = time.monotonic()
            except OSError as error:
                send_error = error

        if send_error is not None:
            with self._pending_lock:
                self._pending.pop(sequence, None)
            failure = NetworkConnectionError("failed to send protocol frame")
            self._abort_connection(failure)
            raise failure from send_error
        return sequence

    # 关闭异常连接并唤醒所有等待者
    def _abort_connection(self, failure: BaseException) -> None:
        with self._lifecycle_lock:
            if self._failure is None:
                self._failure = failure
            self._stop_event.set()
            connection = self._socket
            self._socket = None
            self._negotiated = False
            self._started = False
            self._lease = None

        if connection is not None:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            connection.close()

        with self._pending_lock:
            pending_requests = list(self._pending.values())
        for pending in pending_requests:
            if pending.error is None:
                pending.error = self._failure
            pending.event.set()
        with self._state_condition:
            self._state_condition.notify_all()

    # 返回当前Socket或抛出已记录的连接错误
    def _require_socket(self) -> socket.socket:
        connection = self._socket
        if connection is None:
            self._raise_connection_failure()
            raise NetworkConnectionError("client is not connected")
        return connection

    # 检查版本协商已经完成
    def _require_negotiated(self) -> None:
        if not self.connected:
            self._raise_connection_failure()
            raise NetworkConnectionError("client is not connected")

    # 禁止在流运行时替换Recipe
    def _require_not_started(self) -> None:
        if self._started:
            raise NetworkClientError("pause must be called before changing setup")

    # 抛出后台线程记录的连接错误
    def _raise_connection_failure(self) -> None:
        failure = self._failure
        if failure is not None:
            raise failure

    # 校验ACK已被控制箱接受
    @staticmethod
    def _validate_accepted(response: Mapping[str, Any], operation: str) -> None:
        if response.get("accepted") is not True:
            raise ProtocolError(f"{operation} acknowledgement was not accepted")

    # 校验Recipe确认内容并返回编号
    def _validate_recipe_ack(
        self,
        response: Mapping[str, Any],
        recipe: Recipe,
        frequency_hz: float,
        operation: str,
    ) -> int:
        self._validate_accepted(response, operation)
        recipe_id = self._require_uint(response, "recipe_id", 8)
        expected_fields = [name for name, _ in recipe]
        if response.get("fields") != expected_fields:
            raise ProtocolError(f"{operation} fields do not match the request")
        if response.get("types") != recipe_types(recipe):
            raise ProtocolError(f"{operation} types do not match the recipe")
        accepted_frequency = response.get("frequency_hz")
        if (
            isinstance(accepted_frequency, bool)
            or not isinstance(accepted_frequency, (int, float))
            or float(accepted_frequency) != frequency_hz
        ):
            raise ProtocolError(f"{operation} frequency does not match")
        return recipe_id

    # 从错误JSON构造远端异常
    @staticmethod
    def _remote_error(
        values: Mapping[str, Any],
        related_sequence: int,
    ) -> RemoteProtocolError:
        code = values.get("code")
        message = values.get("message")
        if not isinstance(code, str) or not isinstance(message, str):
            raise ProtocolError("ERROR payload requires string code and message")
        return RemoteProtocolError(code, message, related_sequence)

    # 从JSON读取指定宽度的无符号整数
    @classmethod
    def _require_uint(
        cls,
        values: Mapping[str, Any],
        name: str,
        bits: int,
    ) -> int:
        value = values.get(name)
        cls._validate_uint_value(name, value, bits)
        return int(value)

    # 校验无符号整数
    @staticmethod
    def _validate_uint_value(name: str, value: Any, bits: int) -> None:
        if (
            isinstance(value, bool)
            or not isinstance(value, int)
            or value < 0
            or value >= 1 << bits
        ):
            raise ProtocolError(f"{name} must fit in uint{bits}")

    # 将字段名解析为有序Recipe
    @staticmethod
    def _resolve_fields(
        fields: Sequence[str],
        registry: Mapping[str, Any],
    ) -> Recipe:
        if isinstance(fields, (str, bytes)):
            raise ValueError("fields must be a sequence of field names")
        return resolve_recipe(fields, registry)

    # 校验Recipe频率
    @staticmethod
    def _validate_frequency(value: float) -> float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError("frequency_hz must be a number")
        frequency = float(value)
        if not math.isfinite(frequency) or frequency <= 0.0:
            raise ValueError("frequency_hz must be finite and positive")
        return frequency

    # 校验正超时时间
    @staticmethod
    def _validate_positive_timeout(name: str, value: float) -> None:
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            or float(value) <= 0.0
        ):
            raise ValueError(f"{name} must be finite and positive")

    # 将标量或序列转换为六个有限数
    @staticmethod
    def _as_six_values(
        values: float | Sequence[float],
        name: str,
        *,
        allow_scalar: bool = False,
        require_positive: bool = False,
    ) -> tuple[float, ...]:
        try:
            if (
                allow_scalar
                and not isinstance(values, bool)
                and isinstance(values, (int, float))
            ):
                converted = (float(values),) * 6
            else:
                items = tuple(values)
                if any(
                    isinstance(item, (bool, str, bytes))
                    for item in items
                ):
                    raise TypeError
                converted = tuple(float(item) for item in items)
        except (TypeError, ValueError) as error:
            raise ValueError(f"{name} must contain numeric values") from error
        if len(converted) != 6:
            raise ValueError(f"{name} must contain exactly 6 values")
        if not all(math.isfinite(item) for item in converted):
            raise ValueError(f"{name} must contain only finite values")
        if require_positive and any(item <= 0.0 for item in converted):
            raise ValueError(f"{name} values must be positive")
        return converted
