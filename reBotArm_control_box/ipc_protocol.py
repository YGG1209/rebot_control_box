from __future__ import annotations

import json
import math
import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Any, Mapping, Sequence

from reBotArm_control_box.protocol import RobotMode, SafetyState


# IPC固定参数
IPC_MAGIC = b"RBIP"
IPC_VERSION = 1
IPC_MAX_PAYLOAD_SIZE = 16 * 1024
DEFAULT_IPC_SOCKET_PATH = "/tmp/rebot-control-box.sock"

# magic, version, message type, flags, payload length, request id
IPC_HEADER_STRUCT = struct.Struct("!4sBBHII")
IPC_HEADER_SIZE = IPC_HEADER_STRUCT.size
IPC_MAX_PACKET_SIZE = IPC_HEADER_SIZE + IPC_MAX_PAYLOAD_SIZE


class IPCProtocolError(ValueError):
    pass


class IPCMessageType(IntEnum):
    PING = 0x01
    PONG = 0x02

    SUBMIT_SERVOJ = 0x10
    COMMAND_ACK = 0x11
    PAUSE = 0x12
    PAUSE_ACK = 0x13
    RESET_SESSION = 0x14
    RESET_SESSION_ACK = 0x15

    GET_SNAPSHOT = 0x20
    SNAPSHOT = 0x21

    ERROR = 0x7F


@dataclass(frozen=True, slots=True)
class IPCPacket:
    message_type: IPCMessageType
    request_id: int
    payload: dict[str, Any]


@dataclass(frozen=True, slots=True)
class IPCSnapshot:
    timestamp_ns: int
    daemon_state: RobotMode
    safety_state: SafetyState
    actual_q: tuple[float, ...]
    actual_qd: tuple[float, ...]
    actual_tau: tuple[float, ...]
    target_q: tuple[float, ...]
    last_input_sequence: int
    fault_code: int
    fault_message: str


# 检查整数是否符合无符号位宽
def validate_uint(name: str, value: int, bits: int) -> int:
    if isinstance(value, bool):
        raise IPCProtocolError(f"{name} must be an integer")
    try:
        result = int(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise IPCProtocolError(f"{name} must be an integer") from error
    if result != value or result < 0 or result >= 1 << bits:
        raise IPCProtocolError(f"{name} must fit in uint{bits}")
    return result


# 将JSON对象编码为紧凑字节
def encode_json_payload(values: Mapping[str, Any]) -> bytes:
    if not isinstance(values, Mapping):
        raise IPCProtocolError("IPC payload must be an object")
    try:
        payload = json.dumps(
            dict(values),
            ensure_ascii=False,
            allow_nan=False,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8")
    except (TypeError, ValueError) as error:
        raise IPCProtocolError("IPC payload is not JSON serializable") from error
    if len(payload) > IPC_MAX_PAYLOAD_SIZE:
        raise IPCProtocolError(
            f"IPC payload exceeds {IPC_MAX_PAYLOAD_SIZE} bytes"
        )
    return payload


# 拒绝JSON中的非标准数值
def _reject_json_constant(value: str) -> None:
    raise IPCProtocolError(f"invalid JSON constant: {value}")


# 将JSON字节解码为对象
def decode_json_payload(payload: bytes) -> dict[str, Any]:
    try:
        values = json.loads(
            payload.decode("utf-8"),
            parse_constant=_reject_json_constant,
        )
    except IPCProtocolError:
        raise
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise IPCProtocolError("invalid IPC JSON payload") from error
    if not isinstance(values, dict):
        raise IPCProtocolError("IPC payload must be an object")
    return values


# 将IPC消息编码为单个SEQPACKET数据包
def encode_packet(
    message_type: IPCMessageType | int,
    request_id: int,
    payload: Mapping[str, Any] | None = None,
    *,
    flags: int = 0,
) -> bytes:
    try:
        message_type = IPCMessageType(int(message_type))
    except (TypeError, ValueError) as error:
        raise IPCProtocolError(f"unknown IPC message type: {message_type}") from error

    request_id = validate_uint("request_id", request_id, 32)
    flags = validate_uint("flags", flags, 16)
    payload_bytes = encode_json_payload({} if payload is None else payload)
    header = IPC_HEADER_STRUCT.pack(
        IPC_MAGIC,
        IPC_VERSION,
        int(message_type),
        flags,
        len(payload_bytes),
        request_id,
    )
    return header + payload_bytes


# 将单个SEQPACKET数据包解码为IPC消息
def decode_packet(packet: bytes) -> IPCPacket:
    packet = bytes(packet)
    if len(packet) < IPC_HEADER_SIZE:
        raise IPCProtocolError("IPC packet is shorter than its header")

    magic, version, message_value, flags, payload_size, request_id = (
        IPC_HEADER_STRUCT.unpack_from(packet)
    )
    if magic != IPC_MAGIC:
        raise IPCProtocolError(f"invalid IPC magic: {magic!r}")
    if version != IPC_VERSION:
        raise IPCProtocolError(f"unsupported IPC version: {version}")
    if flags != 0:
        raise IPCProtocolError(f"unsupported IPC flags: {flags}")
    if payload_size > IPC_MAX_PAYLOAD_SIZE:
        raise IPCProtocolError(f"IPC payload is too large: {payload_size}")
    if len(packet) != IPC_HEADER_SIZE + payload_size:
        raise IPCProtocolError("IPC packet length does not match its header")

    try:
        message_type = IPCMessageType(message_value)
    except ValueError as error:
        raise IPCProtocolError(
            f"unknown IPC message type: 0x{message_value:02x}"
        ) from error

    payload = decode_json_payload(packet[IPC_HEADER_SIZE:])
    return IPCPacket(message_type, request_id, payload)


# 检查对象字段集合
def require_fields(
    values: Mapping[str, Any],
    required: set[str],
    *,
    optional: set[str] | None = None,
) -> None:
    optional = set() if optional is None else optional
    actual = set(values)
    missing = required - actual
    unknown = actual - required - optional
    if missing:
        raise IPCProtocolError(
            f"missing IPC field: {sorted(missing)[0]}"
        )
    if unknown:
        raise IPCProtocolError(
            f"unknown IPC field: {sorted(unknown)[0]}"
        )


# 将输入转换为六轴有限浮点数
def six_finite_values(
    name: str,
    values: Sequence[float],
    *,
    require_positive: bool = False,
) -> tuple[float, ...]:
    if isinstance(values, (str, bytes)):
        raise IPCProtocolError(f"{name} must contain exactly 6 values")
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError, OverflowError) as error:
        raise IPCProtocolError(
            f"{name} must contain exactly 6 finite values"
        ) from error
    if len(result) != 6:
        raise IPCProtocolError(f"{name} must contain exactly 6 values")
    if not all(math.isfinite(value) for value in result):
        raise IPCProtocolError(f"{name} must contain only finite values")
    if require_positive and not all(value > 0.0 for value in result):
        raise IPCProtocolError(f"{name} values must be positive")
    return result


# 生成经过校验的servoJ负载
def make_servoj_payload(
    sequence: int,
    target_q: Sequence[float],
    speed_limits: float | Sequence[float],
) -> dict[str, Any]:
    sequence = validate_uint("sequence", sequence, 32)
    target = six_finite_values("target_q", target_q)

    if isinstance(speed_limits, (int, float)) and not isinstance(
        speed_limits, bool
    ):
        speeds = (float(speed_limits),) * 6
    else:
        speeds = six_finite_values(
            "speed_limits",
            speed_limits,
            require_positive=True,
        )
    speeds = six_finite_values(
        "speed_limits",
        speeds,
        require_positive=True,
    )
    return {
        "sequence": sequence,
        "target_q": list(target),
        "speed_limits": list(speeds),
    }


# 解析并校验servoJ负载
def parse_servoj_payload(
    values: Mapping[str, Any],
) -> tuple[int, tuple[float, ...], tuple[float, ...]]:
    require_fields(values, {"sequence", "target_q", "speed_limits"})
    sequence = validate_uint("sequence", values["sequence"], 32)
    target_q = six_finite_values("target_q", values["target_q"])
    speed_limits = six_finite_values(
        "speed_limits",
        values["speed_limits"],
        require_positive=True,
    )
    return sequence, target_q, speed_limits


# 将控制守护进程快照转换为IPC负载
def snapshot_to_payload(snapshot: Any) -> dict[str, Any]:
    timestamp_ns = validate_uint("timestamp_ns", snapshot.timestamp_ns, 64)
    daemon_state = RobotMode(int(snapshot.daemon_state))
    safety_state = SafetyState(int(snapshot.safety_state))
    actual_q = six_finite_values("actual_q", snapshot.actual_q)
    actual_qd = six_finite_values("actual_qd", snapshot.actual_qd)
    actual_tau = six_finite_values("actual_tau", snapshot.actual_tau)
    target_q = six_finite_values("target_q", snapshot.target_q)
    last_sequence = validate_uint(
        "last_input_sequence",
        snapshot.last_input_sequence,
        32,
    )
    fault_code = validate_uint("fault_code", snapshot.fault_code, 32)
    fault_message = str(snapshot.fault_message)
    return {
        "timestamp_ns": timestamp_ns,
        "daemon_state": int(daemon_state),
        "safety_state": int(safety_state),
        "actual_q": list(actual_q),
        "actual_qd": list(actual_qd),
        "actual_tau": list(actual_tau),
        "target_q": list(target_q),
        "last_input_sequence": last_sequence,
        "fault_code": fault_code,
        "fault_message": fault_message,
    }


# 将IPC负载转换为不可变快照
def parse_snapshot_payload(values: Mapping[str, Any]) -> IPCSnapshot:
    required = {
        "timestamp_ns",
        "daemon_state",
        "safety_state",
        "actual_q",
        "actual_qd",
        "actual_tau",
        "target_q",
        "last_input_sequence",
        "fault_code",
        "fault_message",
    }
    require_fields(values, required)

    try:
        daemon_state = RobotMode(
            validate_uint("daemon_state", values["daemon_state"], 8)
        )
        safety_state = SafetyState(
            validate_uint("safety_state", values["safety_state"], 8)
        )
    except ValueError as error:
        raise IPCProtocolError("unknown daemon or safety state") from error

    if not isinstance(values["fault_message"], str):
        raise IPCProtocolError("fault_message must be a string")

    return IPCSnapshot(
        timestamp_ns=validate_uint("timestamp_ns", values["timestamp_ns"], 64),
        daemon_state=daemon_state,
        safety_state=safety_state,
        actual_q=six_finite_values("actual_q", values["actual_q"]),
        actual_qd=six_finite_values("actual_qd", values["actual_qd"]),
        actual_tau=six_finite_values("actual_tau", values["actual_tau"]),
        target_q=six_finite_values("target_q", values["target_q"]),
        last_input_sequence=validate_uint(
            "last_input_sequence",
            values["last_input_sequence"],
            32,
        ),
        fault_code=validate_uint("fault_code", values["fault_code"], 32),
        fault_message=values["fault_message"],
    )


# 生成统一错误负载
def make_error_payload(code: str, message: str) -> dict[str, str]:
    if not code or not isinstance(code, str):
        raise IPCProtocolError("error code must be a non-empty string")
    return {"code": code, "message": str(message)}
