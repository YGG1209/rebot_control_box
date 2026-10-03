from __future__ import annotations

import json
import math
import struct
import time
from dataclasses import dataclass
from enum import Enum, IntEnum
from typing import Any, Iterable, Mapping


# 协议基础参数
MAGIC = b"RBOT"
PROTOCOL_VERSION = 1
SUPPORTED_VERSIONS = (PROTOCOL_VERSION,)
MAX_PAYLOAD_SIZE = 64 * 1024

# magic, version, type, flags, payload length, sequence, monotonic timestamp
HEADER_STRUCT = struct.Struct("!4sBBHIIQ")
HEADER_SIZE = HEADER_STRUCT.size


class ProtocolError(ValueError):
    pass


class MessageType(IntEnum):
    HELLO = 0x01
    HELLO_ACK = 0x02

    SETUP_INPUTS = 0x10
    SETUP_INPUTS_ACK = 0x11
    SETUP_OUTPUTS = 0x12
    SETUP_OUTPUTS_ACK = 0x13

    ACQUIRE_CONTROL = 0x20
    ACQUIRE_CONTROL_ACK = 0x21
    RELEASE_CONTROL = 0x22
    RELEASE_CONTROL_ACK = 0x23

    START = 0x30
    START_ACK = 0x31
    PAUSE = 0x32
    PAUSE_ACK = 0x33

    INPUT_DATA = 0x40
    OUTPUT_DATA = 0x41

    HEARTBEAT = 0x50
    HEARTBEAT_ACK = 0x51
    TEXT_MESSAGE = 0x60
    ERROR = 0x7F


class ControlMode(IntEnum):
    IDLE = 0
    SERVOJ = 1
    SERVOL = 2


class WireType(Enum):
    BOOL = ("BOOL", "?")
    UINT8 = ("UINT8", "B")
    UINT32 = ("UINT32", "I")
    UINT64 = ("UINT64", "Q")
    INT32 = ("INT32", "i")
    DOUBLE = ("DOUBLE", "d")
    VECTOR6D = ("VECTOR6D", "6d")
    VECTOR7D = ("VECTOR7D", "7d")

    # 初始化类型标签和二进制编码器
    def __init__(self, label: str, format_code: str) -> None:
        self.label = label
        self.codec = struct.Struct(f"!{format_code}")


@dataclass(frozen=True, slots=True)
class FieldSpec:
    wire_type: WireType
    description: str
    unit: str = ""


# 上层主机可以写入的字段
INPUT_FIELDS: dict[str, FieldSpec] = {
    "control_mode": FieldSpec(WireType.UINT8, "控制模式"),
    "target_q": FieldSpec(WireType.VECTOR6D, "目标关节位置", "rad"),
    "target_qd": FieldSpec(WireType.VECTOR6D, "目标关节速度", "rad/s"),
    "target_pose": FieldSpec(
        WireType.VECTOR7D,
        "目标TCP位姿[x,y,z,qx,qy,qz,qw]",
        "m,quaternion",
    ),
    "speed_limits": FieldSpec(WireType.VECTOR6D, "各关节速度限制", "rad/s"),
    "speed_scale": FieldSpec(WireType.DOUBLE, "全局速度比例", "0..1"),
}


# 控制箱可以输出的字段
OUTPUT_FIELDS: dict[str, FieldSpec] = {
    "controller_time_ns": FieldSpec(WireType.UINT64, "控制箱单调时钟", "ns"),
    "actual_q": FieldSpec(WireType.VECTOR6D, "实际关节位置", "rad"),
    "actual_qd": FieldSpec(WireType.VECTOR6D, "实际关节速度", "rad/s"),
    "actual_tau": FieldSpec(WireType.VECTOR6D, "实际关节力矩", "Nm"),
    "actual_tcp_pose": FieldSpec(
        WireType.VECTOR7D,
        "实际TCP位姿[x,y,z,qx,qy,qz,qw]",
        "m,quaternion",
    ),
    "target_q": FieldSpec(WireType.VECTOR6D, "当前目标关节位置", "rad"),
    "robot_mode": FieldSpec(WireType.UINT8, "机器人模式"),
    "safety_state": FieldSpec(WireType.UINT8, "安全状态"),
    "fault_code": FieldSpec(WireType.UINT32, "故障位掩码"),
    "last_input_sequence": FieldSpec(WireType.UINT32, "最近应用的输入序号"),
}


Recipe = tuple[tuple[str, FieldSpec], ...]


@dataclass(frozen=True, slots=True)
class Frame:
    version: int
    message_type: MessageType
    flags: int
    sequence: int
    timestamp_ns: int
    payload: bytes


# 检查整数是否符合无符号位宽
def _validate_uint(name: str, value: int, bits: int) -> int:
    value = int(value)
    if value < 0 or value >= 1 << bits:
        raise ValueError(f"{name} must fit in uint{bits}")
    return value


# 将消息编码为完整协议帧
def encode_frame(
    message_type: MessageType | int,
    payload: bytes = b"",
    *,
    sequence: int = 0,
    timestamp_ns: int | None = None,
    version: int = PROTOCOL_VERSION,
    flags: int = 0,
) -> bytes:
    payload = bytes(payload)
    if len(payload) > MAX_PAYLOAD_SIZE:
        raise ValueError(f"payload exceeds {MAX_PAYLOAD_SIZE} bytes")

    try:
        message_type = MessageType(int(message_type))
    except ValueError as error:
        raise ValueError(f"unknown message type: {message_type}") from error

    version = _validate_uint("version", version, 8)
    flags = _validate_uint("flags", flags, 16)
    sequence = _validate_uint("sequence", sequence, 32)
    timestamp_ns = (
        time.monotonic_ns()
        if timestamp_ns is None
        else _validate_uint("timestamp_ns", timestamp_ns, 64)
    )

    header = HEADER_STRUCT.pack(
        MAGIC,
        version,
        int(message_type),
        flags,
        len(payload),
        sequence,
        timestamp_ns,
    )
    return header + payload


class FrameDecoder:
    # 初始化TCP字节流缓存
    def __init__(self) -> None:
        self._buffer = bytearray()

    # 返回尚未组成完整帧的字节数
    @property
    def buffered_bytes(self) -> int:
        return len(self._buffer)

    # 清空解码缓存
    def reset(self) -> None:
        self._buffer.clear()

    # 接收新字节并解析所有完整帧
    def feed(self, data: bytes) -> list[Frame]:
        self._buffer.extend(data)
        frames: list[Frame] = []

        while len(self._buffer) >= HEADER_SIZE:
            (
                magic,
                version,
                message_value,
                flags,
                payload_length,
                sequence,
                timestamp_ns,
            ) = HEADER_STRUCT.unpack_from(self._buffer)

            if magic != MAGIC:
                self.reset()
                raise ProtocolError(f"invalid magic: {magic!r}")
            if version not in SUPPORTED_VERSIONS:
                self.reset()
                raise ProtocolError(f"unsupported version: {version}")
            if payload_length > MAX_PAYLOAD_SIZE:
                self.reset()
                raise ProtocolError(f"payload is too large: {payload_length}")

            try:
                message_type = MessageType(message_value)
            except ValueError as error:
                self.reset()
                raise ProtocolError(
                    f"unknown message type: 0x{message_value:02x}"
                ) from error

            frame_size = HEADER_SIZE + payload_length
            if len(self._buffer) < frame_size:
                break

            payload = bytes(self._buffer[HEADER_SIZE:frame_size])
            del self._buffer[:frame_size]
            frames.append(
                Frame(
                    version=version,
                    message_type=message_type,
                    flags=flags,
                    sequence=sequence,
                    timestamp_ns=timestamp_ns,
                    payload=payload,
                )
            )

        return frames


# 将控制消息编码为紧凑JSON字节
def encode_json_payload(values: Mapping[str, Any]) -> bytes:
    if not isinstance(values, Mapping):
        raise TypeError("JSON payload must be an object")
    return json.dumps(
        dict(values),
        ensure_ascii=False,
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")


# 将JSON字节解码为对象
def decode_json_payload(payload: bytes) -> dict[str, Any]:
    try:
        values = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ProtocolError("invalid JSON payload") from error
    if not isinstance(values, dict):
        raise ProtocolError("JSON payload must be an object")
    return values


# 直接生成包含JSON负载的协议帧
def encode_json_frame(
    message_type: MessageType | int,
    values: Mapping[str, Any],
    *,
    sequence: int = 0,
    timestamp_ns: int | None = None,
    version: int = PROTOCOL_VERSION,
    flags: int = 0,
) -> bytes:
    return encode_frame(
        message_type,
        encode_json_payload(values),
        sequence=sequence,
        timestamp_ns=timestamp_ns,
        version=version,
        flags=flags,
    )


# 检查字段并生成有序Recipe
def resolve_recipe(
    field_names: Iterable[str],
    registry: Mapping[str, FieldSpec],
) -> Recipe:
    recipe: list[tuple[str, FieldSpec]] = []
    seen: set[str] = set()

    for name in field_names:
        if not isinstance(name, str):
            raise ProtocolError("recipe field name must be a string")
        if name in seen:
            raise ProtocolError(f"duplicate recipe field: {name}")
        if name not in registry:
            raise ProtocolError(f"unknown recipe field: {name}")
        seen.add(name)
        recipe.append((name, registry[name]))

    if not recipe:
        raise ProtocolError("recipe must contain at least one field")
    return tuple(recipe)


# 返回Recipe对应的协议类型名称
def recipe_types(recipe: Recipe) -> list[str]:
    return [spec.wire_type.label for _, spec in recipe]


# 计算包含Recipe编号的负载长度
def recipe_payload_size(recipe: Recipe) -> int:
    return 1 + sum(spec.wire_type.codec.size for _, spec in recipe)


# 按字段类型编码单个值
def _pack_value(spec: FieldSpec, value: Any) -> bytes:
    wire_type = spec.wire_type

    try:
        if wire_type in (WireType.VECTOR6D, WireType.VECTOR7D):
            vector = tuple(float(item) for item in value)
            expected_size = 6 if wire_type is WireType.VECTOR6D else 7
            if len(vector) != expected_size:
                raise ProtocolError(
                    f"{wire_type.label} requires {expected_size} values"
                )
            if not all(math.isfinite(item) for item in vector):
                raise ProtocolError(f"{wire_type.label} contains non-finite value")
            return wire_type.codec.pack(*vector)

        if wire_type is WireType.DOUBLE:
            number = float(value)
            if not math.isfinite(number):
                raise ProtocolError("DOUBLE value must be finite")
            return wire_type.codec.pack(number)

        return wire_type.codec.pack(value)
    except ProtocolError:
        raise
    except (TypeError, ValueError, struct.error) as error:
        raise ProtocolError(f"cannot encode {wire_type.label} value") from error


# 按Recipe顺序编码实时数据
def pack_recipe_payload(
    recipe_id: int,
    recipe: Recipe,
    values: Mapping[str, Any],
) -> bytes:
    recipe_id = _validate_uint("recipe_id", recipe_id, 8)
    payload = bytearray((recipe_id,))

    for name, spec in recipe:
        if name not in values:
            raise ProtocolError(f"missing recipe value: {name}")
        payload.extend(_pack_value(spec, values[name]))

    return bytes(payload)


# 按Recipe顺序解码实时数据
def unpack_recipe_payload(
    payload: bytes,
    recipe: Recipe,
) -> tuple[int, dict[str, Any]]:
    expected_size = recipe_payload_size(recipe)
    if len(payload) != expected_size:
        raise ProtocolError(
            f"recipe payload size is {len(payload)}, expected {expected_size}"
        )

    recipe_id = payload[0]
    offset = 1
    values: dict[str, Any] = {}

    for name, spec in recipe:
        unpacked = spec.wire_type.codec.unpack_from(payload, offset)
        offset += spec.wire_type.codec.size

        if spec.wire_type in (WireType.VECTOR6D, WireType.VECTOR7D):
            if not all(math.isfinite(item) for item in unpacked):
                raise ProtocolError(f"{name} contains non-finite value")
            values[name] = unpacked
        else:
            value = unpacked[0]
            if spec.wire_type is WireType.DOUBLE and not math.isfinite(value):
                raise ProtocolError(f"{name} must be finite")
            values[name] = value

    return recipe_id, values


# 根据32位回绕规则判断序号是否更新
def is_newer_sequence(sequence: int, previous: int) -> bool:
    sequence = _validate_uint("sequence", sequence, 32)
    previous = _validate_uint("previous", previous, 32)
    difference = (sequence - previous) & 0xFFFFFFFF
    return 0 < difference < 0x80000000
