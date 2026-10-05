from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping


@dataclass(frozen=True, slots=True)
class ControlLease:
    """独占控制权信息。"""

    control_session_id: int | str
    lease_ms: int


@dataclass(frozen=True, slots=True)
class RobotState:
    """控制箱最新状态。"""

    frame_sequence: int
    controller_timestamp_ns: int
    received_at_ns: int
    values: Mapping[str, Any]

    # 按字段名读取状态
    def __getitem__(self, name: str) -> Any:
        return self.values[name]

    # 读取状态并提供默认值
    def get(self, name: str, default: Any = None) -> Any:
        return self.values.get(name, default)
