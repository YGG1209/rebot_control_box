from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass, replace
from typing import Any

import numpy as np

from reBotArm_control_py.controllers import RebotArmController

from reBotArm_control_box.protocol import (
    RobotMode,
    SafetyState,
    is_newer_sequence,
)


# 控制层沿用协议公开的机器人状态
DaemonState = RobotMode


# 状态机允许的转换
ALLOWED_TRANSITIONS = {
    DaemonState.DISCONNECTED: {DaemonState.CONNECTING},
    DaemonState.CONNECTING: {
        DaemonState.IDLE,
        DaemonState.FAULT,
        DaemonState.STOPPING,
    },
    DaemonState.IDLE: {
        DaemonState.RUNNING,
        DaemonState.FAULT,
        DaemonState.STOPPING,
    },
    DaemonState.RUNNING: {
        DaemonState.IDLE,
        DaemonState.FAULT,
        DaemonState.STOPPING,
    },
    DaemonState.FAULT: {
        DaemonState.STOPPING,
        DaemonState.DISCONNECTED,
    },
    DaemonState.STOPPING: {
        DaemonState.DISCONNECTED,
        DaemonState.FAULT,
    },
}


@dataclass(frozen=True, slots=True)
class ServoJCommand:
    sequence: int
    target_q: tuple[float, ...]
    speed_limits: tuple[float, ...]
    received_at_ns: int


@dataclass(frozen=True, slots=True)
class RobotSnapshot:
    timestamp_ns: int
    daemon_state: DaemonState
    safety_state: SafetyState
    actual_q: tuple[float, ...]
    actual_qd: tuple[float, ...]
    actual_tau: tuple[float, ...]
    target_q: tuple[float, ...]
    last_input_sequence: int
    fault_code: int
    fault_message: str


class LatestCommandBuffer:
    # 初始化线程安全命令缓存
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: ServoJCommand | None = None
        self._last_sequence: int | None = None

    # 保存比上一条更新的命令
    def put(self, command: ServoJCommand) -> bool:
        with self._lock:
            if (
                self._last_sequence is not None
                and not is_newer_sequence(command.sequence, self._last_sequence)
            ):
                return False
            self._latest = command
            self._last_sequence = command.sequence
            return True

    # 返回当前最新命令
    def latest(self) -> ServoJCommand | None:
        with self._lock:
            return self._latest

    # 丢弃指定的活动命令并保留序号
    def expire(self, sequence: int) -> None:
        with self._lock:
            if self._latest is not None and self._latest.sequence == sequence:
                self._latest = None

    # 清空活动命令并保留序号
    def clear_active(self) -> None:
        with self._lock:
            self._latest = None

    # 清空命令并重新开始序号会话
    def reset(self) -> None:
        with self._lock:
            self._latest = None
            self._last_sequence = None


class ControlDaemon:
    # 初始化状态机、命令缓存和看门狗
    def __init__(
        self,
        controller_factory: Callable[[], Any] | None = None,
        *,
        control_rate_hz: float = 60.0,
        watchdog_timeout_s: float = 0.1,
        feedback_timeout_s: float = 10.0,
        feedback_stale_timeout_s: float = 0.2,
        hold_speed_rad_s: float = 0.1,
        max_speed_rad_s: float | Sequence[float] = 1.57,
        max_target_error_rad: float | Sequence[float] = 1.0,
        joint_limits: Sequence[Sequence[float]] | None = None,
        clock_ns: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        if not math.isfinite(control_rate_hz) or control_rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be finite and positive")
        if not math.isfinite(watchdog_timeout_s) or watchdog_timeout_s <= 0.0:
            raise ValueError("watchdog_timeout_s must be finite and positive")
        if not math.isfinite(feedback_timeout_s) or feedback_timeout_s <= 0.0:
            raise ValueError("feedback_timeout_s must be finite and positive")
        if (
            not math.isfinite(feedback_stale_timeout_s)
            or feedback_stale_timeout_s <= 0.0
        ):
            raise ValueError(
                "feedback_stale_timeout_s must be finite and positive"
            )
        if not math.isfinite(hold_speed_rad_s) or hold_speed_rad_s <= 0.0:
            raise ValueError("hold_speed_rad_s must be finite and positive")

        self._controller_factory = (
            controller_factory
            if controller_factory is not None
            else lambda: RebotArmController("posvel")
        )
        self._control_period_ns = int(1_000_000_000 / control_rate_hz)
        self._watchdog_timeout_ns = int(watchdog_timeout_s * 1_000_000_000)
        self._feedback_timeout_s = feedback_timeout_s
        self._feedback_stale_timeout_ns = int(
            feedback_stale_timeout_s * 1_000_000_000
        )
        self._hold_speed = (float(hold_speed_rad_s),) * 6
        self._max_speed = self._as_six_values(
            max_speed_rad_s,
            "max_speed_rad_s",
            allow_scalar=True,
            require_positive=True,
        )
        self._max_target_error = self._as_six_values(
            max_target_error_rad,
            "max_target_error_rad",
            allow_scalar=True,
            require_positive=True,
        )
        self._configured_joint_limits = (
            None
            if joint_limits is None
            else self._as_joint_limits(joint_limits)
        )
        self._joint_limits = self._configured_joint_limits
        self._clock_ns = clock_ns

        self._lifecycle_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._step_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._controller: Any | None = None
        self._commands = LatestCommandBuffer()
        self._hold_target = (0.0,) * 6
        self._last_feedback_timestamp: float | None = None
        self._last_feedback_progress_ns: int | None = None

        self._snapshot = RobotSnapshot(
            timestamp_ns=self._clock_ns(),
            daemon_state=DaemonState.DISCONNECTED,
            safety_state=SafetyState.NORMAL,
            actual_q=(0.0,) * 6,
            actual_qd=(0.0,) * 6,
            actual_tau=(0.0,) * 6,
            target_q=(0.0,) * 6,
            last_input_sequence=0,
            fault_code=0,
            fault_message="",
        )

    # 返回当前状态机状态
    @property
    def state(self) -> DaemonState:
        with self._state_lock:
            return self._snapshot.daemon_state

    # 返回当前安全状态
    @property
    def safety_state(self) -> SafetyState:
        with self._state_lock:
            return self._snapshot.safety_state

    # 返回不可变状态快照
    def get_snapshot(self) -> RobotSnapshot:
        with self._state_lock:
            return self._snapshot

    # 连接机械臂并启动控制循环
    def start(self) -> None:
        with self._lifecycle_lock:
            if self.state is not DaemonState.DISCONNECTED:
                raise RuntimeError(f"cannot start from state {self.state.name}")

            self._transition(DaemonState.CONNECTING)
            controller = self._controller_factory()
            self._controller = controller

            try:
                controller.connect()
                self._joint_limits = (
                    self._configured_joint_limits
                    if self._configured_joint_limits is not None
                    else self._extract_joint_limits(controller)
                )
                self._last_feedback_timestamp = None
                self._last_feedback_progress_ns = None
                actual_q, actual_qd, actual_tau = self._wait_for_feedback(controller)
                self._hold_target = actual_q
                self._commands.reset()
                controller.servoJ(
                    np.asarray(self._hold_target),
                    np.asarray(self._hold_speed),
                )
                self._set_safety(SafetyState.NORMAL)
                self._transition(DaemonState.IDLE)
                self._publish_snapshot(
                    actual_q,
                    actual_qd,
                    actual_tau,
                    self._hold_target,
                )

                self._stop_event.clear()
                self._thread = threading.Thread(
                    target=self._control_loop,
                    name="rebot-control-daemon",
                    daemon=True,
                )
                self._thread.start()
            except Exception as error:
                self._record_fault(error)
                try:
                    controller.disconnect()
                except Exception:
                    pass
                self._controller = None
                raise

    # 停止控制循环并断开机械臂
    def shutdown(self) -> None:
        with self._lifecycle_lock:
            if self.state is DaemonState.DISCONNECTED:
                return

            if self.state is not DaemonState.STOPPING:
                self._transition(DaemonState.STOPPING)

            self._stop_event.set()
            thread = self._thread
            if (
                thread is not None
                and thread.is_alive()
                and thread is not threading.current_thread()
            ):
                thread.join()
            self._thread = None

            controller = self._controller
            try:
                if controller is not None:
                    controller.disconnect()
            except Exception as error:
                self._record_fault(error)
                raise
            finally:
                self._controller = None
                self._commands.reset()

            self._set_safety(SafetyState.NORMAL)
            self._transition(DaemonState.DISCONNECTED)

    # 提交一条最新servoJ命令
    def submit_servoj(
        self,
        sequence: int,
        target_q: Sequence[float],
        speed_limits: float | Sequence[float],
    ) -> bool:
        if self.state not in (DaemonState.IDLE, DaemonState.RUNNING):
            raise RuntimeError(f"cannot submit command in state {self.state.name}")

        target = self._as_six_values(target_q, "target_q")
        speeds = self._as_six_values(
            speed_limits,
            "speed_limits",
            allow_scalar=True,
            require_positive=True,
        )
        if any(
            speed > maximum
            for speed, maximum in zip(speeds, self._max_speed)
        ):
            raise ValueError("speed_limits exceeds configured maximum")

        actual_q = self.get_snapshot().actual_q
        if any(
            abs(goal - actual) > maximum
            for goal, actual, maximum in zip(
                target,
                actual_q,
                self._max_target_error,
            )
        ):
            raise ValueError("target_q is too far from current actual_q")
        self._validate_joint_limits(target)

        command = ServoJCommand(
            sequence=int(sequence),
            target_q=target,
            speed_limits=speeds,
            received_at_ns=self._clock_ns(),
        )
        return self._commands.put(command)

    # 暂停运动并保持当前关节位置
    def pause(self) -> None:
        if self.state not in (DaemonState.IDLE, DaemonState.RUNNING):
            raise RuntimeError(f"cannot pause in state {self.state.name}")

        with self._step_lock:
            self._commands.clear_active()
            actual_q, actual_qd, actual_tau = self._read_feedback()
            self._hold_target = actual_q
            self._set_safety(SafetyState.NORMAL)
            if self.state is DaemonState.RUNNING:
                self._transition(DaemonState.IDLE)
            self._publish_snapshot(
                actual_q,
                actual_qd,
                actual_tau,
                self._hold_target,
            )

    # 重置远程控制会话及输入序号
    def reset_session(self) -> None:
        self.pause()
        self._commands.reset()

    # 在指定状态之间执行合法转换
    def _transition(self, new_state: DaemonState) -> None:
        with self._state_lock:
            current = self._snapshot.daemon_state
            if new_state is current:
                return
            if new_state not in ALLOWED_TRANSITIONS[current]:
                raise RuntimeError(
                    f"invalid daemon transition: {current.name} -> {new_state.name}"
                )
            self._snapshot = replace(
                self._snapshot,
                timestamp_ns=self._clock_ns(),
                daemon_state=new_state,
            )

    # 更新安全状态
    def _set_safety(self, safety_state: SafetyState) -> None:
        with self._state_lock:
            self._snapshot = replace(
                self._snapshot,
                timestamp_ns=self._clock_ns(),
                safety_state=safety_state,
            )

    # 记录控制故障并进入FAULT
    def _record_fault(self, error: Exception) -> None:
        with self._state_lock:
            self._snapshot = replace(
                self._snapshot,
                timestamp_ns=self._clock_ns(),
                daemon_state=DaemonState.FAULT,
                safety_state=SafetyState.CONTROL_FAULT,
                fault_code=1,
                fault_message=str(error),
            )

    # 等待第一帧有效反馈
    def _wait_for_feedback(
        self,
        controller: Any,
    ) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]]:
        deadline = time.monotonic() + self._feedback_timeout_s
        feedback = controller.rebotarm.arm_state.feedback

        while feedback.timestamp == 0.0:
            if time.monotonic() >= deadline:
                raise TimeoutError("feedback did not become ready")
            time.sleep(0.01)

        return self._read_feedback()

    # 读取并检查六轴反馈
    def _read_feedback(
        self,
    ) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]]:
        controller = self._require_controller()
        feedback = controller.rebotarm.arm_state.feedback
        self._check_feedback_timestamp(feedback.timestamp)
        actual_q = self._as_six_values(feedback.arm.position, "actual_q")
        actual_qd = self._as_six_values(feedback.arm.velocity, "actual_qd")
        actual_tau = self._as_six_values(feedback.arm.torque, "actual_tau")
        return actual_q, actual_qd, actual_tau

    # 检查反馈时间戳持续推进且未超过停滞阈值
    def _check_feedback_timestamp(self, value: Any) -> None:
        timestamp = float(value)
        if not math.isfinite(timestamp) or timestamp <= 0.0:
            raise RuntimeError("feedback timestamp is invalid")

        now_ns = self._clock_ns()
        previous = self._last_feedback_timestamp
        if previous is None or timestamp > previous:
            self._last_feedback_timestamp = timestamp
            self._last_feedback_progress_ns = now_ns
            return
        if timestamp < previous:
            raise RuntimeError("feedback timestamp moved backwards")

        last_progress_ns = self._last_feedback_progress_ns
        if (
            last_progress_ns is not None
            and now_ns - last_progress_ns > self._feedback_stale_timeout_ns
        ):
            raise TimeoutError("feedback timestamp is stale")

    # 从真实控制器模型提取前六轴有限软限位
    @staticmethod
    def _extract_joint_limits(
        controller: Any,
    ) -> tuple[tuple[float | None, float | None], ...] | None:
        model = getattr(controller, "_model", None)
        if model is None:
            return None

        lower = np.asarray(model.lowerPositionLimit, dtype=np.float64)
        upper = np.asarray(model.upperPositionLimit, dtype=np.float64)
        if lower.size < 6 or upper.size < 6:
            raise ValueError("robot model must provide limits for 6 joints")

        limits: list[tuple[float | None, float | None]] = []
        for low, high in zip(lower[:6], upper[:6]):
            finite_low = float(low) if math.isfinite(float(low)) else None
            finite_high = float(high) if math.isfinite(float(high)) else None
            if (
                finite_low is not None
                and finite_high is not None
                and finite_low >= finite_high
            ):
                raise ValueError("robot model contains invalid joint limits")
            limits.append((finite_low, finite_high))
        return tuple(limits)

    # 校验目标未越过当前启用的关节软限位
    def _validate_joint_limits(self, target: tuple[float, ...]) -> None:
        if self._joint_limits is None:
            return
        for index, (value, bounds) in enumerate(
            zip(target, self._joint_limits),
            start=1,
        ):
            lower, upper = bounds
            if lower is not None and value < lower:
                raise ValueError(f"target_q joint {index} is below soft limit")
            if upper is not None and value > upper:
                raise ValueError(f"target_q joint {index} is above soft limit")

    # 控制异常时尽力保持最后一帧实际位置
    def _best_effort_hold_snapshot(self) -> None:
        snapshot = self.get_snapshot()
        self._hold_target = snapshot.actual_q
        try:
            controller = self._require_controller()
            controller.servoJ(
                np.asarray(self._hold_target),
                np.asarray(self._hold_speed),
            )
        except Exception:
            pass

    # 返回已连接控制器
    def _require_controller(self) -> Any:
        if self._controller is None:
            raise RuntimeError("robot controller is not connected")
        return self._controller

    # 执行固定频率控制循环
    def _control_loop(self) -> None:
        next_cycle_ns = self._clock_ns()

        while not self._stop_event.is_set():
            try:
                with self._step_lock:
                    self._control_step(self._clock_ns())
            except Exception as error:
                self._best_effort_hold_snapshot()
                self._record_fault(error)
                self._stop_event.set()
                break

            next_cycle_ns += self._control_period_ns
            wait_s = (next_cycle_ns - self._clock_ns()) / 1_000_000_000
            if wait_s > 0.0:
                self._stop_event.wait(wait_s)
            else:
                next_cycle_ns = self._clock_ns()

    # 执行一次命令应用和看门狗检查
    def _control_step(self, now_ns: int) -> None:
        controller = self._require_controller()
        actual_q, actual_qd, actual_tau = self._read_feedback()
        command = self._commands.latest()

        if (
            command is not None
            and now_ns - command.received_at_ns <= self._watchdog_timeout_ns
        ):
            controller.servoJ(
                np.asarray(command.target_q),
                np.asarray(command.speed_limits),
            )
            self._hold_target = command.target_q
            self._set_safety(SafetyState.NORMAL)
            if self.state is DaemonState.IDLE:
                self._transition(DaemonState.RUNNING)
            target_q = command.target_q
            last_sequence = command.sequence
        else:
            last_sequence = self.get_snapshot().last_input_sequence
            if command is not None:
                self._commands.expire(command.sequence)
                self._hold_target = actual_q
                self._set_safety(SafetyState.COMMAND_TIMEOUT)
                if self.state is DaemonState.RUNNING:
                    self._transition(DaemonState.IDLE)

            controller.servoJ(
                np.asarray(self._hold_target),
                np.asarray(self._hold_speed),
            )
            target_q = self._hold_target

        self._publish_snapshot(
            actual_q,
            actual_qd,
            actual_tau,
            target_q,
            last_sequence,
        )

    # 发布控制箱状态快照
    def _publish_snapshot(
        self,
        actual_q: tuple[float, ...],
        actual_qd: tuple[float, ...],
        actual_tau: tuple[float, ...],
        target_q: tuple[float, ...],
        last_input_sequence: int | None = None,
    ) -> None:
        with self._state_lock:
            sequence = (
                self._snapshot.last_input_sequence
                if last_input_sequence is None
                else int(last_input_sequence)
            )
            self._snapshot = replace(
                self._snapshot,
                timestamp_ns=self._clock_ns(),
                actual_q=actual_q,
                actual_qd=actual_qd,
                actual_tau=actual_tau,
                target_q=target_q,
                last_input_sequence=sequence,
                fault_code=0,
                fault_message="",
            )

    # 将标量或数组转换为六轴有限数
    @staticmethod
    def _as_six_values(
        values: float | Sequence[float],
        name: str,
        *,
        allow_scalar: bool = False,
        require_positive: bool = False,
    ) -> tuple[float, ...]:
        if allow_scalar and np.isscalar(values):
            array = np.full(6, float(values), dtype=np.float64)
        else:
            array = np.asarray(values, dtype=np.float64)

        if array.shape != (6,):
            raise ValueError(f"{name} must contain exactly 6 values")
        if not np.all(np.isfinite(array)):
            raise ValueError(f"{name} must contain only finite values")
        if require_positive and np.any(array <= 0.0):
            raise ValueError(f"{name} values must be positive")
        return tuple(float(value) for value in array)

    # 将显式关节限位转换为六组有限上下界
    @staticmethod
    def _as_joint_limits(
        limits: Sequence[Sequence[float]],
    ) -> tuple[tuple[float, float], ...]:
        array = np.asarray(limits, dtype=np.float64)
        if array.shape != (6, 2):
            raise ValueError("joint_limits must contain 6 lower/upper pairs")
        if not np.all(np.isfinite(array)):
            raise ValueError("joint_limits must contain only finite values")
        if np.any(array[:, 0] >= array[:, 1]):
            raise ValueError(
                "joint_limits lower bounds must be below upper bounds"
            )
        return tuple((float(low), float(high)) for low, high in array)


# 直接启动守护进程并等待退出信号
def main() -> None:
    daemon = ControlDaemon()
    daemon.start()
    print("控制守护进程已启动，按 Ctrl+C 停止")

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n控制守护进程正在停止")
    finally:
        daemon.shutdown()


if __name__ == "__main__":
    main()

