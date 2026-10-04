from __future__ import annotations

import argparse
import math
import sys
import time
from collections.abc import Callable, Sequence

from reBotArm_control_box.network_client import (
    DEFAULT_CONTROLLER_HOST,
    DEFAULT_CONTROLLER_PORT,
    RebotNetworkClient,
    RobotState,
)
from reBotArm_control_box.protocol import RobotMode, SafetyState


STREAM_FREQUENCY_HZ = 60.0
HOLD_DURATION_S = 2.0


# 校验正浮点命令行参数
def positive_float(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise argparse.ArgumentTypeError("must be a finite positive number")
    return number


# 校验关节编号
def joint_number(value: str) -> int:
    number = int(value)
    if number < 1 or number > 6:
        raise argparse.ArgumentTypeError("must be between 1 and 6")
    return number


# 检查状态是否已进入故障
def check_robot_state(state: RobotState, *, check_timeout: bool) -> None:
    fault_code = int(state.get("fault_code", 0))
    robot_mode = int(state.get("robot_mode", RobotMode.IDLE))
    safety_state = int(state.get("safety_state", SafetyState.NORMAL))
    if fault_code != 0 or robot_mode == int(RobotMode.FAULT):
        raise RuntimeError(f"控制箱报告故障，fault_code={fault_code}")
    if check_timeout and safety_state != int(SafetyState.NORMAL):
        name = SafetyState(safety_state).name
        raise RuntimeError(f"控制箱安全状态异常: {name}")


# 按60 Hz发送一段目标轨迹
def stream_targets(
    client: RebotNetworkClient,
    duration_s: float,
    target_at: Callable[[float], Sequence[float]],
    speed_rad_s: float,
) -> int:
    period_s = 1.0 / STREAM_FREQUENCY_HZ
    started_at = time.monotonic()
    deadline = started_at
    last_sequence: int | None = None

    while True:
        now = time.monotonic()
        elapsed = now - started_at
        if elapsed >= duration_s:
            break

        last_sequence = client.send_servoj(
            target_at(elapsed),
            speed_rad_s,
        )
        state = client.latest_state
        if state is not None:
            check_robot_state(state, check_timeout=elapsed > 0.25)

        deadline += period_s
        wait_s = deadline - time.monotonic()
        if wait_s > 0.0:
            time.sleep(wait_s)
        elif wait_s < -period_s:
            deadline = time.monotonic()

    if last_sequence is None:
        raise RuntimeError("trajectory duration produced no command")
    return last_sequence


# 尽力执行退出阶段的安全请求
def safe_cleanup(name: str, operation: Callable[[], None]) -> None:
    try:
        operation()
    except Exception as error:
        print(f"警告：{name}失败：{error}", file=sys.stderr)


# 运行上层主机servoJ真机示例
def run_demo(arguments: argparse.Namespace) -> None:
    amplitude_rad = math.radians(arguments.amplitude_deg)
    angular_frequency = arguments.speed / amplitude_rad
    joint_index = arguments.joint - 1

    print("本程序会使机械臂运动，请先完成以下检查：")
    print("1. 清空机械臂工作空间，并确认关节运动不会碰撞。")
    print("2. 急停按钮可立即触及，现场有人持续监护。")
    print("3. MiniPC控制守护进程和网络服务已经启动。")
    input("确认安全条件满足后按 Enter 连接；Ctrl+C 取消: ")

    client = RebotNetworkClient(
        arguments.host,
        arguments.port,
        client_name="rebot-servoj-demo",
    )
    try:
        print(f"正在连接 {arguments.host}:{arguments.port} ...")
        client.connect()
        client.setup_outputs()
        client.setup_inputs()
        lease = client.acquire_control(requested_lease_ms=1000)
        print(
            "已获得控制权："
            f"session={lease.control_session_id}, lease={lease.lease_ms} ms"
        )
        client.start()

        first_state = client.wait_for_state(timeout_s=3.0)
        actual_q_value = first_state.get("actual_q")
        if not isinstance(actual_q_value, Sequence) or len(actual_q_value) != 6:
            raise RuntimeError("状态输出没有有效的actual_q")
        initial_q = tuple(float(value) for value in actual_q_value)
        initial_deg = [round(math.degrees(value), 3) for value in initial_q]
        print(f"当前关节角(deg): {initial_deg}")

        print(f"以60 Hz保持当前位置 {HOLD_DURATION_S:.1f} 秒...")
        last_sequence = stream_targets(
            client,
            HOLD_DURATION_S,
            lambda _: initial_q,
            arguments.speed,
        )
        client.wait_until_applied(last_sequence, timeout_s=0.5)

        print(
            f"关节{arguments.joint}开始正弦往返："
            f"幅值={arguments.amplitude_deg:g} deg，"
            f"最大目标速度={arguments.speed:g} rad/s，"
            f"持续={arguments.duration:g} s"
        )

        def sine_target(elapsed_s: float) -> tuple[float, ...]:
            target = list(initial_q)
            target[joint_index] += amplitude_rad * math.sin(
                angular_frequency * elapsed_s
            )
            return tuple(target)

        last_sequence = stream_targets(
            client,
            arguments.duration,
            sine_target,
            arguments.speed,
        )
        client.wait_until_applied(last_sequence, timeout_s=0.5)
        print("运动段完成，正在暂停并保持当前位置。")
    finally:
        if client.started:
            safe_cleanup("PAUSE", client.pause)
        if client.has_control:
            safe_cleanup("RELEASE_CONTROL", client.release_control)
        client.close()


# 解析命令行并启动示例
def main() -> None:
    parser = argparse.ArgumentParser(
        description="reBot上层主机60 Hz servoJ真机测试",
    )
    parser.add_argument("--host", default=DEFAULT_CONTROLLER_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_CONTROLLER_PORT)
    parser.add_argument("--joint", type=joint_number, default=1)
    parser.add_argument(
        "--amplitude-deg",
        type=positive_float,
        default=2.0,
    )
    parser.add_argument("--speed", type=positive_float, default=0.1)
    parser.add_argument("--duration", type=positive_float, default=10.0)
    arguments = parser.parse_args()

    try:
        run_demo(arguments)
    except KeyboardInterrupt:
        print("\n用户取消，已执行安全退出。")
    except Exception as error:
        print(f"测试失败：{error}", file=sys.stderr)
        raise SystemExit(1) from error


if __name__ == "__main__":
    main()
