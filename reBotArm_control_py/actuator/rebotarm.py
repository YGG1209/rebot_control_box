"""reBotArm 分组控制系统 — JointGroup 架构。

配置驱动的硬件抽象层：
  - 所有参数均在 config/rebotarm.yaml 中定义（hardware_yaml 指定硬件配置文件）
  - 关节按 groups 分组，每组独立控制模式

使用示例::

    # 所有分组使用 POS_VEL 模式
    arm = RebotArm("posvel")
    arm.connect()
    arm.arm_state.command.arm.position[:] = joint_pos
    arm._joint_command_ready.set()

    arm.disconnect()
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import yaml

from motorbridge import Controller, Mode, CallError

from ..dynamics import compute_generalized_gravity, load_dynamics_model
from .buffer_type import ArmState

_CFG_DIR = Path(__file__).parent.parent.parent / "config"
_GLOBAL_CFG = _CFG_DIR / "rebotarm.yaml"


def _resolve_hw_cfg_path(hw_yaml: str | None = None) -> Path:
    if hw_yaml is None:
        if not _GLOBAL_CFG.exists():
            raise FileNotFoundError(f"{_GLOBAL_CFG} not found")
        data = yaml.safe_load(_GLOBAL_CFG.read_text())
        hw_yaml = data.get("hardware_yaml") if data else None
        if not hw_yaml:
            raise ValueError("hardware_yaml not set in rebotarm.yaml")

    p = Path(hw_yaml)
    if p.is_absolute():
        return p
    path = _CFG_DIR / hw_yaml
    if path.exists():
        return path
    raise FileNotFoundError(f"hardware config not found: {path}")


# --------------------------------------------------------------------------
# 配置加载
# --------------------------------------------------------------------------

@dataclass
class JointCfg:
    name: str
    motor_id: int
    feedback_id: int
    model: str
    vendor: str = "damiao"
    kp: float = 0.0
    kd: float = 0.0
    vel_kp: float = 0.0
    vel_ki: float = 0.0
    pos_kp: float = 0.0
    pos_ki: float = 0.0
    vlim: float = 0.0
    offset: float = 0.0
    gravity_k: float = 0.0
    tau_bias: float = 0.0
    friction: float = 0.0


def load_cfg(hw_yaml: str | None = None) -> dict:
    hw_path = _resolve_hw_cfg_path(hw_yaml)

    with open(hw_path, "r") as f:
        data = yaml.safe_load(f)

    joints = []
    for j in data.get("joints", []):
        mc = j.get("MIT", {})
        pc = j.get("POS_VEL", {})
        calibration = j.get("calibration", {})
        joints.append(JointCfg(
            name=j["name"],
            motor_id=int(j["motor_id"]),
            feedback_id=int(j["feedback_id"]),
            model=str(j.get("model", "4340P")),
            vendor=str(j.get("vendor", "damiao")).lower(),
            kp=float(mc.get("kp", 0.0)),
            kd=float(mc.get("kd", 0.0)),
            vel_kp=float(pc.get("vel_kp", 0.0)),
            vel_ki=float(pc.get("vel_ki", 0.0)),
            pos_kp=float(pc.get("pos_kp", 0.0)),
            pos_ki=float(pc.get("pos_ki", 0.0)),
            vlim=float(pc.get("vlim", 2.0)),
            offset=float(calibration.get("offset", 0.0)),
            gravity_k=float(calibration.get("gravity_k", 0.0)),
            tau_bias=float(calibration.get("tau_bias", 0.0)),
            friction=float(calibration.get("friction", 0.0)),
        ))

    return {
        "name": data.get("name", "reBotArm"),
        "channel": data.get("channel", "/dev/ttyACM0"),
        "comm_rate": float(data.get("comm_rate", 350.0)),
        "urdf_path": data.get("urdf_path"),
        "movel_rate": float(data.get("movel_rate", 10.0)),
        "servol_rate": float(data.get("servol_rate", 200.0)), 
        "groups": data.get("groups", {}),
        "joints": joints,
    }


# --------------------------------------------------------------------------
# NoOpGroup — 无执行器时的空操作桩
# --------------------------------------------------------------------------

class NoOpGroup:
    """当配置中不存在 gripper 组时的空实现。

    所有属性和方法与 JointGroup 接口兼容，但不对电机发送任何指令，
    方便用户代码在有/无夹爪时共用同一套逻辑，无需条件判断。
    """

    name: str = "gripper"
    _mode: str = "mit"

    @property
    def num_joints(self) -> int:
        return 0

    @property
    def joint_names(self) -> List[str]:
        return []

    @property
    def mode(self) -> str:
        return "mit"

    def enable(self) -> None:
        pass

    def disable(self) -> None:
        pass

    def mode_mit(self, kp=None, kd=None) -> bool:
        self._mode = "mit"
        return True

    def mode_pos_vel(self, vlim=None) -> bool:
        self._mode = "pos_vel"
        return True

    def mode_vel(self) -> bool:
        self._mode = "vel"
        return True

    def send_mit(self, pos, vel=None, kp=None, kd=None, tau=None) -> None:
        pass

    def send_pos_vel(self, pos, vlim=None) -> None:
        pass

    def send_vel(self, vel) -> None:
        pass

    def get_positions(self) -> np.ndarray:
        return np.array([], dtype=np.float64)

    def get_velocities(self) -> np.ndarray:
        return np.array([], dtype=np.float64)

    def __repr__(self) -> str:
        return "NoOpGroup(gripper, no actuator)"


# --------------------------------------------------------------------------
# JointGroup — 单组关节控制
# --------------------------------------------------------------------------

class JointGroup:
    """一组关节的独立控制器。

    每组拥有独立的控制模式（MIT / POS_VEL）、PID 参数和电机列表，
    可单独使能、切换模式、发送命令。

    由 RebotArm 通过 __getattr__ 代理访问，例如 arm.arm / arm.gripper。
    组内关节数量、顺序由配置决定。
    """

    def __init__(
        self,
        name: str,
        joint_names: List[str],
        all_joints: List[JointCfg],
        motor_map: Dict[str, any],
        ctrl_map: Dict[str, Controller],
    ) -> None:
        self.name = name
        self._jn: List[str] = joint_names
        self._jcfgs: List[JointCfg] = [
            next(j for j in all_joints if j.name == n) for n in joint_names
        ]
        self._mm: Dict[str, any] = motor_map
        self._cm: Dict[str, Controller] = ctrl_map
        self._mode: str = "mit"
        self._mit_kp: np.ndarray = np.array([j.kp for j in self._jcfgs], dtype=np.float64)
        self._mit_kd: np.ndarray = np.array([j.kd for j in self._jcfgs], dtype=np.float64)
        self._pv_vlim: np.ndarray = np.array([j.vlim for j in self._jcfgs], dtype=np.float64)
        self._gravity_offset = np.array([j.offset for j in self._jcfgs])
        self._gravity_k = np.array([j.gravity_k for j in self._jcfgs])
        self._tau_bias = np.array([j.tau_bias for j in self._jcfgs])
        self._friction = np.array([j.friction for j in self._jcfgs])

    # ── 属性 ────────────────────────────────────────────────────────────

    @property
    def num_joints(self) -> int:
        return len(self._jn)

    @property
    def joint_names(self) -> List[str]:
        return list(self._jn)

    @property
    def mode(self) -> str:
        return self._mode

    # ── 使能 / 失能 ────────────────────────────────────────────────────

    def enable(self) -> None:
        by_vendor: Dict[str, List[str]] = {}
        for jc in self._jcfgs:
            by_vendor.setdefault(jc.vendor, []).append(jc.name)
        for vendor in by_vendor:
            try:
                self._cm[vendor].enable_all()
            except CallError as e:
                print(f"[{self.name}/enable] {e}")
            time.sleep(0.05)

    def disable(self) -> None:
        by_vendor: Dict[str, List[str]] = {}
        for jc in self._jcfgs:
            by_vendor.setdefault(jc.vendor, []).append(jc.name)
        for vendor in by_vendor:
            try:
                self._cm[vendor].disable_all()
            except CallError as e:
                print(f"[{self.name}/disable] {e}")
            time.sleep(0.05)

    # ── 模式切换 ────────────────────────────────────────────────────────

    def _write_pv_params(self, jc: JointCfg) -> None:
        m = self._mm[jc.name]
        try:
            if jc.vendor == "robstride":
                m.robstride_write_param_f32(0x7017, jc.vlim)
                time.sleep(0.01)
                m.robstride_write_param_f32(0x701F, jc.vel_kp)
                time.sleep(0.01)
                m.robstride_write_param_f32(0x7020, jc.vel_ki)
                time.sleep(0.01)
                m.robstride_write_param_f32(0x701E, jc.pos_kp)
            elif jc.vendor == "damiao":
                m.write_register_f32(25, jc.vel_kp)
                m.write_register_f32(26, jc.vel_ki)
                m.write_register_f32(27, jc.pos_kp)
                m.write_register_f32(28, jc.pos_ki)
            time.sleep(0.02)
        except Exception as e:
            print(f"[{self.name}/pv_params/{jc.name}] {e}")

    def mode_mit(
        self,
        kp: Optional[np.ndarray] = None,
        kd: Optional[np.ndarray] = None,
    ) -> bool:
        self._mode = "mit"
        if kp is not None:
            self._mit_kp = np.asarray(kp, dtype=np.float64).reshape(-1)
        if kd is not None:
            self._mit_kd = np.asarray(kd, dtype=np.float64).reshape(-1)
        ok = True
        for jc in self._jcfgs:
            try:
                self._mm[jc.name].ensure_mode(Mode.MIT, 1000)
            except CallError as e:
                print(f"[{self.name}/mode_mit/{jc.name}] {e}")
                ok = False
            time.sleep(0.05)
        time.sleep(0.2)
        return ok

    def mode_pos_vel(
        self,
        vlim: Optional[np.ndarray] = None,
    ) -> bool:
        self._mode = "pos_vel"
        if vlim is not None:
            self._pv_vlim = np.asarray(vlim, dtype=np.float64).reshape(-1)
        ok = True
        for jc in self._jcfgs:
            self._write_pv_params(jc)
            try:
                self._mm[jc.name].ensure_mode(Mode.POS_VEL, 1000)
            except CallError as e:
                print(f"[{self.name}/mode_pos_vel/{jc.name}] {e}")
                ok = False
            time.sleep(0.05)
        time.sleep(0.2)
        return ok

    def mode_vel(self) -> bool:
        self._mode = "vel"
        ok = True
        for jc in self._jcfgs:
            try:
                self._mm[jc.name].ensure_mode(Mode.VEL, 1000)
            except CallError as e:
                print(f"[{self.name}/mode_vel/{jc.name}] {e}")
                ok = False
            time.sleep(0.05)
        time.sleep(0.2)
        return ok

    # ── MIT 发送 ────────────────────────────────────────────────────────

    def send_mit(
        self,
        pos: np.ndarray,
        vel: Optional[np.ndarray] = None,
        kp: Optional[np.ndarray] = None,
        kd: Optional[np.ndarray] = None,
        tau: Optional[np.ndarray] = None,
    ) -> None:
        n = self.num_joints
        pos = np.asarray(pos, dtype=np.float64).reshape(-1)
        if vel is None:
            vel = np.zeros(n)
        if tau is None:
            tau = np.zeros(n)
        if kp is None:
            kp = self._mit_kp
        if kd is None:
            kd = self._mit_kd

        for i, jc in enumerate(self._jcfgs):
            try:
                self._mm[jc.name].send_mit(
                    float(pos[i]),
                    float(vel[i]),
                    float(kp[i]),
                    float(kd[i]),
                    float(tau[i]),
                )
            except CallError:
                pass

    # ── POS_VEL 发送 ───────────────────────────────────────────────────

    def send_pos_vel(
        self,
        pos: np.ndarray,
        vlim: Optional[np.ndarray] = None,
    ) -> None:
        pos = np.asarray(pos, dtype=np.float64).reshape(-1)
        if vlim is None:
            vlim = self._pv_vlim
        vlim = np.asarray(vlim, dtype=np.float64).reshape(-1)
        for i in range(min(len(pos), len(vlim))):
            try:
                self._mm[self._jcfgs[i].name].send_pos_vel(
                    float(pos[i]),
                    float(vlim[i]),
                )
            except CallError:
                pass

    # ── VEL 发送 ───────────────────────────────────────────────────────

    def send_vel(self, vel: np.ndarray) -> None:
        vel = np.asarray(vel, dtype=np.float64).reshape(-1)
        for i in range(min(len(vel), self.num_joints)):
            try:
                self._mm[self._jcfgs[i].name].send_vel(float(vel[i]))
            except CallError:
                pass

    # ── 状态读取 ───────────────────────────────────────────────────────

    def _poll_feedback(self) -> None:
        """仅处理 CAN 接收队列中的反馈帧（快速，无总线发送）。"""
        seen: set[str] = set()
        for jc in self._jcfgs:
            if jc.vendor not in seen:
                seen.add(jc.vendor)
                try:
                    self._cm[jc.vendor].poll_feedback_once()
                except Exception:
                    pass

    def _request_feedback(self) -> None:
        """发送显式反馈请求帧 + 处理接收队列（慢，有总线发送）。"""
        for jc in self._jcfgs:
            try:
                self._mm[jc.name].request_feedback()
            except Exception:
                pass
        self._poll_feedback()

    def get_positions(self, request_feedback: bool = True) -> np.ndarray:
        # 始终发送显式请求帧 + 处理接收队列
        # motorbridge 内部会针对 RS/DM 分别处理
        if request_feedback: self._request_feedback()
        
        out: list[float] = []
        for jc in self._jcfgs:
            m = self._mm[jc.name]
            st = m.get_state()
            if st is not None:
                out.append(st.pos)
            else:
                # 缓存为空时回退到 SDO 读取（安全兜底）
                if jc.vendor == "robstride":
                    try:
                        out.append(float(m.robstride_get_param_f32(0x7019)))
                        continue
                    except CallError:
                        pass
                out.append(0.0)
        return np.array(out, dtype=np.float64)

    def get_velocities(self, request_feedback: bool = True) -> np.ndarray:
        # NOTE (RobStride): the cached state has the same staleness problem as
        # get_positions, and the mechVel param (0x701A) was measured NOT to be
        # rad/s on RS firmware (inconsistent scale/sign vs dq/dt, 2026-07-17).
        # For a live velocity on RobStride, finite-difference get_positions().
        if request_feedback:
            self._request_feedback()
        return np.array([
            self._mm[jc.name].get_state().vel
            if self._mm[jc.name].get_state() is not None else 0.0
            for jc in self._jcfgs
        ], dtype=np.float64)

    def __repr__(self) -> str:
        return f"JointGroup({self.name!r}, joints={self.num_joints}, mode={self._mode})"


# --------------------------------------------------------------------------
# RebotArm — 分组控制器容器
# --------------------------------------------------------------------------

class RebotArm:
    """reBotArm 分组控制系统。

    持有多个 JointGroup，每组独立控制模式，独立发送命令，
    在同一个控制循环中按组顺序同步发送，防止总线争用。

    按组访问（通过 __getattr__）::

        arm.arm       # 机械臂关节组
        arm.gripper   # 夹爪关节组（如果有）

    也可以通过 groups 字典::

        arm.groups["arm"]
        arm.groups["gripper"]

    手动添加组::

        arm.add_group("custom", ["joint1", "joint2"])
    """

    def __init__(self, mode, hw_yaml: str | None = None) -> None:
        self._hw_yaml = _resolve_hw_cfg_path(hw_yaml).name
        cfg = load_cfg(hw_yaml)

        self.mode = mode
        self._name: str = cfg["name"]
        self._channel: str = cfg["channel"]
        self._comm_period: float = 1.0 / cfg["comm_rate"]
        self._servol_rate: float = cfg["servol_rate"]
        self._all_joints: List[JointCfg] = cfg["joints"]
        self._groups_def: dict = cfg["groups"]

        self._ctrl_map: Dict[str, Controller] = {}
        self._motor_map: Dict[str, any] = {}
        self._groups: Dict[str, JointGroup] = {}

        self._connected: bool = False

        # 按硬件配置分组，并创建同维度的命令/反馈缓冲区。
        self._build_groups()
        self.arm_state = ArmState(
            mode,
            self._groups["arm"].num_joints,
            self._groups["gripper"].num_joints,
        )

        if mode == "mit":
            # MIT 命令默认使用各组配置的刚度和阻尼。
            for group_name in ("arm", "gripper"):
                group = self._groups[group_name]
                if group.num_joints == 0:
                    continue
                command = getattr(self.arm_state.command, group_name)
                command.kp[:] = group._mit_kp
                command.kd[:] = group._mit_kd

            arm_group = self._groups["arm"]
            # 重力力矩顺序必须与 arm 组的发送顺序完全一致。
            self._dynamics_model = load_dynamics_model(cfg["urdf_path"])
            model_joint_names = list(self._dynamics_model.names[1:])
            if model_joint_names != arm_group.joint_names:
                raise ValueError(
                    "dynamics model joints must match the arm group: "
                    f"{model_joint_names} != {arm_group.joint_names}"
                )
            self._dynamics_data = self._dynamics_model.createData()

        # 预先建立全局反馈顺序到 arm/gripper 缓冲区的索引映射。
        joint_indexes = {
            name: index
            for index, name in enumerate(self.joint_names)
        }
        self._arm_indexes = [
            joint_indexes[name]
            for name in self._groups["arm"].joint_names
        ]
        self._gripper_indexes = [
            joint_indexes[name]
            for name in self._groups["gripper"].joint_names
        ]

        # RebotArm 统一持有通讯线程生命周期和两组命令事件。
        self.comunicater = None
        self._stop_comunicater = threading.Event()
        self._joint_command_ready = threading.Event()
        self._gripper_command_ready = threading.Event()

    def connect(self) -> None:
        """连接总线并启动后台通讯线程。"""
        if self._connected:
            return
        try:
            self._setup_motors()
        except Exception:
            for ctrl in self._ctrl_map.values():
                ctrl.close()
            self._ctrl_map.clear()
            self._motor_map.clear()
            raise

        self._connected = True
        try:
            for group in self._groups.values():
                if group.num_joints == 0:
                    continue
                if self.mode == "mit":
                    success = group.mode_mit()
                else:
                    success = group.mode_pos_vel()
                if not success:
                    raise RuntimeError(f"failed to set {self.mode} mode")

            self.enable_all()
            self._stop_comunicater.clear()
            self.comunicater = threading.Thread(
                target=(
                    self._communicate_mit
                    if self.mode == "mit"
                    else self._communicate_posvel
                ),
                name="comunicater",
                daemon=True,
            )
            self.comunicater.start()
        except Exception:
            self.disconnect()
            raise

    def _make_controller(self, vendor: str) -> Controller:
        channel_path = Path(self._channel)
        resolved_channel = (
            str(channel_path.resolve())
            if channel_path.exists()
            else self._channel
        )
        if resolved_channel.startswith("/dev/tty"):
            return Controller.from_dm_serial(self._channel, 921600)
        return Controller(self._channel)

    def _setup_motors(self) -> None:
        for jc in self._all_joints:
            vendor = jc.vendor
            if vendor not in self._ctrl_map:
                self._ctrl_map[vendor] = self._make_controller(vendor)
            ctrl = self._ctrl_map[vendor]

            if vendor == "damiao":
                mot = ctrl.add_damiao_motor(jc.motor_id, jc.feedback_id, jc.model)
            elif vendor == "robstride":
                mot = ctrl.add_robstride_motor(jc.motor_id, jc.feedback_id, jc.model)
            elif vendor == "myactuator":
                mot = ctrl.add_myactuator_motor(jc.motor_id, jc.feedback_id, jc.model)
            elif vendor == "hightorque":
                mot = ctrl.add_hightorque_motor(jc.motor_id, jc.feedback_id, jc.model)
            else:
                raise ValueError(f"Unsupported vendor: {vendor}")

            self._motor_map[jc.name] = mot

    def _build_groups(self) -> None:
        for gname, gdef in self._groups_def.items():
            joints_def = gdef.get("joints", [])
            g = JointGroup(
                name=gname,
                joint_names=joints_def,
                all_joints=self._all_joints,
                motor_map=self._motor_map,
                ctrl_map=self._ctrl_map,
            )
            self._groups[gname] = g
        if "gripper" not in self._groups:
            self._groups["gripper"] = NoOpGroup()

    # ── 属性 ────────────────────────────────────────────────────────────

    @property
    def num_joints(self) -> int:
        return len(self._all_joints)

    @property
    def joint_names(self) -> List[str]:
        return [j.name for j in self._all_joints]

    @property
    def groups(self) -> Dict[str, JointGroup]:
        return self._groups

    @property
    def has_gripper(self) -> bool:
        return not isinstance(self._groups.get("gripper", None), NoOpGroup)

    @property
    def hardware_yaml(self) -> str:
        return self._hw_yaml

    @property
    def servol_rate(self) -> float:
        return self._servol_rate

    def __getattr__(self, name: str) -> any:
        if name.startswith("_"):
            raise AttributeError(name)
        if name in self._groups:
            return self._groups[name]
        raise AttributeError(name)

    # ── 手动添加组 ────────────────────────────────────────────────────

    def add_group(self, name: str, joint_names: List[str]) -> JointGroup:
        if name in self._groups:
            raise ValueError(f"组 {name!r} 已存在")
        g = JointGroup(
            name=name,
            joint_names=joint_names,
            all_joints=self._all_joints,
            motor_map=self._motor_map,
            ctrl_map=self._ctrl_map,
        )
        self._groups[name] = g
        return g

    # ── 全局使能 / 失能 ────────────────────────────────────────────────

    def enable_all(self) -> None:
        for g in self._groups.values():
            g.enable()

    def disable_all(self) -> None:
        for g in self._groups.values():
            g.disable()

    # ── 零点 ────────────────────────────────────────────────────────────

    def set_zero(self, poll_max: int = 200, poll_interval: float = 0.05) -> None:
        self.disable_all()
        time.sleep(0.3)
        for jc in self._all_joints:
            for _ in range(poll_max):
                for m in self._motor_map.values():
                    try:
                        m.request_feedback()
                    except Exception:
                        pass
                for ctrl in self._ctrl_map.values():
                    try:
                        ctrl.poll_feedback_once()
                    except Exception:
                        pass
                st = self._motor_map[jc.name].get_state()
                if st is not None and st.status_code == 0:
                    break
                time.sleep(poll_interval)
            try:
                self._motor_map[jc.name].set_zero_position()
            except CallError as e:
                print(f"[set_zero] {jc.name}: {e}")
            time.sleep(0.1)

    # ── 全局状态读取 ───────────────────────────────────────────────────

    def get_state(
        self,
        request_feedback: bool = True,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        if request_feedback:
            for m in self._motor_map.values():
                try:
                    m.request_feedback()
                except Exception:
                    pass
        for ctrl in self._ctrl_map.values():
            try:
                ctrl.poll_feedback_once()
            except Exception:
                pass
        pos, vel, torq = [], [], []
        for jc in self._all_joints:
            st = self._motor_map[jc.name].get_state()
            if st is not None:
                pos.append(st.pos)
                vel.append(st.vel)
                torq.append(st.torq)
            else:
                pos.append(0.0)
                vel.append(0.0)
                torq.append(0.0)
        return (
            np.array(pos, dtype=np.float64),
            np.array(vel, dtype=np.float64),
            np.array(torq, dtype=np.float64),
        )

    def get_state_with_time(
        self,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
        """返回机械臂状态和软件单调时间戳，不是硬件时间戳。"""
        pos, vel, torq = self.get_state()
        timestamp = time.monotonic()
        return pos, vel, torq, timestamp

    def get_positions(self) -> np.ndarray:
        return self.get_state()[0]

    def get_velocities(self) -> np.ndarray:
        return self.get_state()[1]

    def get_torques(self) -> np.ndarray:
        return self.get_state()[2]

    # ── 后台通讯 ────────────────────────────────────────────────────────

    def _communicate_posvel(self) -> None:
        next_cycle = time.monotonic()
        while not self._stop_comunicater.is_set():
            position, velocity, torque, timestamp = self.get_state_with_time()
            feedback = self.arm_state.feedback
            feedback.arm.position[:] = position[self._arm_indexes]
            feedback.arm.velocity[:] = velocity[self._arm_indexes]
            feedback.arm.torque[:] = torque[self._arm_indexes]
            feedback.gripper.position[:] = position[self._gripper_indexes]
            feedback.gripper.velocity[:] = velocity[self._gripper_indexes]
            feedback.gripper.torque[:] = torque[self._gripper_indexes]
            feedback.timestamp = timestamp

            if self._joint_command_ready.is_set():
                self._joint_command_ready.clear()
                command = self.arm_state.command.arm
                position = command.position.copy()
                velocity = command.velocity.copy()
                self._groups["arm"].send_pos_vel(position, velocity)

            if self._gripper_command_ready.is_set():
                self._gripper_command_ready.clear()
                command = self.arm_state.command.gripper
                position = command.position.copy()
                velocity = command.velocity.copy()
                self._groups["gripper"].send_pos_vel(position, velocity)

            next_cycle += self._comm_period
            timeout = next_cycle - time.monotonic()
            if timeout > 0.0:
                self._stop_comunicater.wait(timeout)
            else:
                next_cycle = time.monotonic()

    def _communicate_mit(self) -> None:
        arm_group = self._groups["arm"]
        arm_command_initialized = False
        command_position = None
        command_velocity = None
        command_kp = None
        command_kd = None
        command_torque = None
        command_time = 0.0
        velocity_timeout = 0.0
        next_cycle = time.monotonic()
        while not self._stop_comunicater.is_set():
            position, velocity, torque, timestamp = self.get_state_with_time()
            feedback = self.arm_state.feedback
            feedback.arm.position[:] = position[self._arm_indexes]
            feedback.arm.velocity[:] = velocity[self._arm_indexes]
            feedback.arm.torque[:] = torque[self._arm_indexes]
            feedback.gripper.position[:] = position[self._gripper_indexes]
            feedback.gripper.velocity[:] = velocity[self._gripper_indexes]
            feedback.gripper.torque[:] = torque[self._gripper_indexes]
            feedback.timestamp = timestamp

            if not arm_command_initialized:
                if any(
                    self._motor_map[name].get_state() is None
                    for name in arm_group.joint_names
                ):
                    next_cycle += self._comm_period
                    timeout = next_cycle - time.monotonic()
                    if timeout > 0.0:
                        self._stop_comunicater.wait(timeout)
                    else:
                        next_cycle = time.monotonic()
                    continue
                command_state = self.arm_state.command
                command = command_state.arm
                command.position[:] = feedback.arm.position
                command.velocity[:] = 0.0
                command.torque[:] = 0.0
                command_state.timestamp = 0.0
                command_position = command.position.copy()
                command_velocity = command.velocity.copy()
                command_kp = command.kp.copy()
                command_kd = command.kd.copy()
                command_torque = command.torque.copy()
                arm_command_initialized = True

            if self._joint_command_ready.is_set():
                self._joint_command_ready.clear()
                command_state = self.arm_state.command
                command = command_state.arm
                command_position = command.position.copy()
                command_velocity = command.velocity.copy()
                command_kp = command.kp.copy()
                command_kd = command.kd.copy()
                command_torque = command.torque.copy()
                command_time = command_state.timestamp
                velocity_timeout = command_state.velocity_timeout

            if (
                command_time != 0.0
                and time.monotonic() - command_time >= velocity_timeout
            ):
                expired_timestamp = command_time
                command_time = 0.0
                command_velocity[:] = 0.0
                command_state = self.arm_state.command
                if command_state.timestamp == expired_timestamp:
                    command_state.timestamp = 0.0
                    command_state.arm.velocity[:] = 0.0

            gravity = compute_generalized_gravity(
                self._dynamics_model,
                feedback.arm.position + arm_group._gravity_offset,
                self._dynamics_data,
            )
            compensation = (
                arm_group._gravity_k * gravity
                + arm_group._friction * np.sign(feedback.arm.velocity)
                + arm_group._tau_bias
            )
            arm_group.send_mit(
                command_position,
                command_velocity,
                command_kp,
                command_kd,
                command_torque + compensation,
            )

            if self._gripper_command_ready.is_set():
                self._gripper_command_ready.clear()
                command = self.arm_state.command.gripper
                position = command.position.copy()
                velocity = command.velocity.copy()
                kp = command.kp.copy()
                kd = command.kd.copy()
                torque = command.torque.copy()
                self._groups["gripper"].send_mit(
                    position,
                    velocity,
                    kp,
                    kd,
                    torque,
                )

            next_cycle += self._comm_period
            timeout = next_cycle - time.monotonic()
            if timeout > 0.0:
                self._stop_comunicater.wait(timeout)
            else:
                next_cycle = time.monotonic()

    # ── 生命周期 ────────────────────────────────────────────────────────

    def disconnect(self) -> None:
        if not self._connected:
            return
        communicater = self.comunicater
        self._stop_comunicater.set()
        if communicater is not None and communicater.is_alive():
            communicater.join()
        self.comunicater = None
        self._joint_command_ready.clear()
        self._gripper_command_ready.clear()
        self.disable_all()
        time.sleep(0.5)
        for ctrl in self._ctrl_map.values():
            ctrl.shutdown()
            time.sleep(0.1)
            ctrl.close()
        self._ctrl_map.clear()
        self._motor_map.clear()
        self._connected = False

    def estop(self) -> None:
        self.disable_all()

    # ── 上下文管理器 ───────────────────────────────────────────────────────

    def __enter__(self) -> "RebotArm":
        return self

    def __exit__(self, *args) -> None:
        self.disconnect()

    def __repr__(self) -> str:
        gs = ", ".join(f"{k}({g.num_joints}j)" for k, g in self._groups.items())
        return f"RebotArm({self._name!r}, [{gs}])"
