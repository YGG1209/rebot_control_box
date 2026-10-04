# reBot Control Daemon

## 1. 模块定位

`control_daemon.py` 是控制箱的本地实时控制层，位于网络服务与机械臂驱动之间。它负责管理机械臂连接、执行控制命令、发布状态快照，并在命令中断或控制异常时执行安全处理。

```text
上层主机
    ↓ TCP / 60 Hz
network_server.py
    ↓ 本机接口
control_daemon.py
    ↓ servoJ / 60 Hz
RebotArmController
    ↓ 底层通信 / 350 Hz
达妙电机
```

该模块不负责TCP连接、协议字节编解码、客户端控制权租约和用户认证。这些功能由网络层负责。

## 2. 当前功能

- 管理机械臂的连接和断开；
- 运行默认60 Hz控制循环；
- 接收并校验六轴`servoJ`目标；
- 限制远程目标速度、单周期目标偏差和关节位置范围；
- 只保留最新有效命令；
- 根据uint32序号拒绝重复、旧序和乱序命令；
- 支持序号从`4294967295`回绕到`0`；
- 100 ms未收到新命令时进入位置保持；
- 反馈时间戳停止推进超过200 ms时进入故障保护；
- 发布线程安全、不可变的机器人状态快照；
- 捕获反馈读取、控制下发和连接过程中的异常。

当前版本只实现关节空间`servoJ`控制，不包含`servoL`、轨迹队列、逆运动学和网络服务。

## 3. 默认参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `control_rate_hz` | 60.0 Hz | 控制守护线程运行频率 |
| `watchdog_timeout_s` | 0.1 s | 流式命令超时时间 |
| `feedback_timeout_s` | 10.0 s | 启动时等待第一帧反馈的最长时间 |
| `feedback_stale_timeout_s` | 0.2 s | 运行期反馈时间戳停滞阈值 |
| `hold_speed_rad_s` | 0.1 rad/s | 位置保持时各关节速度限制 |
| `max_speed_rad_s` | 1.57 rad/s | 远程命令允许的最大关节速度 |
| `max_target_error_rad` | 1.0 rad | 目标与最新实际位置的最大偏差 |
| `joint_limits` | 自动读取 | 六轴显式软限位，可覆盖模型限位 |
| 控制器模式 | `posvel` | `RebotArmController`工作模式 |

看门狗在控制循环中检查，因此100 ms是超时判定阈值，实际处理发生在随后的一个控制周期内。默认60 Hz时最多会增加约16.7 ms调度时间。

## 4. 状态机

`DaemonState`直接使用`protocol.py`中的`RobotMode`，保证本地状态与网络协议数值一致。

| 数值 | 状态 | 含义 |
|---:|---|---|
| 0 | `DISCONNECTED` | 未连接机械臂 |
| 1 | `CONNECTING` | 正在连接并等待反馈 |
| 2 | `IDLE` | 已连接，正在保持位置 |
| 3 | `RUNNING` | 正在应用流式控制命令 |
| 4 | `FAULT` | 控制过程发生故障 |
| 5 | `STOPPING` | 正在停止线程并断开机械臂 |

主要状态转换如下：

```text
DISCONNECTED → CONNECTING → IDLE → RUNNING
                    ↓         ↑       ↓
                  FAULT       └─ IDLE ← 命令超时或pause

IDLE/RUNNING/FAULT → STOPPING → DISCONNECTED
```

非法状态转换会抛出`RuntimeError`，防止网络层在错误的生命周期阶段下发控制命令。

## 5. 安全状态

| 数值 | 状态 | 含义 |
|---:|---|---|
| 0 | `NORMAL` | 控制正常 |
| 1 | `COMMAND_TIMEOUT` | 有效流式命令超过100 ms未刷新 |
| 2 | `CONTROL_FAULT` | 控制线程、反馈读取或硬件调用异常 |

机器人状态和安全状态是两组独立信息。例如命令超时后，机器人状态为`IDLE`，安全状态为`COMMAND_TIMEOUT`。

## 6. 启动与停止

调用`start()`时依次执行：

1. 从`DISCONNECTED`进入`CONNECTING`；
2. 创建并连接`RebotArmController("posvel")`；
3. 等待第一帧有效反馈；
4. 将实际关节位置设为保持目标；
5. 发送一次位置保持命令；
6. 进入`IDLE`并启动控制线程。

调用`shutdown()`时停止控制线程、断开机械臂、清空命令缓存，最后回到`DISCONNECTED`。断开控制器可能导致电机失能，因此真机操作前必须扶稳机械臂并准备急停。

## 7. 命令缓存

`LatestCommandBuffer`使用“只保留最新值”的策略，不建立轨迹队列。每条`ServoJCommand`包含：

| 字段 | 含义 |
|---|---|
| `sequence` | 客户端uint32输入序号 |
| `target_q` | 六轴目标关节位置，单位rad |
| `speed_limits` | 六轴速度限制，单位rad/s |
| `received_at_ns` | 控制箱收到命令时的单调时钟 |

新命令必须比上一条已接受命令更新。重复、旧序和乱序命令返回`False`，不会刷新看门狗。命令过期或执行`pause()`后，活动命令会被清除，但最后序号仍会保留；只有`reset_session()`或重新启动守护进程才会重置序号会话。

## 8. 100 ms看门狗

看门狗使用控制箱本地的`time.monotonic_ns()`计算命令年龄，不依赖上层主机时间戳。

正常情况下：

1. 网络层持续提交新的有效命令；
2. 控制循环应用最新目标；
3. 状态保持为`RUNNING + NORMAL`。

超过100 ms没有新的有效命令时：

1. 当前活动命令被丢弃；
2. 读取超时瞬间的实际关节位置；
3. 将该位置设置为新的保持目标；
4. 使用`hold_speed_rad_s`持续发送保持命令；
5. 状态变为`IDLE + COMMAND_TIMEOUT`。

收到序号更新的有效命令后，守护进程自动恢复为`RUNNING + NORMAL`。网络超时不会直接失能电机，以避免机械臂因重力突然下落。

## 9. 对外接口

### `start()`

连接机械臂、建立初始位置保持并启动控制线程。只能在`DISCONNECTED`状态调用。

### `submit_servoj(sequence, target_q, speed_limits)`

提交最新的关节位置命令。只能在`IDLE`或`RUNNING`状态调用。

- `sequence`应为uint32递增序号；
- `target_q`必须包含6个有限数，单位rad；
- `speed_limits`可以是一个正数，也可以是6个正数，单位rad/s；
- 速度不得超过`max_speed_rad_s`；
- 每轴目标与最新`actual_q`的偏差不得超过`max_target_error_rad`；
- 目标不得越过显式限位或控制器模型前六轴的有限位置限位；
- 返回`True`表示命令已接受；
- 返回`False`表示序号重复、过旧或乱序。

### `pause()`

清除活动命令，读取当前实际关节位置并进入位置保持。状态变为`IDLE + NORMAL`。

### `reset_session()`

先执行`pause()`，再清除最后输入序号。网络层释放控制权或建立新控制会话时应调用此接口。

### `get_snapshot()`

返回线程安全、不可变的`RobotSnapshot`。网络服务可以直接读取该对象生成`OUTPUT_DATA`。

### `shutdown()`

停止控制线程并断开机械臂。重复调用不会产生额外操作。

## 10. 状态快照

`RobotSnapshot`包含以下字段：

| 字段 | 含义 |
|---|---|
| `timestamp_ns` | 快照更新时的控制箱单调时钟 |
| `daemon_state` | 当前机器人状态 |
| `safety_state` | 当前安全状态 |
| `actual_q` | 六轴实际关节位置，rad |
| `actual_qd` | 六轴实际关节速度，rad/s |
| `actual_tau` | 六轴实际关节力矩，Nm |
| `target_q` | 当前应用或保持的目标位置，rad |
| `last_input_sequence` | 最近应用的输入序号 |
| `fault_code` | 故障编号，当前非零值为1 |
| `fault_message` | 底层异常信息 |

快照采用不可变数据类，网络线程读取时不会获得一半新、一半旧的数据。

## 11. 基本用法

```python
import time

from reBotArm_control_box.control_daemon import ControlDaemon


daemon = ControlDaemon()

try:
    daemon.start()

    sequence = 1
    target_q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    while True:
        accepted = daemon.submit_servoj(sequence, target_q, 0.2)
        if not accepted:
            raise RuntimeError("servoJ sequence was rejected")

        sequence = (sequence + 1) & 0xFFFFFFFF
        time.sleep(1.0 / 60.0)
finally:
    daemon.shutdown()
```

该示例会驱动真实机械臂，仅用于说明接口。真机测试时目标位置应从当前反馈平滑生成，不能直接使用示例中的全零位置。

## 12. 测试

单元测试使用模拟控制器，不会连接真实机械臂：

```bash
python -m unittest reBotArm_control_box.test_control_daemon -v
```

联合运行守护进程和协议测试：

```bash
python -m unittest \
  reBotArm_control_box.test_control_daemon \
  reBotArm_control_box.test_protocol \
  -v
```

测试覆盖启动保持、正常命令、旧序拒绝、uint32回绕、命令超时、超时恢复、暂停、非法输入和连接故障。

## 13. 与网络层的连接原则

后续`network_server.py`只需要完成以下转换：

- 将有效`INPUT_DATA`转换为`submit_servoj()`调用；
- 将`PAUSE`转换为`pause()`调用；
- 在控制会话结束时调用`reset_session()`；
- 周期读取`get_snapshot()`并编码为`OUTPUT_DATA`；
- 将`FAULT`和`COMMAND_TIMEOUT`原样报告给上层主机。

Socket接收线程不能直接调用`RebotArmController`，也不能绕过命令缓存和看门狗操作机械臂。

## 14. 安全边界

- 控制循环持续检查反馈时间戳；反馈倒退、无效或停滞时进入`FAULT`；
- 控制循环异常时先尽力发送最后一帧实际位置保持，再停止循环；
- 模拟控制器没有`_model`时不自动启用位置限位，可通过`joint_limits`显式配置；
- 软件看门狗不能替代物理急停；
- 网络断开不能作为电机急停手段；
- `shutdown()`可能使机械臂失能并下落；
- 当前`fault_code`只是基础占位值，尚未细分电机、通信和控制算法故障；
- 正式运行前仍需完成真机超时保持、故障恢复和长时间稳定性测试。
