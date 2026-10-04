# 上层主机客户端与真机验证

`network_client.py`是运行在上层主机上的Python客户端。它通过持久TCP连接
MiniPC控制箱，完成版本协商、Recipe配置、控制权申请、60 Hz servoJ发送和
状态接收。客户端只依赖Python标准库，不直接访问机械臂串口。

## 1. 运行结构

```text
上层主机
  host_servoj_demo.py / RebotNetworkClient
             ↓ 持久TCP，默认192.168.100.2:5000
MiniPC
  network_server.py
             ↓ Unix Domain Socket
  ipc_server.py + ControlDaemon
             ↓ 350 Hz底层通信
  reBot机械臂
```

上层主机只发送关节目标。串口、达妙电机通信、命令看门狗和安全保持均留在
MiniPC内执行。

## 2. MiniPC启动顺序

确认机械臂工作空间已清空、急停可用，并在仓库根目录打开两个终端。

终端一启动控制守护进程和IPC：

```bash
cd /home/rebot/rebot_control_box
source .venv/bin/activate
python -m reBotArm_control_box.ipc_server
```

看到机械臂连接完成和IPC启动提示后，在终端二启动网络服务：

```bash
cd /home/rebot/rebot_control_box
source .venv/bin/activate
python -m reBotArm_control_box.network_server \
  --host 192.168.100.2 \
  --port 5000
```

`--host`必须是MiniPC连接上层主机的有线网卡地址。不要把它填写成上层主机
地址。系统防火墙需要允许上层主机访问TCP 5000端口。

## 3. 上层主机首次真机测试

上层主机需要具有当前仓库中的`reBotArm_control_box`目录。进入仓库根目录后
执行：

```bash
python -m reBotArm_control_box.host_servoj_demo \
  --host 192.168.100.2 \
  --port 5000 \
  --joint 1 \
  --amplitude-deg 2 \
  --speed 0.1 \
  --duration 10
```

程序会先显示安全确认，不会在启动后立即运动。按Enter后执行以下流程：

1. 建立TCP连接并协商协议v1；
2. 配置60 Hz输入和输出Recipe；
3. 申请独占控制权并启动数据流；
4. 从`actual_q`读取六个关节的当前位置；
5. 以60 Hz保持当前位置2秒；
6. 让指定关节围绕初始位置做小幅正弦往返；
7. 无论正常结束、异常或Ctrl+C，均依次尝试`PAUSE`、释放控制权和断开连接。

参数含义：

- `--joint`：运动关节编号，范围1到6；
- `--amplitude-deg`：相对初始位置的正弦幅值，单位度；
- `--speed`：轨迹最大目标速度及关节速度限制，单位rad/s；
- `--duration`：运动段时长，单位秒。

首次测试建议保留默认的关节1、2度、0.1 rad/s，并由现场人员全程监护。

## 4. 在自己的上层程序中调用

```python
import time

from reBotArm_control_box.network_client import RebotNetworkClient


client = RebotNetworkClient("192.168.100.2", 5000)

try:
    client.connect()
    client.setup_outputs()
    client.setup_inputs()
    client.acquire_control(requested_lease_ms=1000)
    client.start()

    state = client.wait_for_state(timeout_s=3.0)
    target_q = tuple(state["actual_q"])

    next_cycle = time.monotonic()
    last_sequence = 0
    for _ in range(600):
        last_sequence = client.send_servoj(target_q, 0.1)
        next_cycle += 1.0 / 60.0
        time.sleep(max(0.0, next_cycle - time.monotonic()))

    client.wait_until_applied(last_sequence, timeout_s=0.5)
finally:
    if client.started:
        client.pause()
    if client.has_control:
        client.release_control()
    client.close()
```

也可以使用上下文自动关闭TCP连接：

```python
from reBotArm_control_box.network_client import RebotNetworkClient


with RebotNetworkClient("192.168.100.2", 5000) as client:
    client.setup_outputs()
    client.start()
    state = client.wait_for_state()
```

只读状态订阅不需要`setup_inputs()`和`acquire_control()`。配置输出后直接
`start()`即可，随后使用`wait_for_state()`读取状态。

## 5. 客户端API

- `connect()`：建立连接并完成`HELLO`版本协商；
- `setup_outputs(fields, frequency_hz)`：选择状态字段和1到60 Hz输出频率；
- `setup_inputs()`：配置v1固定的60 Hz servoJ输入Recipe；
- `acquire_control()`：申请当前TCP连接的独占控制权；
- `start()`：启动实时输入和输出；
- `send_servoj(target_q, speed_limits)`：发送一帧六轴弧度目标并返回输入序号；
- `wait_for_state()`：等待异步状态，返回不可变的`RobotState`；
- `wait_until_applied(sequence)`：等待`last_input_sequence`确认目标已被应用；
- `pause()`：停止流式输入并让MiniPC进入安全保持；
- `release_control()`：在暂停后释放控制权；
- `close()`：停止后台线程并关闭TCP连接。

`speed_limits`可以是一个正数，也可以是六个正数组成的序列。`target_q`和
`actual_q`均使用弧度。

## 6. 线程与安全行为

- 只有一个后台接收线程读取Socket，并用`FrameDecoder`处理拆包和粘包；
- 所有发送共用一个锁，管理消息、心跳和实时目标使用同一个严格递增序号；
- ACK和ERROR按`related_sequence`关联到原请求；
- `OUTPUT_DATA`异步解包后只保存最新状态，并唤醒状态等待者；
- 空闲约0.25秒后发送应用层心跳，servoJ目标绝不会被客户端自动重发；
- 任何请求超时、协议错误或TCP断开都会唤醒等待线程并停止继续发送；
- 控制流超过100 ms未刷新时，MiniPC看门狗会停止跟踪新目标并保持位置。

TCP断开不是急停，也不会替代实体急停。上层程序不应依赖桌面系统调度实现
功能安全，运动范围、速度、碰撞检查和现场急停仍需单独保证。

## 7. 常见问题

连接被拒绝时，先检查MiniPC上的两个进程是否都在运行，再从上层主机执行：

```bash
ping 192.168.100.2
```

收到`SERVER_BUSY`表示已有另一个上层TCP客户端连接。先关闭旧客户端，再重新
连接。收到`IPC_UNAVAILABLE`表示网络服务无法访问本机控制守护进程，应先恢复
`ipc_server.py`。收到`COMMAND_TIMEOUT`表示60 Hz发送循环曾超过100 ms没有提供
新目标，应立即排查主机负载、网络阻塞和程序中的耗时操作。
