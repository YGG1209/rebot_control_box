# reBot Control Box Protocol v1

## 1. 协议定位

该协议用于上层主机与 reBot MiniPC 控制箱之间的数据交换。网络接口只传输控制目标、状态和管理命令，不直接暴露达妙电机串口。

```text
上层主机客户端
    ↓ 持久TCP连接
network_server.py
    ↓ 本机IPC
control_daemon.py
    ↓
RebotArmController
    ↓ 350 Hz
达妙电机
```

`protocol.py` 只负责消息定义和字节编解码，不负责 Socket、IPC、机械臂控制或安全状态转换。

## 2. 传输规则

- v1 使用持久 TCP 连接。
- 实时输入和状态输出的目标频率为 60 Hz。
- 建议客户端和服务端启用 `TCP_NODELAY`。
- 端口属于部署配置，不写入线协议。
- 所有二进制整数和浮点数使用网络字节序。
- 单帧负载最大为 65536 字节。
- TCP 是字节流，接收端必须处理拆包与粘包。

## 3. 连接流程

```text
TCP连接
  → HELLO / HELLO_ACK
  → SETUP_OUTPUTS / SETUP_OUTPUTS_ACK
  → 可选 SETUP_INPUTS / SETUP_INPUTS_ACK
  → 可选 ACQUIRE_CONTROL / ACQUIRE_CONTROL_ACK
  → START / START_ACK
  → INPUT_DATA 与 OUTPUT_DATA
  → PAUSE / PAUSE_ACK
  → RELEASE_CONTROL / RELEASE_CONTROL_ACK
  → 关闭TCP连接
```

只读取状态的客户端不需要设置输入字段，也不需要申请控制权。

## 4. 固定帧头

每条消息由24字节帧头和可选负载组成。Python格式为：

```python
struct.Struct("!4sBBHIIQ")
```

| 偏移 | 大小 | 类型 | 字段 | 说明 |
|---:|---:|---|---|---|
| 0 | 4 | bytes | magic | 固定为 `RBOT` |
| 4 | 1 | uint8 | version | 当前为1 |
| 5 | 1 | uint8 | message_type | 消息类型 |
| 6 | 2 | uint16 | flags | v1固定发送0 |
| 8 | 4 | uint32 | payload_length | 负载字节数 |
| 12 | 4 | uint32 | sequence | 发送方向独立递增序号 |
| 16 | 8 | uint64 | timestamp_ns | 发送方单调时钟 |

完整帧长度为：

```text
24 + payload_length
```

## 5. 消息类型

| 数值 | 名称 | 负载 |
|---:|---|---|
| 0x01 | HELLO | JSON |
| 0x02 | HELLO_ACK | JSON |
| 0x10 | SETUP_INPUTS | JSON |
| 0x11 | SETUP_INPUTS_ACK | JSON |
| 0x12 | SETUP_OUTPUTS | JSON |
| 0x13 | SETUP_OUTPUTS_ACK | JSON |
| 0x20 | ACQUIRE_CONTROL | JSON |
| 0x21 | ACQUIRE_CONTROL_ACK | JSON |
| 0x22 | RELEASE_CONTROL | JSON |
| 0x23 | RELEASE_CONTROL_ACK | JSON |
| 0x30 | START | JSON或空负载 |
| 0x31 | START_ACK | JSON |
| 0x32 | PAUSE | JSON或空负载 |
| 0x33 | PAUSE_ACK | JSON |
| 0x40 | INPUT_DATA | 二进制Recipe |
| 0x41 | OUTPUT_DATA | 二进制Recipe |
| 0x50 | HEARTBEAT | 空负载 |
| 0x51 | HEARTBEAT_ACK | JSON |
| 0x60 | TEXT_MESSAGE | JSON |
| 0x7F | ERROR | JSON |

`HEARTBEAT_ACK`示例：

```json
{
  "related_sequence": 42
}
```

## 6. 版本协商

`HELLO`的帧头使用当前引导版本1，客户端再通过
`supported_versions`声明自己支持的所有协议版本。

客户端连接后首先发送：

```json
{
  "client_name": "rebot-python-client",
  "supported_versions": [1]
}
```

服务端接受后返回：

```json
{
  "accepted": true,
  "selected_version": 1,
  "controller_name": "rebot-control-box",
  "related_sequence": 0
}
```

如果双方没有共同版本，服务端返回 `ERROR` 后关闭连接。

## 7. JSON管理消息

管理消息使用UTF-8 JSON对象，编码时不保留无用空格。顶层值必须是对象，禁止：

- 非UTF-8数据；
- 数组或单独数值作为顶层值；
- `NaN`；
- `Infinity`；
- `-Infinity`。

## 8. Recipe机制

输入和输出Recipe分别使用固定编号：

| 方向 | 配置消息 | Recipe ID | 频率 |
|---|---|---:|---:|
| 主机→控制箱 | SETUP_INPUTS | 1 | v1固定60 Hz |
| 控制箱→主机 | SETUP_OUTPUTS | 2 | 1–60 Hz |

`SETUP_INPUTS`必须完整包含`control_mode`、`target_q`和
`speed_limits`，但字段顺序可由客户端选择：

```json
{
  "frequency_hz": 60.0,
  "fields": [
    "control_mode",
    "target_q",
    "speed_limits"
  ]
}
```

`SETUP_OUTPUTS`可以选择任意当前可用的非空字段集合：

```json
{
  "frequency_hz": 60.0,
  "fields": [
    "controller_time_ns",
    "actual_q",
    "actual_qd",
    "robot_mode",
    "safety_state"
  ]
}
```

服务端验证字段后返回：

```json
{
  "accepted": true,
  "recipe_id": 2,
  "frequency_hz": 60.0,
  "fields": [
    "controller_time_ns",
    "actual_q",
    "actual_qd",
    "robot_mode",
    "safety_state"
  ],
  "types": [
    "UINT64",
    "VECTOR6D",
    "VECTOR6D",
    "UINT8",
    "UINT8"
  ],
  "related_sequence": 1
}
```

后续DATA负载不再发送字段名，格式为：

```text
recipe_id:uint8
field_1
field_2
...
```

字段顺序必须与SETUP请求完全一致。未知字段、重复字段、非字符串字段和空Recipe均无效。Recipe ID仅在当前TCP会话内有效，重连后必须重新配置。

在尚未`START`时，重复SETUP会原子替换同方向的旧Recipe；无效配置返回`ERROR`但不关闭TCP连接，也不替换已有Recipe。

## 9. 数据类型

| 类型 | 大小 | 编码 |
|---|---:|---|
| BOOL | 1字节 | 布尔值 |
| UINT8 | 1字节 | 无符号整数 |
| UINT32 | 4字节 | 无符号整数 |
| UINT64 | 8字节 | 无符号整数 |
| INT32 | 4字节 | 有符号整数 |
| DOUBLE | 8字节 | IEEE-754双精度 |
| VECTOR6D | 48字节 | 连续6个DOUBLE |
| VECTOR7D | 56字节 | 连续7个DOUBLE |

所有DOUBLE和向量元素必须是有限数。

## 10. 输入字段

| 字段 | 类型 | 单位 | 当前可用 | 说明 |
|---|---|---|---|---|
| control_mode | UINT8 | - | 是 | 0空闲、1 servoJ；2 servoL预留 |
| target_q | VECTOR6D | rad | 是 | 目标关节位置 |
| target_qd | VECTOR6D | rad/s | 否 | 目标关节速度 |
| target_pose | VECTOR7D | m和四元数 | 否 | TCP目标位姿 |
| speed_limits | VECTOR6D | rad/s | 是 | 各关节速度限制 |
| speed_scale | DOUBLE | 0..1 | 否 | 全局速度比例 |

## 11. 输出字段

| 字段 | 类型 | 单位 | 当前可用 | 说明 |
|---|---|---|---|---|
| controller_time_ns | UINT64 | ns | 是 | 控制箱单调时钟 |
| actual_q | VECTOR6D | rad | 是 | 实际关节位置 |
| actual_qd | VECTOR6D | rad/s | 是 | 实际关节速度 |
| actual_tau | VECTOR6D | Nm | 是 | 实际关节力矩 |
| actual_tcp_pose | VECTOR7D | m和四元数 | 否 | 实际TCP位姿 |
| target_q | VECTOR6D | rad | 是 | 当前目标关节位置 |
| robot_mode | UINT8 | - | 是 | 机器人模式 |
| safety_state | UINT8 | - | 是 | 安全状态 |
| fault_code | UINT32 | - | 是 | 故障位掩码 |
| last_input_sequence | UINT32 | - | 是 | 最近应用的输入序号 |

### robot_mode

| 数值 | 名称 | 说明 |
|---:|---|---|
| 0 | DISCONNECTED | 未连接机械臂 |
| 1 | CONNECTING | 正在连接机械臂 |
| 2 | IDLE | 已连接并保持位置 |
| 3 | RUNNING | 正在应用流式控制目标 |
| 4 | FAULT | 控制进程发生故障 |
| 5 | STOPPING | 正在停止并断开 |

### safety_state

| 数值 | 名称 | 说明 |
|---:|---|---|
| 0 | NORMAL | 正常 |
| 1 | COMMAND_TIMEOUT | 流式命令超过100 ms未刷新 |
| 2 | CONTROL_FAULT | 控制进程故障 |

## 12. 控制权

同一时间只允许一个客户端写入运动目标。其他客户端可以只读订阅状态。
当前网络服务第一阶段仅允许一个活跃TCP客户端，额外连接会收到
`SERVER_BUSY`；多个只读客户端将在控制权租约完成后开放。

`ACQUIRE_CONTROL` 示例：

```json
{
  "mode": "exclusive",
  "requested_lease_ms": 1000
}
```

服务端返回是否接受、会话编号和实际租约时间。连接断开或租约超时后，控制权由控制守护进程释放。

## 13. 序号规则

- 客户端和服务端分别维护自己的uint32序号。
- ACK和ERROR通过JSON负载中的`related_sequence`关联对应请求。
- 服务端主动发送的异步数据帧使用服务端自身的递增序号。
- 相同序号视为重复帧。
- 旧序号和乱序输入必须丢弃。
- uint32从 `4294967295` 回绕到 `0` 时，`0` 被视为更新。
- 多条控制目标等待处理时，只保留最新有效目标。

## 14. 时间戳规则

`timestamp_ns` 使用发送方 `time.monotonic_ns()`。不同机器的单调时钟没有共同起点，不能直接相减计算单向延迟。未进行时钟同步时，应使用请求响应序号测量往返延迟。

## 15. 看门狗

- 流式控制期望输入频率为60 Hz。
- 100 ms没有收到有效输入时，控制守护进程进入受控保持或停止。
- TCP协商完成后1秒没有收到完整有效帧时，关闭连接并释放远程控制权。
- 暂时没有实时数据时，客户端应以小于1秒的周期发送`HEARTBEAT`。
- 网络超时不能直接失能关节，避免机械臂掉落。
- 网络停止命令不能替代物理急停。

看门狗和安全动作由 `control_daemon.py` 实现，Socket线程只负责报告连接状态。

## 16. 错误消息

```json
{
  "code": "INVALID_RECIPE",
  "message": "unknown recipe field: foo",
  "related_sequence": 42
}
```

帧头、版本和权限错误可以在发送 `ERROR` 后关闭连接。运动目标数据错误时，应拒绝本帧，并由控制守护进程决定是否进入故障状态。

## 17. 兼容性

- v1字段含义和单位发布后不得修改。
- 新字段可以追加，但客户端必须通过SETUP选择后才会接收。
- 不兼容的帧头或数据语义修改必须提升协议版本。
- 后续UDP实时通道可以复用相同帧头、序号、Recipe和看门狗语义。
