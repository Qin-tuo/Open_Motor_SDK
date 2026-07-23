# khcan 使用说明
# ***!!!注意灵足/富兴、因克斯、达妙/达秒、海泰不同型号的 p/v/t/kp/kd 映射范围不同，必须使用正确型号或显式填写完整映射参数，否则会导致电机比例映射不正常!!!***
**驱动内置常见型号的协议参数；TOML 中填写准确的 `type` 即可自动匹配，未知型号才需要显式填写 `p_min/p_max/v_min/v_max/kp_min/kp_max/kd_min/kd_max/t_min/t_max`。ENCOS 参数以仓库内 `doc/ENCOS协议.pdf` 的型号表为准。**
# **Type5（高擎）默认使用 CAN FD。**

## 简介
`khcan` 是一个基于 ROS 2 `ament_cmake` 的 SocketCAN 电机驱动包。
当前包会安装可复用驱动库 `libkhcan_driver.so`，其他 ROS 2 工程可直接链接该库进行控制；也提供单进程整体底层驱动节点 `motor_driver_node`，一次读取完整 `motor.toml`，再按 CAN 通道暴露 `/canX/...` 接口给上层控制。
当前仓库中可用的入口程序有：
- `motor_driver_node`：整体底层驱动节点，读取完整 TOML 后统一管理所有电机，并按通道提供 ROS 2 topic/service
- `show_status`：可选择 M4/K1/自定义 TOML，默认只周期性查询所有电机状态；清错、使能、退出失能必须显式加参数
- `test_mit_mode`：根据 `api_type` 自动切换到 MIT 模式（Type1/2/3/5/8），对单个电机执行正弦摆动测试

## 整体底层驱动节点
推荐用 `motor_driver_node` 作为整机底层驱动入口：
```bash
ros2 launch khcan motor_driver.launch.py
```

仓库同时提供按本体区分的 bringup，分别固定加载各自的 TOML：
```bash
ros2 launch khcan m4_bringup.launch.py
ros2 launch khcan k1_bringup.launch.py
```

- M4：`config/m4.toml`
- K1：`config/k1.toml`
- 通用/开发默认：`config/motor.toml`

节点只创建一个 `BaseRobot` 实例，并一次加载完整 `motor.toml`。随后会根据 TOML 中出现的 `chan` 自动创建通道接口，例如 `chan = 1` 的电机会暴露在 `/can1/...` 下。

每个通道提供同一套接口：

| 接口 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/canX/command` | `sensor_msgs/msg/JointState` | 上层发布 | 发送目标位置/速度/力矩 |
| `/canX/joint_states` | `sensor_msgs/msg/JointState` | 上层订阅 | 当前通道电机反馈 |
| `/canX/status_feedback` | `khcan/msg/MotorStatusArray` | 上层订阅 | 故障、模式、在线状态与反馈时延 |
| `/canX/enable` | `std_srvs/srv/Trigger` | 上层调用 | 使能当前通道全部电机 |
| `/canX/disable` | `std_srvs/srv/Trigger` | 上层调用 | 失能当前通道全部电机 |
| `/canX/clear_error` | `std_srvs/srv/Trigger` | 上层调用 | 清除当前通道全部电机错误 |
| `/canX/set_zero` | `std_srvs/srv/Trigger` | 上层调用 | 设置当前通道全部电机零点 |
| `/canX/set_mode` | `khcan/srv/SetMotorMode` | 上层调用 | 设置当前通道全部或指定电机模式 |

`/canX/status_feedback` 中每个 `MotorStatus` 包含 `name`、`motor_id`、`mode`、
`motor_state`、`fault_code`、`online`、`feedback_age_ms`、`position`、`speed`、
`torque`、`current` 和 `temperature`。

`torque` 与 `current` 始终是不同物理量：协议直接反馈转矩时使用协议值；ENCOS 有型号 Kt 时由电流换算转矩；海泰/JC 只有电流反馈而没有可靠 Kt 时，`current` 使用 A，`torque` 发布为 NaN；Type5 只有驱动转矩反馈时，`torque` 使用 N·m，`current` 发布为 NaN。

`/canX/command` 使用 `JointState.name` 匹配 `motor.toml` 中的电机 `name`。推荐上层始终带 `name`，避免通道内顺序变动造成误发：
```bash
ros2 topic pub --once /can1/command sensor_msgs/msg/JointState \
"{name: ['left_hip_pitch_joint'], position: [0.2], velocity: [0.0], effort: [0.0]}"
```

如果 `name` 为空，则按该通道在 TOML 中的电机顺序匹配数组下标。`position`、`velocity`、`effort` 可只填需要更新的字段，未提供的字段会沿用上一条命令值。
`effort` 在 MIT/力矩模式表示 N·m；在 Type1 速度模式、Type7 电流模式以及 Type8 位置/速度模式表示电流限制 A，驱动按当前模式选择对应型号范围限幅。Type1 速度模式必须显式给出大于 `0 A` 的限制值，省略或填 `0` 会拒绝命令。

设置模式示例：
```bash
ros2 service call /can1/set_mode khcan/srv/SetMotorMode \
"{names: [], mode: 0}"
```

`names` 为空表示设置当前通道全部电机；非空时只设置指定名称的电机。

启动参数：
```bash
ros2 launch khcan motor_driver.launch.py \
  config:=/absolute/path/to/motor.toml \
  rate_hz:=100.0 \
  auto_enable:=false \
  default_mode:=-1 \
  command_timeout_ms:=100 \
  feedback_timeout_ms:=500
```

`auto_enable:=true` 只发送使能命令，不会隐式清错；需要清错时显式调用 `/canX/clear_error`。

## 安全、校验与状态新鲜度

- 配置加载会校验必填字段、数值类型、量程、增益、CAN ID，以及重复的逻辑编号、名称和反馈路由；失败时抛出异常并终止节点初始化。
- 反馈路由键由设备、标准/扩展帧格式和 CAN ID 共同组成，标准帧和扩展帧可以使用相同 ID。
- 命令订阅使用 QoS `KeepLast(1)`；发送前拒绝 NaN/Inf，并按位置、速度、力矩和增益范围限幅。
- `command_timeout_ms` 默认 `100`。电机收到命令后若超时，节点会发送失能命令；发送失败会在后续周期重试。设为 `0` 可关闭。
- `feedback_timeout_ms` 默认 `500`。只有协议成功解析的反馈才刷新时间；`MotorStatus.online` 和 `feedback_age_ms` 使用单调时钟计算，从未收到有效反馈时 age 为 `uint32` 最大值。
- CAN 接口进入 `ENETDOWN` 后会关闭套接字并停止接收线程。当前不自动重连，恢复接口后需重启节点。
- `BaseRobot` 内部状态不公开；使用 `GetMotorInfo()`、`GetMotorSnapshot()` 和 `GetCommand_N()` 获取线程安全快照。

## 当前目录结构
```text
khcan/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── motor.toml
│   ├── m4.toml
│   └── k1.toml
├── include/
├── launch/
│   ├── motor_driver.launch.py
│   ├── m4_bringup.launch.py
│   └── k1_bringup.launch.py
├── main/
│   ├── motor_driver_node.cpp
│   ├── show_status.cpp
│   └── test_mit_mode.cpp
├── srv/
│   └── SetMotorMode.srv
└── src/
```

## 代码框架对应关系
- `main/motor_driver_node.cpp`：整体底层驱动节点；单进程加载完整 TOML，并按 `chan` 暴露 `/canX/...` 接口
- `main/show_status.cpp`：状态查看入口；默认不清错、不使能、不退出失能
- `main/test_mit_mode.cpp`：通用 MIT 测试入口，自动适配 Type1/2/3/5/8 的 MIT 模式编号
- `config/motor.toml`：通用/开发默认配置
- `config/m4.toml`、`config/k1.toml`：各本体独立配置
- `srv/SetMotorMode.srv`：通道级设置模式服务
- `launch/motor_driver.launch.py`：整体底层驱动节点启动文件
- `launch/m4_bringup.launch.py`、`launch/k1_bringup.launch.py`：选择本体配置后复用通用启动文件
- `src/config_loader.cpp`：加载 TOML 配置
- `src/device.cpp`：SocketCAN 设备收发和各类 CAN 协议解析
- `src/robot.cpp`：机器人整体控制逻辑
- `include/`：头文件与类型定义

### SDK 下载
```bash
# 安装下载/解压工具（若未安装）
sudo apt install -y wget unzip

# 下载 SDK（latest）
wget https://gitee.com/ChengDu-KunHong/KH-UCANFD_Linux_SDK/releases/download/latest/KH-UCANFD_Linux_SDK.zip

# 解压并进入 SDK 目录
unzip KH-UCANFD_Linux_SDK.zip
cd KH-UCANFD_Linux_SDK_x.y.z/
```

### 编译与安装（脚本方式）
```bash
./build.sh
```

## 配置说明
程序运行时会通过包安装目录读取：
```text
share/khcan/config/motor.toml
```
M4/K1 bringup 会分别读取 `share/khcan/config/m4.toml` 和
`share/khcan/config/k1.toml`。源码中的原始配置文件位于：
```text
config/*.toml
```

多本体共用同一套驱动代码和型号表，本体 TOML 只保存该本体的关节名称、逻辑编号、CAN 路由、现场增益和软限位。不要为每个本体复制驱动代码，也不要把 K1/M4 的路由合并进一个配置；bringup 通过 `config` 参数选择单份 TOML。新增本体时新增 `config/<body>.toml`，并复用 `motor_driver.launch.py` 即可。

每个 TOML 中 `motors = [ {...}, {...} ]` 的每个内联表代表 1 个电机。
程序会按 `src/config_loader.cpp` 读取以下字段；身份和路由字段必须填写，
型号表已覆盖的协议映射字段可以省略：

### 字段总览
| 字段 | 含义 | 典型单位/范围 | 是否参与控制报文 |
|---|---|---|---|
| `num` | 逻辑编号（业务编号） | 正整数，整份配置内唯一 | 否（用于日志、状态标识和测试选择） |
| `name` | 电机名称 | 非空字符串，整份配置内唯一 | 是（ROS 命令按名称路由） |
| `type` | 电机型号名（如 `RS06`、`EC-A4310-P2-36`、`DM8009`、`M4438_30`、`HT3505-J8`、`JC`） | 字符串 | Type1/3/7/8/9 下用于自动匹配映射范围；Type5 下参与力矩/增益缩放 |
| `api_type` | 协议类型编号 | `1/2/3/5/7/8/9` | 是（决定走哪套协议） |
| `chan` | CAN 通道号 | 非负整数，如 `1` | 是（映射为 `can<chan>`） |
| `canid` | 电机 CAN ID | 正整数，具体上限由协议决定；Type8 为 `1~255` | 是 |
| `p_min` | 位置映射最小值 | 通常 rad | 是 |
| `p_max` | 位置映射最大值 | 通常 rad | 是 |
| `v_min` | 速度映射最小值 | 通常 rad/s | 是 |
| `v_max` | 速度映射最大值 | 通常 rad/s | 是 |
| `kp_min` | 比例增益映射最小值 | 协议定义范围内 | 是 |
| `kp_max` | 比例增益映射最大值 | 协议定义范围内 | 是 |
| `kd_min` | 微分增益映射最小值 | 协议定义范围内 | 是 |
| `kd_max` | 微分增益映射最大值 | 协议定义范围内 | 是 |
| `t_min` | 力矩映射最小值 | 通常 N·m（按驱动定义） | 是 |
| `t_max` | 力矩映射最大值 | 通常 N·m（按驱动定义） | 是 |
| `current_min` | 电流映射/限幅最小值 | A | 电流反馈解码和电流型命令限幅 |
| `current_max` | 电流映射/限幅最大值 | A | 电流反馈解码和电流型命令限幅 |
| `torque_constant` | 电流换算输出转矩的型号常数 | N·m/A | 有电流反馈时换算转矩 |
| `kp_in_use` | 上电默认使用的 `kp` | 浮点，必须在映射范围内 | 是 |
| `kd_in_use` | 上电默认使用的 `kd` | 浮点，必须在映射范围内 | 是 |
| `pos_min` | 运行时位置软限位最小值 | 通常 rad | 间接（发送前限幅） |
| `pos_max` | 运行时位置软限位最大值 | 通常 rad | 间接（发送前限幅） |

### 字段精简规则

- 每台电机必须保留 `num`、`name`、`type`、`api_type`、`chan`、`canid`。
- `kp_in_use`、`kd_in_use` 是关节现场调参值，不是电机型号常量；除当前不使用增益的 Type9 外应保留。
- 已知型号的 `p/v/kp/kd/t` 映射边界由型号表补齐，TOML 无需重复。显式填写时会覆盖型号表，并实际影响发送编码和反馈解码。
- 已知 ENCOS/RS 型号的 `current_min/current_max` 以及 ENCOS 的 `torque_constant` 同样由型号表补齐。未知型号只有在需要电流型命令或电流反馈时才显式填写；缺少有效电流范围时，驱动拒绝对应命令而不是使用错误单位发送。
- 未知型号必须提供完整映射边界。K1 的 `EC-A6416-P2-25` 已进入型号表：位置 `±12.5 rad`、速度 `±18 rad/s`、KP `0~500`、KD `0~5`、MIT/力矩范围 `±120 N·m`、反馈电流范围 `±60 A`、转矩常数 `2.74 N·m/A`。这些是不同物理量，不能互相代替。
- `pos_min`、`pos_max` 只在 `pos_min < pos_max` 时启用软限位；两者均为 `0` 与省略等价，应直接省略。
- M4/K1 的已知型号不在 TOML 中重复填写 `p/v/kp/kd/t/current` 型号常量；RS00 使用型号表的 `-14~14 N·m`。RS01 轮毂默认使用 Type1 的 MIT 模式 `0`。

### ENCOS 型号表

下表来自 `doc/ENCOS协议.pdf` 的 V1.19 型号参数表。所有型号的位置范围均为 `±12.5 rad`、速度范围均为 `±18 rad/s`、KP 均为 `0~500`；KD、MIT/力矩映射、电流反馈和转矩常数如下：

| 型号 | KD | 转矩范围 (N·m) | 电流范围 (A) | Kt (N·m/A) |
|---|---:|---:|---:|---:|
| `EC-A8112-P1-18` | `0~5` | `±90` | `±60` | `2.1` |
| `EC-A4310-P2-36` | `0~5` | `±30` | `±30` | `1.4` |
| `EC-A4315-P2-36` | `0~5` | `±70` | `±30` | `2.8` |
| `EC-A6408-P2-25` | `0~5` | `±60` | `±60` | `2.35` |
| `EC-A6416-P2-25` | `0~5` | `±120` | `±60` | `2.74` |
| `EC-A10020-P1-12` | `0~50` | `±150` | `±70` | `2.5` |
| `EC-A10020-P2-24` | `0~50` | `±300` | `±140` | `2.6` |
| `EC-A13715-P1-12.67` | `0~50` | `±320` | `±220` | `2.5` |
| `EC-A13720-P1-11.4` | `0~50` | `±400` | `±220` | `2.5` |

### `api_type` 对照
- `1`：Type1（灵足/富兴扩展帧协议）
- `2`：Type2（领控）
- `3`：Type3（达妙）
- `5`：Type5（高擎 16-bit ID 协议）
- `7`：Type7（海泰标准帧协议）
- `8`：Type8（ENCOS / 因克斯 EC-A 系列标准帧协议）
- `9`：Type9（JC 系列标准 CAN 舵机协议）

### 驱动能力确认
当前支持的 Type1/2/3/5/7/8/9 均走 `DeviceX` 的 SocketCAN/CAN FD 路径。

| `api_type` | 驱动能力 | 当前诊断入口 |
|---|---|---|
| `1` | 保留 Type1 协议发送与反馈解析 | `test_mit_mode` 可用于 MIT 模式简单验证 |
| `2` | 保留 Type2 协议发送与反馈解析 | `test_mit_mode` 可用于 MIT 模式简单验证 |
| `3` | 保留 Type3 协议发送与反馈解析 | `test_mit_mode` 可用于 MIT 模式简单验证 |
| `5` | 保留 Type5 / 高擎 CAN FD 协议发送与反馈解析 | `test_mit_mode` 可用于 MIT 模式简单验证 |
| `7` | 保留 Type7 / 海泰 Rev.3.07b0 标准 CAN 协议发送与反馈解析，含绝对位置和 MIT 模式 | 可通过 `show_status` 或 `motor_driver_node` 查看状态 |
| `8` | ENCOS / 因克斯 EC-A 系列标准 CAN 协议发送与反馈解析，支持 MIT/位置/速度/力矩 | `test_mit_mode` 可用于 MIT 模式简单验证 |
| `9` | JC 系列标准 CAN 舵机协议，支持位置/速度/力矩发送和状态回读 | 推荐通过 `motor_driver_node` 的 `/canX/...` 接口控制 |

也就是说，上层可通过 `motor_driver_node` 的通道接口控制整机，也可直接链接库后通过 `BaseRobot::Move_N()` / `BaseRobot::Move()` 进入统一发送路径；CAN 电机按各自 `api_type` 分发到 `DeviceX`。

### 关键说明
- `chan` 会转换成设备名 `can<chan>`。例如 `chan = 1` 会使用 `can1`。
- `type` 不参与协议分发（协议分发只看 `api_type`），但在 `api_type=1/3/7/8/9` 时会自动匹配映射范围；在 `api_type=5`（高擎）时会用于型号缩放适配：
  - 力矩发送使用型号补偿（`tqe_adjust` 思路）
  - 力矩回读使用型号还原（`tqe_restore` 思路）
  - MIT 模式的 `kp/kd` 也会做型号补偿
  - 若 `type` 未匹配已知型号，则回退为 `k=1.0,d=0.0`（等价无补偿）
- `api_type=8`（ENCOS / 因克斯 EC-A）使用经典标准 CAN 帧；`SetMode_N()` 支持 `0` MIT 混控、`1` 位置、`2` 速度、`3` 力矩。模式 1/2 下命令的 `torque` 字段表示电流限制（A），模式 0/3 下表示转矩（N·m）。反馈电流按型号电流范围解码，转矩按型号转矩常数换算。
- `api_type=9`（JC 系列）使用标准 CAN 帧；`SetMode_N()` 支持 `1~5`，其中 `1` 走速度命令、`5` 走力矩命令，`2~4` 按位置命令发送。`type = "JC"` 时会自动给出基础映射范围，`kp_in_use/kd_in_use` 可省略。
- JC 协议资料中没有明确的清错写命令；`ClearError_N()` 会触发一次故障寄存器查询并返回失败，避免上层把“已查询”误判为“已清除”。
- Type1/3/7/8 的型号参数按仓库内对应厂商协议维护：灵足/富兴支持 `RS00`~`RS06`、`CyberGear`；ENCOS/因克斯支持 `EC-A8112-P1-18`、`EC-A4310-P2-36`、`EC-A4315-P2-36`、`EC-A6408-P2-25`、`EC-A6416-P2-25`、`EC-A10020-P1-12`、`EC-A10020-P2-24`、`EC-A13715-P1-12.67`、`EC-A13720-P1-11.4`；达妙/达秒支持 `DM4310`、`DM4310_48V`、`DM4340`、`DM4340_48V`、`DM6006`、`DM8006`、`DM8009`、`DM10010L`、`DM10010`、`DMH3510`、`DMH6215`、`DMG6220`。
- 对 Type1/2/3/7/8，`kp_in_use` 和 `kd_in_use` 仍建议显式保留在 TOML 中，便于现场调参。Type9 当前不使用 `kp/kd`，可省略。`p/v/t/kp/kd` 的 `min/max` 可由型号表自动填写，也可以在 TOML 中显式覆盖。
- `api_type=7`（海泰 Rev.3.07b0）默认将 `send.mode=0` 映射为 `0xC2` 绝对位置控制；`SetMode_N()` 支持 `0` 绝对位置、`1` 电流、`2` 速度、`3` 相对位置、`4` MIT 模式。模式 1 需要配置有效的 `current_min/current_max`，缺失时命令会失败。普通查询交替使用 `0xA4` 复合状态和 `0xA3` 多圈位置。进入 MIT 模式后驱动先用单字节 `0xF0` 读取电机实际映射范围，在收到合法响应前拒绝发送 MIT 命令；`ConfigureHaitaiMitLimits_N()` 才会主动写入一组映射范围。
- Type7 海泰可在 `type` 中填写具体型号并自动加载命令限幅：`HT2205` 为 `95.5rad / 125.66rad/s / 0.06Nm`，`HT3505-J8` 为 `95.5rad / 32.04rad/s / 0.85Nm`，`HT4305` / `HT4305-J10` 为 `95.5rad / 41.89rad/s / 3.0Nm`，`HT4310-J10` 为 `95.5rad / 31.42rad/s / 1.0Nm`，`HT6010-J6` 为 `95.5rad / 70.16rad/s / 9.0Nm`。未知型号必须显式填写完整映射；TOML 显式字段优先覆盖型号表。MIT 帧本身仍以电机 `0xF0` 回读的实际范围编码和解码。
- `p/v/t/kp/kd` 的 `min/max` 既用于发送映射，也用于接收反解（不同 `api_type` 有差异，但都依赖这些边界）。
- `kp_in_use`、`kd_in_use` 会在初始化时拷贝到每个电机的发送缓存，后续可再通过接口动态修改。
- `pos_min < pos_max` 时作为关节位置软限位；两者都为 `0` 时使用协议位置范围。其他无效组合会在配置加载时被拒绝。
- `api_type=1`（灵足/富兴）当前只接受模式 `0`（运控/MIT）和 `2`（速度）。速度模式下 `Move_N()` 使用 `cmd.v` 写入 `0x700A spd_ref`，使用显式非零的 `abs(cmd.t)` 写入 `0x7018 limit_cur`；该值会按型号电流上限限幅，省略或为 `0` 时命令失败。模式 `1/3` 尚未实现完整命令路径，因此会明确返回失败，不会误发 MIT 帧。

Type5 当前内置的常见型号系数：`M3536_32`、`M4438_30`、`M4438_32`、`M4538_19`、`M5043_20`、`M5046_20`、`M5047_09`、`M5047_36`、`M6056_36`、`M7256_35`、`M60SG_35`、`M60BM_35`。

### 示例编写
灵足/富兴（按型号自动加载 MIT 映射范围）：{num = 1, name = "R_1", type = "RS06", api_type = 1, chan = 1, canid = 4, kp_in_use = 50, kd_in_use = 0.5, pos_min = 0.0, pos_max = 0.0}

领控（Type2）：{num = 2, name = "R_LK_1", type = "LK", api_type = 2, chan = 1, canid = 1, p_min = -12.5, p_max = 12.5, v_min = -18.0, v_max = 18.0, kp_min = 0.0, kp_max = 500.0, kd_min = 0.0, kd_max = 5.0, t_min = -30.0, t_max = 30.0, kp_in_use = 20, kd_in_use = 0.8, pos_min = 0.0, pos_max = 0.0}

ENCOS / 因克斯（按型号自动加载 MIT 映射范围）：{num = 8, name = "R_EC_1", type = "EC-A4310-P2-36", api_type = 8, chan = 1, canid = 1, kp_in_use = 20, kd_in_use = 0.8, pos_min = 0.0, pos_max = 0.0}

达妙/达秒（按型号自动加载 MIT 映射范围）：{num = 3, name = "R_DM_1", type = "DM8009", api_type = 3, chan = 1, canid = 1, kp_in_use = 1, kd_in_use = 1, pos_min = 0.0, pos_max = 0.0}

高擎（推荐填写具体型号）：{num = 7, name = "R_Arm", type = "M4438_30", api_type = 5, chan = 1, canid = 1,  kp_in_use = 150, kd_in_use = 0.2 }


海泰（按具体型号加载 MIT 默认限幅）：{num = 21, name = "R_HT_1", type = "HT3505-J8", api_type = 7, chan = 1, canid = 1, kp_in_use = 20.0, kd_in_use = 0.8, pos_min = -6.28, pos_max = 6.28}


JC 系列（Type9 标准 CAN 舵机）：{num = 41, name = "JC_SERVO_1", type = "JC", api_type = 9, chan = 1, canid = 1, pos_min = -1.5708, pos_max = 1.5708}

## 编译
建议在工作区根目录执行：
```bash
colcon build --packages-select khcan --symlink-install
source install/setup.bash
```

安装后会包含：
- `lib/libkhcan_driver.so`：可被其他工程链接的驱动库
- `include/*.hpp`：公开头文件
- `share/khcan/config/motor.toml`：默认配置文件
- `share/khcan/config/m4.toml`、`share/khcan/config/k1.toml`：本体配置文件
- `share/khcan/launch/*.launch.py`：通用及 M4/K1 bringup 启动文件
- `ros2 run khcan show_status`
- `ros2 run khcan motor_driver_node`
- `ros2 run khcan test_mit_mode`

### 作为驱动库植入其他工程
在下游 ROS 2 包中可直接 `find_package(khcan)` 并链接导出的 CMake target：
```cmake
find_package(khcan REQUIRED)

add_executable(my_controller src/my_controller.cpp)
target_link_libraries(my_controller
  khcan_driver
)
```

最小调用示例：
```cpp
#include "robot.hpp"

int main() {
    BaseRobot robot("/path/to/motor.toml");

    robot.Enable_N(0);

    MotorCmdVec cmd{};
    cmd.p = 1.0f;
    cmd.v = 0.0f;
    cmd.t = 1.0f;
    robot.Move_N(0, cmd);

    robot.Disable_N(0);
    return 0;
}
```

如果当前终端使用 Conda，推荐这样构建：
```bash
PYTHON_EXECUTABLE=/usr/bin/python3 colcon build \
  --packages-select khcan \
  --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

## 运行
推荐启动整体底层驱动节点：
```bash
ros2 launch khcan motor_driver.launch.py
```

也可以直接运行节点：
```bash
ros2 run khcan motor_driver_node --ros-args \
  -p config:=/absolute/path/to/motor.toml \
  -p rate_hz:=100.0 \
  -p auto_enable:=false \
  -p default_mode:=-1 \
  -p command_timeout_ms:=100 \
  -p feedback_timeout_ms:=500
```

诊断工具仍可单独运行：
```bash
ros2 run khcan show_status
ros2 run khcan show_status --config m4
ros2 run khcan show_status --config k1
ros2 run khcan test_mit_mode
```


## CAN 接口准备
程序依赖 `motor.toml` 中声明的 `canX` 接口。
运行前请确保对应接口已经存在，并且处于可用状态。

手动示例：
```bash
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up

sudo ip link set can2 down
sudo ip link set can2 type can bitrate 1000000
sudo ip link set can2 up

sudo ip link set can3 down
sudo ip link set can3 type can bitrate 1000000
sudo ip link set can3 up

sudo ip link set can4 down
sudo ip link set can4 type can bitrate 1000000
sudo ip link set can4 up

sudo ip link set can5 down
sudo ip link set can5 type can bitrate 1000000
sudo ip link set can5 up

sudo ip link set can6 down
sudo ip link set can6 type can bitrate 1000000
sudo ip link set can6 up
```

**Type5（高擎）在 MIT 模式下会发送 CAN FD 帧。若你使用 `main/test_mit_mode.cpp`（可执行名 `test_mit_mode`），建议接口按 FD 模式配置：**
```bash
sudo ip link set can0 type can bitrate 1000000 dbitrate 1000000 fd on
sudo ip link set can0 up
```

Type5 MIT 默认使用 **CAN FD 且不带 BRS**（兼容性更好）。  

Type5 `mode=0` 使用 MIT2 组合帧（设模式 + 写 `pos/vel/tqe` + 写 `kp/kd` + 查询状态），回包会解析 `0x24/0x28/0x2C`（并兼容 `0x27`）。


代码中也会在接口存在但未启动时尝试拉起接口；如果权限不足，会提示你：
- 使用 `sudo` 运行程序
- 或给可执行文件添加 `CAP_NET_RAW` / `CAP_NET_ADMIN`

## 调整查询频率
`show_status` 默认 10 Hz，可通过参数调整：
```bash
ros2 run khcan show_status --hz 20
```

按本体选择安装在 `share/khcan/config` 中的 TOML：
```bash
ros2 run khcan show_status --config m4
ros2 run khcan show_status --config k1.toml
```

自定义配置使用完整路径；原有的位置参数写法仍然兼容：
```bash
ros2 run khcan show_status --config /absolute/path/to/custom.toml
ros2 run khcan show_status /absolute/path/to/custom.toml
```

默认只查询状态。如果确认现场安全并且需要清错/使能：
```bash
ros2 run khcan show_status --clear-errors --enable
```

## 当前仓库不包含的内容
根据当前代码，下面这些内容并不在仓库内，因此不再作为使用方式说明：
- IMU 相关包与串口配置
- `yesense_interface` / `yesense_std_ros2`
- `remote` 遥控模块源码
- `demo_socketcan` 节点
- `setup_can_from_toml.sh` 脚本

## 常见问题
- `ament_index_cpp::get_package_share_directory("khcan")` 找不到包：
  先确认已经执行过：
  ```bash
  source install/setup.bash
  ```
- 修改了 `config/motor.toml` 但运行结果没变化：
  若未使用 `--symlink-install`，请重新编译并重新 `source`。
- 报错 `canX` 打不开：
  先确认 CAN 驱动、接口名、波特率和权限是否正确。
- 报错 `sendExtendedId(Fd)Frame: No buffer space available`：
  先检查 `ip -details -statistics link show canX`，重点看 `bus-errors`/`error-passive`/`bus-off`。
  Type5 MIT 下可优先确认 `HQ_CANFD_BRS`（默认不设置即关闭），并适当增大 `txqueuelen`。
- `test_mit_mode` 有发送但电机不动、`p/v/t` 长时间不变：
  常见是总线未应答（波特率/BRS/接线/CAN ID不一致）。建议先抓包确认是否有电机回包，再核对 `type` 是否填写为具体型号（Type5 下影响力矩与增益缩放）。
