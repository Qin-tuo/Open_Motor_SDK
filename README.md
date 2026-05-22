# khcan 使用说明
# ***!!!注意灵足/富兴、因克斯、达妙/达秒、海泰不同型号的 p/v/t/kp/kd 映射范围不同，必须使用正确型号或显式填写完整映射参数，否则会导致电机比例映射不正常!!!***
**驱动已内置 https://can.robotsfan.com/ 中常见型号参数；TOML 中填写具体 `type` 即可自动匹配，未知型号才需要显式填写 `p_min/p_max/v_min/v_max/kp_min/kp_max/kd_min/kd_max/t_min/t_max`。**
# **Type5（高擎）和 Type6（PFL28）默认使用 CAN FD。**

## 简介
`khcan` 是一个基于 ROS 2 `ament_cmake` 的电机/舵机驱动包，主要支持 SocketCAN 电机，同时通过 Type4 接入飞特 TTL 串口舵机。
当前包会安装可复用驱动库 `libkhcan_driver.so`，其他 ROS 2 工程可直接链接该库进行控制；`main/` 下仅保留少量诊断入口。
当前仓库中可用的入口程序有：
- `show_status`：从 `config/motor.toml` 读取配置，默认只周期性查询所有电机状态；清错、使能、退出失能必须显式加参数
- `test_mit_mode`：根据 `api_type` 自动切换到 MIT 模式（Type1/2/3/5/8），对单个电机执行正弦摆动测试
- `test_rs01_speed_swing`：RS01 轮电机走速度模式，其余关节先回零再小幅往复摆动

## 当前目录结构
```text
khcan/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── motor.toml
├── include/
├── main/
│   ├── show_status.cpp
│   ├── test_mit_mode.cpp
│   └── test_rs01_speed_swing.cpp
└── src/
```

## 代码框架对应关系
- `main/show_status.cpp`：状态查看入口；默认不清错、不使能、不退出失能
- `main/test_mit_mode.cpp`：通用 MIT 测试入口，自动适配 Type1/2/3/5/8 的 MIT 模式编号
- `main/test_rs01_speed_swing.cpp`：RS01 速度模式 + 8 个关节小幅摆动测试入口
- `config/motor.toml`：电机编号、类型、CAN 通道、CAN ID 与控制参数配置
- `src/config_loader.cpp`：加载 TOML 配置
- `src/device.cpp`：SocketCAN 设备收发、海泰协议辅助和飞特 TTL 串口舵机协议接入
- `src/robot.cpp`：机器人整体控制逻辑
- `include/`：头文件与类型定义

## 依赖
### ROS 2 依赖
当前包在 `package.xml` 中声明的依赖很少：
- `ament_cmake`
- `ament_index_cpp`

### 系统依赖
代码直接使用 Linux SocketCAN 相关头文件与接口，因此需要：
- Linux 系统
- 可用的 CAN 网络接口，如 `can1`、`can2`
- Type4 飞特舵机需要可用串口设备，如 `/dev/ttyUSB0`
- 编译工具链，如 `g++`、`cmake`

如果你的环境用的是 Conda Python，`colcon build` 可能会因为缺少 `catkin_pkg` 失败。此时可改用系统 Python 构建，或在当前 Python 环境中安装该依赖。

## Linux SDK 驱动安装
如果当前系统还没有安装昆宏 Linux SDK 驱动，可先完成以下步骤，再继续本仓库的编译与运行。

### 系统环境需求
1. Linux 系统运行 32 位或 64 位内核
2. `make`、`gcc` 编译工具
3. Linux 内核头文件包（需与当前内核版本匹配）
4. `g++` 及 `libstdc++` 相关库
5. `libpopt-dev`

### 依赖安装命令
```bash
# 1) 安装编译基础工具
sudo apt update && sudo apt install -y build-essential g++

# 2) 安装内核头文件（必须与当前内核版本匹配）
sudo apt install -y linux-headers-$(uname -r)

# 3) 安装 libpopt 依赖
sudo apt install -y libpopt-dev

# 4) 校验 gcc 版本（需与内核编译版本一致）
cat /boot/config-$(uname -r) | grep -i "gcc_version"
# 示例输出：CONFIG_GCC_VERSION=120300（表示需安装 gcc-12）
# 若版本不匹配：sudo apt install -y gcc-12
```

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

`build.sh` 常用参数：
- `./build.sh`：安装驱动
- `./build.sh -c`：交叉编译驱动
- `./build.sh -rules`：安装驱动并加载 udev 命名规则
- `./build.sh -u` / `./build.sh -uninstall`：卸载驱动
- `./build.sh -h`：显示帮助

### 编译与安装（手动方式）
```bash
sudo make clean
sudo make netdev
sudo make install
```

### 驱动加载验证
```bash
# 加载 kcan 模块并确认已加载
sudo modprobe kcan
lsmod | grep kcan

# 确认 CAN 接口已识别（如 can0/can1）
ip -d link show

# 确认 USB 设备已绑定到 kcan 驱动
lsusb -t
```

### Ubuntu 额外依赖
```bash
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev
```

### (可选) ROS2 封装相关依赖
```bash
sudo apt-get install net-tools
sudo apt-get install can-utils
sudo apt-get install ros-jazzy-can-msgs
sudo apt-get install ros-jazzy-ros2-socketcan
```

## 配置说明
程序运行时会通过包安装目录读取：
```text
share/khcan/config/motor.toml
```
源码中的原始配置文件位于：
```text
config/motor.toml
```

`motor.toml` 中 `motors = [ {...}, {...} ]` 的每个内联表代表 1 个电机。  
程序会按 `src/config_loader.cpp` 固定读取以下字段（缺失会报错退出）：

### 字段总览
| 字段 | 含义 | 典型单位/范围 | 是否参与控制报文 |
|---|---|---|---|
| `num` | 逻辑编号（业务编号） | 整数 | 否（仅上层标识） |
| `name` | 电机名称（便于日志识别） | 字符串 | 否 |
| `type` | 电机/舵机型号名（如 `RS06`、`EC-A4310-P2-36`、`DM8009`、`SCS0037-C001`、`M4438_30`、`PFL28`、`HT3505-J8`） | 字符串 | Type1/3/4/7/8 下用于自动匹配映射范围；Type5 下参与力矩/增益缩放 |
| `api_type` | 协议类型编号 | `1/2/3/4/5/6/7/8` | 是（决定走哪套协议） |
| `chan` | CAN 通道号 | 正整数，如 `1` | CAN 电机使用，Type4 飞特串口舵机不填 |
| `port` | 串口设备路径 | 如 `/dev/ttyUSB0` | Type4 飞特串口舵机使用 |
| `baud` | 串口波特率 | SCS0037 默认 `500000` | Type4 飞特串口舵机使用，可省略 |
| `canid` | 总线 ID | CAN 电机为 CAN ID；Type4 为飞特舵机 ID | 是 |
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
| `kp_in_use` | 上电默认使用的 `kp` | 浮点 | 是（Type4 暂保留但不参与飞特位置指令） |
| `kd_in_use` | 上电默认使用的 `kd` | 浮点 | 是（Type4 暂保留但不参与飞特位置指令） |
| `pos_min` | 运行时位置软限位最小值 | 通常 rad | 间接（发送前限幅） |
| `pos_max` | 运行时位置软限位最大值 | 通常 rad | 间接（发送前限幅） |

### `api_type` 对照
- `1`：Type1（灵足/富兴扩展帧协议）
- `2`：Type2（领控）
- `3`：Type3（达妙）
- `4`：Type4（飞特 TTL 串口舵机协议，适配微雪/USB TTL 半双工控制板）
- `5`：Type5（高擎 16-bit ID 协议）
- `6`：Type6（AgiBot PFL28/L28，`pos(float)+current(float)`）
- `7`：Type7（海泰标准帧协议）
- `8`：Type8（ENCOS / 因克斯 EC-A 系列标准帧协议）

### 驱动能力确认
当前清理只删除了独立测试/排障入口，不影响底层电机驱动能力。Type1/2/3/5/6/7/8 仍走 `DeviceX` 的 SocketCAN/CAN FD 路径；Type4 已切换为飞特串口舵机路径，不再使用 SocketCAN。

| `api_type` | 驱动能力 | 当前诊断入口 |
|---|---|---|
| `1` | 保留 Type1 协议发送与反馈解析 | `test_mit_mode` 可用于 MIT 模式简单验证 |
| `2` | 保留 Type2 协议发送与反馈解析 | `test_mit_mode` 可用于 MIT 模式简单验证 |
| `3` | 保留 Type3 协议发送与反馈解析 | `test_mit_mode` 可用于 MIT 模式简单验证 |
| `4` | 飞特 TTL 串口舵机发送与反馈解析，当前内置 `SCS0037-C001` 参数 | `test_feetech_servo` 可用于小幅摆动验证 |
| `5` | 保留 Type5 / 高擎 CAN FD 协议发送与反馈解析 | `test_mit_mode` 可用于 MIT 模式简单验证 |
| `6` | 保留 Type6 / PFL28 协议发送与反馈解析 | 独立 PFL28 诊断入口已删除，请通过库 API 控制 |
| `7` | 保留 Type7 / 海泰 Rev.3.07b0 标准 CAN 协议发送与反馈解析，含绝对位置和 MIT 模式 | 可通过 `show_status` / `test_haitai_mode` 查看状态 |
| `8` | ENCOS / 因克斯 EC-A 系列标准 CAN 协议发送与反馈解析，支持 MIT/位置/速度/力矩 | `test_mit_mode` 可用于 MIT 模式简单验证 |

也就是说，上层仍通过 `BaseRobot::Move_N()` / `BaseRobot::Move()` 进入统一发送路径；CAN 电机按各自 `api_type` 分发到 `DeviceX`，Type4 飞特舵机会分发到串口驱动。

### 关键说明
- `chan` 会转换成设备名 `can<chan>`。例如 `chan = 1` 会使用 `can1`；Type4 飞特串口舵机不用 `chan`，改填 `port`。
- `type` 不参与协议分发（协议分发只看 `api_type`），但在 `api_type=1/3/4/7/8` 时会自动匹配映射范围；在 `api_type=5`（高擎）时会用于型号缩放适配：
  - 力矩发送使用型号补偿（`tqe_adjust` 思路）
  - 力矩回读使用型号还原（`tqe_restore` 思路）
  - MIT 模式的 `kp/kd` 也会做型号补偿
  - 若 `type` 未匹配已知型号，则回退为 `k=1.0,d=0.0`（等价无补偿）
- `api_type=6`（PFL28）默认将 `send.position` 作为位置命令、`send.torque` 作为电流命令（A）。
- `api_type=8`（ENCOS / 因克斯 EC-A）使用经典标准 CAN 帧；`SetMode_N()` 支持 `0` MIT 混控、`1` 位置、`2` 速度、`3` 力矩/电流，`send.position/speed/torque/kp/kd` 按 EC-A 型号映射范围打包。
- `api_type=4`（飞特 SCS0037-C001）默认将 `send.position` 按 `0~4.712389rad` 映射到舵机 `0~1023` 位置值；`send.speed` 按 rad/s 转成飞特速度值，填 `0` 表示不限制/由舵机默认处理；`send.torque` 暂不参与飞特位置指令。
- 修改飞特舵机 ID 前，必须确保总线上只接一个舵机，避免多个出厂默认 `ID=1` 的舵机被同时改成同一个 ID。
- Type1/3/7/8 的内置型号参数来自 `https://can.robotsfan.com/`：灵足/富兴支持 `RS00`~`RS06`、`CyberGear`；ENCOS/因克斯支持 `EC-A8112-P1-18`、`EC-A4310-P2-36`、`EC-A6408-P2-25`、`EC-A10020-P1-12`、`EC-A10020-P2-24`、`EC-A13715-P1-12.67`、`EC-A13720-P1-11.4`；达妙/达秒支持 `DM4310`、`DM4310_48V`、`DM4340`、`DM4340_48V`、`DM6006`、`DM8006`、`DM8009`、`DM10010L`、`DM10010`、`DMH3510`、`DMH6215`、`DMG6220`。
- 对 Type1/2/3/4/7/8，`kp_in_use` 和 `kd_in_use` 仍建议显式保留在 TOML 中，便于现场调参；Type4 当前不使用 kp/kd 做闭环控制，但保留字段以兼容统一配置。`p/v/t/kp/kd` 的 `min/max` 可由型号表自动填写，也可以在 TOML 中显式覆盖。
- `api_type=7`（海泰 Rev.3.07b0）默认将 `send.mode=0` 映射为 `0xC2` 绝对位置控制；`SetMode_N()` 支持 `0` 绝对位置、`1` 电流、`2` 速度、`3` 相对位置、`4` MIT 模式。海泰 `QueryPos` 使用 `0xA4` 复合状态查询，能同时回读位置、速度、电流和温度；驱动层不会在 `Enable_N()` 或 `SetMode_N()` 中自动查询/写入 `0xF0`，只有显式调用 `ConfigureHaitaiMitLimits_N()` 或上层策略时才会发送 `0xF0`。
- Type7 海泰可在 `type` 中填写具体型号并自动使用 MIT 默认限幅：`HT2205` 为 `95.5rad / 125.66rad/s / 0.06Nm`，`HT3505-J8` 为 `95.5rad / 32.04rad/s / 0.85Nm`，`HT4305` / `HT4305-J10` 为 `95.5rad / 41.89rad/s / 3.0Nm`，`HT4310-J10` 为 `95.5rad / 31.42rad/s / 1.0Nm`，`HT6010-J6` 为 `95.5rad / 70.16rad/s / 9.0Nm`。未知型号回退到协议默认 `95.5rad / 45rad/s / 18Nm`；TOML 中显式填写的 `p/v/t/kp/kd` 字段始终优先覆盖内置默认值。
- `p/v/t/kp/kd` 的 `min/max` 既用于发送映射，也用于接收反解（不同 `api_type` 有差异，但都依赖这些边界）。
- `kp_in_use`、`kd_in_use` 会在初始化时拷贝到每个电机的发送缓存，后续可再通过接口动态修改。
- `pos_min`、`pos_max` 用于 `Move/Move_N` 的发送前限幅；若 `pos_min >= pos_max`，限幅逻辑会被跳过（等价于不启用软限位）。
- `api_type=1`（灵足/富兴）`SetMode_N(..., 2)` 会切到速度模式；随后 `Move_N()` 使用 `cmd.v` 写入 `0x700A spd_ref`，使用 `abs(cmd.t)` 写入 `0x7018 limit_cur`（未填写或为 0 时使用配置中的正向 `t_max`，并限制到协议 `0~43A`）。其他模式仍使用通信类型 1 的运控/MIT 帧。

Type5 当前内置的常见型号系数：`M3536_32`、`M4438_30`、`M4438_32`、`M4538_19`、`M5043_20`、`M5046_20`、`M5047_09`、`M5047_36`、`M6056_36`、`M7256_35`、`M60SG_35`、`M60BM_35`。

### 示例编写
灵足/富兴（按型号自动加载 MIT 映射范围）：{num = 1, name = "R_1", type = "RS06", api_type = 1, chan = 1, canid = 4, kp_in_use = 50, kd_in_use = 0.5, pos_min = 0.0, pos_max = 0.0}

领控（Type2）：{num = 2, name = "R_LK_1", type = "LK", api_type = 2, chan = 1, canid = 1, p_min = -12.5, p_max = 12.5, v_min = -18.0, v_max = 18.0, kp_min = 0.0, kp_max = 500.0, kd_min = 0.0, kd_max = 5.0, t_min = -30.0, t_max = 30.0, kp_in_use = 20, kd_in_use = 0.8, pos_min = 0.0, pos_max = 0.0}

ENCOS / 因克斯（按型号自动加载 MIT 映射范围）：{num = 8, name = "R_EC_1", type = "EC-A4310-P2-36", api_type = 8, chan = 1, canid = 1, kp_in_use = 20, kd_in_use = 0.8, pos_min = 0.0, pos_max = 0.0}

达妙/达秒（按型号自动加载 MIT 映射范围）：{num = 3, name = "R_DM_1", type = "DM8009", api_type = 3, chan = 1, canid = 1, kp_in_use = 1, kd_in_use = 1, pos_min = 0.0, pos_max = 0.0}

高擎（推荐填写具体型号）：{num = 7, name = "R_Arm", type = "M4438_30", api_type = 5, chan = 1, canid = 1,  kp_in_use = 150, kd_in_use = 0.2 }

PFL28（位置+电流控制）：{num = 11, name = "R_PUSHROD", type = "PFL28", api_type = 6, chan = 0, canid = 1, p_min = 0.0, p_max = 9.5, t_min = 0.0, t_max = 2.5, pos_min = 0.0, pos_max = 9.5}

海泰（按具体型号加载 MIT 默认限幅）：{num = 21, name = "R_HT_1", type = "HT3505-J8", api_type = 7, chan = 1, canid = 1, kp_in_use = 20.0, kd_in_use = 0.8, pos_min = -6.28, pos_max = 6.28}

飞特 SCS0037-C001（Type4 串口舵机，不走 SocketCAN）：{num = 31, name = "FT_SERVO_1", type = "SCS0037-C001", api_type = 4, port = "/dev/ttyUSB0", baud = 500000, canid = 1, kp_in_use = 0.0, kd_in_use = 0.0, pos_min = 0.0, pos_max = 4.712389}

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
- `ros2 run khcan show_status`
- `ros2 run khcan test_mit_mode`
- `ros2 run khcan test_rs01_speed_swing`

### 作为驱动库植入其他工程
在下游 ROS 2 包中可直接 `find_package(khcan)` 并链接导出的 CMake target：
```cmake
find_package(khcan REQUIRED)

add_executable(my_controller src/my_controller.cpp)
target_link_libraries(my_controller
  khcan::khcan_driver
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
当前仓库没有 `launch` 文件，直接运行可执行程序即可：
```bash
ros2 run khcan show_status
ros2 run khcan test_mit_mode
ros2 run khcan test_rs01_speed_swing --wheel-speed 2.0 --wheel-current 3.0 --amp 0.20 --freq 0.25
```

PFL28 独立排障入口已删除。Type6/PFL28 驱动仍保留在库中，业务代码请通过 `BaseRobot` 直接发送位置/电流命令。

## CAN 接口准备
程序依赖 `motor.toml` 中声明的 `canX` 接口。
运行前请确保对应接口已经存在，并且处于可用状态。

手动示例：
```bash
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up

sudo ip link set can2 type can bitrate 1000000
sudo ip link set can2 up

sudo ip link set can3 type can bitrate 1000000
sudo ip link set can3 up

sudo ip link set can4 type can bitrate 1000000
sudo ip link set can4 up

sudo ip link set can5 type can bitrate 1000000
sudo ip link set can5 up

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

PFL28（Type6）建议按 CAN FD 配置：
```bash
sudo ip link set can0 type can bitrate 1000000 sample-point 0.8 dbitrate 5000000 dsample-point 0.75 fd on
sudo ip link set can0 up
```
Type6 默认采用智元/Xyber 风格：标准 ID `0` 的 64 字节 CAN FD 广播帧，同一 CAN 设备上的 PFL28 会按 `canid - 1` 一起写入 8 字节槽位：`position(float32 LE)` + `current(float32 LE)`。如需退回 P2P 标准 ID `canid` + 8 字节帧，可设置 `PFL28_XYBER_MODE=0`。

代码中也会在接口存在但未启动时尝试拉起接口；如果权限不足，会提示你：
- 使用 `sudo` 运行程序
- 或给可执行文件添加 `CAP_NET_RAW` / `CAP_NET_ADMIN`

## 调整查询频率
`show_status` 默认 10 Hz，可通过参数调整：
```bash
ros2 run khcan show_status --hz 20
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
- `launch` 启动文件
- `setup_can_from_toml.sh` 脚本

## 常见问题
- `ament_index_cpp::get_package_share_directory("khcan")` 找不到包：
  先确认已经执行过：
  source install/setup.bash
```raw
- 修改了 `config/motor.toml` 但运行结果没变化：
  若未使用 `--symlink-install`，请重新编译并重新 `source`。
- 报错 `canX` 打不开：
  先确认 CAN 驱动、接口名、波特率和权限是否正确。
- 报错 `sendExtendedId(Fd)Frame: No buffer space available`：
  先检查 `ip -details -statistics link show canX`，重点看 `bus-errors`/`error-passive`/`bus-off`。
  Type5 MIT 下可优先确认 `HQ_CANFD_BRS`（默认不设置即关闭），并适当增大 `txqueuelen`。
- `test_mit_mode` 有发送但电机不动、`p/v/t` 长时间不变：
  常见是总线未应答（波特率/BRS/接线/CAN ID不一致）。建议先抓包确认是否有电机回包，再核对 `type` 是否填写为具体型号（Type5 下影响力矩与增益缩放）。

```
