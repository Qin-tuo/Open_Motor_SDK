# khcan 使用说明
# ***!!!注意灵足/富兴，因克斯，达秒对于不同电机型号的p_min,p_max等均不相同 必须严格按照电机型号填写对应数据!!! 否则会导致电机比例映射不正常***
**具体可在 https://can.robotsfan.com/ 此网站查看**
# **Type5（高擎）和 Type6（PFL28）默认使用 CAN FD。**

## 简介
`khcan` 是一个基于 ROS 2 `ament_cmake` 的 SocketCAN 电机控制包。
当前仓库中可用的入口程序有：
- `show_status`：从 `config/motor.toml` 读取配置，初始化并周期性查询所有电机状态
- `test_mit_mode`：根据 `api_type` 自动切换到 MIT 模式（Type1/2/3/5），对单个电机执行正弦摆动测试
- `test_pfl28`：针对 `api_type=6`（PFL28）发送位置+电流命令并打印反馈

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
│   └── test_mit_mode.cpp
└── src/
```

## 代码框架对应关系
- `main/show_status.cpp`：程序入口，执行初始化、清错、使能和状态打印循环
- `main/test_mit_mode.cpp`：通用 MIT 测试入口，自动适配 Type1/2/3/5 的 MIT 模式编号
- `main/test_pfl28.cpp`：PFL28 测试入口（正弦位置指令 + 限幅电流）
- `config/motor.toml`：电机编号、类型、CAN 通道、CAN ID 与控制参数配置
- `src/config_loader.cpp`：加载 TOML 配置
- `src/device.cpp`：SocketCAN 设备收发、接口打开与状态检查
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
| `type` | 电机型号名（如 `LZRS06`、`M4438_30`、`PFL28`） | 字符串 | Type5 下会参与力矩/增益缩放 |
| `api_type` | 协议类型编号 | `1/2/3/4/5/6` | 是（决定走哪套协议） |
| `chan` | CAN 通道号 | 正整数，如 `1` | 间接（映射接口名） |
| `canid` | 电机 CAN ID | 通常 `1~255`（按驱动手册） | 是 |
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
| `kp_in_use` | 上电默认使用的 `kp` | 浮点 | 是（初始化写入发送结构） |
| `kd_in_use` | 上电默认使用的 `kd` | 浮点 | 是（初始化写入发送结构） |
| `pos_min` | 运行时位置软限位最小值 | 通常 rad | 间接（发送前限幅） |
| `pos_max` | 运行时位置软限位最大值 | 通常 rad | 间接（发送前限幅） |

### `api_type` 对照
- `1`：Type1（灵足/富兴扩展帧协议）
- `2`：Type2（领控）
- `3`：Type3（达妙）
- `4`：Type4（RoboMaster C620 协议）
- `5`：Type5（高擎 16-bit ID 协议）
- `6`：Type6（AgiBot PFL28/L28，`pos(float)+current(float)`）

### 关键说明
- `chan` 会转换成设备名 `can<chan>`。例如 `chan = 1` 会使用 `can1`。
- `type` 不参与协议分发（协议分发只看 `api_type`），但在 `api_type=5`（高擎）时会用于型号缩放适配：
  - 力矩发送使用型号补偿（`tqe_adjust` 思路）
  - 力矩回读使用型号还原（`tqe_restore` 思路）
  - MIT 模式的 `kp/kd` 也会做型号补偿
  - 若 `type` 未匹配已知型号，则回退为 `k=1.0,d=0.0`（等价无补偿）
- `api_type=6`（PFL28）默认将 `send.position` 作为位置命令、`send.torque` 作为电流命令（A）。
- `p/v/t/kp/kd` 的 `min/max` 既用于发送映射，也用于接收反解（不同 `api_type` 有差异，但都依赖这些边界）。
- `kp_in_use`、`kd_in_use` 会在初始化时拷贝到每个电机的发送缓存，后续可再通过接口动态修改。
- `pos_min`、`pos_max` 用于 `Move/Move_N` 的发送前限幅；若 `pos_min >= pos_max`，限幅逻辑会被跳过（等价于不启用软限位）。

Type5 当前内置的常见型号系数：`M3536_32`、`M4438_30`、`M4438_32`、`M4538_19`、`M5043_20`、`M5046_20`、`M5047_09`、`M5047_36`、`M6056_36`、`M7256_35`、`M60SG_35`、`M60BM_35`。

### 示例编写
灵足：{num = 1, name = "R_1", type = "LZRS06", api_type = 1, chan = 1, canid = 4, p_min = -12.57, p_max = 12.57, v_min = -50, v_max = 50, kp_min = 0, kp_max = 5000, kd_min = 0, kd_max = 100, t_min = -36, t_max = 36, kp_in_use = 50, kd_in_use = 0.5, pos_min = 0.0, pos_max = 0.0}

达秒：{num = 1, name = "R_F_1", type = "DMJ8009P", api_type = 3, chan = 1, canid = 1, p_min = -12.5, p_max = 12.5, v_min = -45, v_max = 45, kp_min = 0, kp_max = 500, kd_min = 0, kd_max = 5, t_min = -54, t_max = 54, kp_in_use = 1, kd_in_use = 1, pos_min = 0.0, pos_max = 0.0 }

高擎（推荐填写具体型号）：{num = 7, name = "R_Arm", type = "M4438_30", api_type = 5, chan = 1, canid = 1,  kp_in_use = 150, kd_in_use = 0.2 }

PFL28（位置+电流控制）：{num = 11, name = "R_PUSHROD", type = "PFL28", api_type = 6, chan = 0, canid = 1, p_min = 0.0, p_max = 9.5, t_min = 0.0, t_max = 2.5, pos_min = 0.0, pos_max = 9.5}

## 编译
建议在工作区根目录执行：
```bash
colcon build --packages-select khcan --symlink-install
source install/setup.bash
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
ros2 run khcan test_pfl28   # 仅测试 api_type=6 电机
```

`test_pfl28` 常用调参环境变量：
```bash
export PFL28_TEST_CURRENT=2.0   # 推杆电流指令(A)
export PFL28_UP_CURRENT=2.0     # 上行阶段电流
export PFL28_DOWN_CURRENT=2.5   # 下行阶段电流（建议先给大一点）
export PFL28_ALLOW_NEG_CURRENT=0 # 若固件需要负电流回缩可设为1，并把 DOWN_CURRENT 设为负值
export PFL28_WARMUP_SEC=6       # 上电自动回零等待时间
export PFL28_FORCE_ZERO_SEC=2   # 启动后先强制 set_pos(0) 的时间（默认2s，建议保留）
export PFL28_FEEDBACK_WAIT_SEC=2 # 起步前等待完整状态回包的时间，避免用默认0作为反馈
export PFL28_STALL_ABORT_SEC=4   # 检测到“有误差但零电流且位置不变”超过该时间就报错退出；设0可禁用
export PFL28_PHASE_TIMEOUT_SEC=6 # 单阶段最长持续时间（到不了位也会换向）
export PFL28_SWITCH_EPS=0.03     # 到位误差阈值
export PFL28_STABLE_COUNT=8      # 连续到位判定次数（20ms*8≈160ms）
export PFL28_CMD_PERIOD_MS=20    # 发送周期(ms)，可调大到50/100排查节奏问题
export PFL28_MAX_STEP=0.05       # 每20ms最大位置步长，避免大跳变被拒绝
export PFL28_HIGH_POS=7.6       # 高位目标(0~10)
export PFL28_LOW_POS=1.9        # 低位目标(0~10)
# L28 推荐范围：pos 0~9.5，current 0~2.5（默认 CAN FD）。
export PFL28_USE_CANFD=1         # 1=CANFD(默认), 0=经典CAN(仅排障临时使用)
# 仅在 PFL28_USE_CANFD=1 时，BRS 生效：
export PFL28_CANFD_BRS=0
ros2 run khcan test_pfl28
```

若日志持续出现以下组合：
- `fb_cur≈0`
- `fb_pos` 长时间几乎不变
- 且 `cmd_pos` 与 `fb_pos` 误差较大

`test_pfl28` 现在会在超时后主动退出并提示“使能链/电源链”问题。该场景通常不是协议发送失败，而是执行器未真正进入可出力状态（如电源、ESTOP、或上电自检/回零异常）。

扫描 PFL28 实际 ID（点对点模式）：
```bash
# 用法: scan_pfl28_id.sh <ifname> <start_id> <end_id>
./scripts/scan_pfl28_id.sh can2 1 127

# 可选参数（环境变量）
PFL28_SCAN_BRS=0 PFL28_SCAN_TRIES=3 PFL28_SCAN_WAIT_MS=80 ./scripts/scan_pfl28_id.sh can2 1 127
```

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
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up
```
若现场链路兼容性排障，可临时改成经典 CAN（`PFL28_USE_CANFD=0`）。
Type6 协议帧为标准 ID（`canid`）+ 8 字节数据：`position(float32 LE)` + `current(float32 LE)`。

代码中也会在接口存在但未启动时尝试拉起接口；如果权限不足，会提示你：
- 使用 `sudo` 运行程序
- 或给可执行文件添加 `CAP_NET_RAW` / `CAP_NET_ADMIN`

## 调整查询频率
状态查询频率在 `main/show_status.cpp` 中写死为：
```cpp
double target_hz = 50.0;
```
如需修改轮询频率，可直接编辑该值后重新编译。

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
