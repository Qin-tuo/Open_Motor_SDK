# SocketCAN + rmrobot 快速使用说明

## 0. 项目目录（新结构）
- `core/`：`rmrobot` 主包（启动文件、配置、核心控制代码）
- `imu/yesense_interface/`：IMU 消息定义包
- `imu/yesense_std_ros2/`：IMU 驱动节点包
- `imu/wheeltec_udev.sh`：IMU udev 规则脚本
- `remote/`：SBUS 遥控接收模块源码

## 1. Linux_SDK驱动安装

### 1.1 系统环境需求
1. Linux系统运行32位或64位内核
2. `make`、`gcc` 编译工具
3. Linux系统内核头文件包（或交叉编译内核源码树）
4. `g++` 编译器及 `libstdc++` 库
5. `libpopt-dev` 包

### 1.2 依赖安装命令
```bash
# 1. 安装编译基础工具
sudo apt update && sudo apt install -y build-essential g++

# 2. 安装内核头文件（必须与当前内核版本匹配）
sudo apt install -y linux-headers-$(uname -r)

# 3. 安装libpopt依赖
sudo apt install -y libpopt-dev

# 4. 验证gcc版本（需与内核编译版本一致）
cat /boot/config-$(uname -r) | grep -i "gcc_version"
# 示例输出：CONFIG_GCC_VERSION=120300（表示需安装gcc-12）
# 若版本不匹配：sudo apt install -y gcc-12
```

### 1.3 SDK下载
```bash
# 安装wget（若未安装）
sudo apt install -y wget

# 下载SDK（latest）
wget https://gitee.com/ChengDu-KunHong/KH-UCANFD_Linux_SDK/releases/download/latest/KH-UCANFD_Linux_SDK.zip

# 解压SDK
unzip KH-UCANFD_Linux_SDK.zip

# 进入SDK目录（x.y.z按实际版本替换）
cd KH-UCANFD_Linux_SDK_x.y.z/
```

### 1.4 编译与安装
脚本自动安装：
```bash
./build.sh
```

`build.sh` 常用参数：
```text
./build.sh              安装驱动
./build.sh -c           交叉编译驱动
./build.sh -rules       安装驱动并加载udev命名规则
./build.sh -u           卸载驱动
./build.sh -uninstall   卸载驱动
./build.sh -h           显示帮助信息
```

手动安装：
```bash
sudo make clean
sudo make netdev
sudo make install
```

### 1.5 驱动加载验证
```bash
# 加载kcan驱动模块
sudo modprobe kcan

# 验证驱动是否加载成功（输出含"kcan"即正常）
lsmod | grep kcan

# 重新拔插 CAN FD 设备后，查看CAN接口（输出含can0/can1等）
ip -d link show

# 查看USB设备驱动绑定（输出含 Driver=kcan 即正常）
lsusb -t
```

Ubuntu 依赖：
```bash
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev
```

（可选）封装为 ROS2 后所需依赖：
```bash
sudo apt-get install -y net-tools
sudo apt-get install -y can-utils
sudo apt-get install -y ros-jazzy-can-msgs
sudo apt-get install -y ros-jazzy-ros2-socketcan
```

### 1.6 设置驱动自动保持

```bash
# 安装dkms
sudo apt install dkms
# 重新安装并开启驱动dkms功能
sudo make clean
sudo make netdev
sudo make install_with_dkms
```
## 2. 常用工具命令

### 2.1 lskcan（枚举接口信息）
```bash
lskcan
```
用于列举接口状态及波特率配置。

### 2.2 kcan_monitor（总线监视器）
```bash
kcan_monitor
```

字段说明：
- `dev name`: 设备物理标识与通道号
- `ndev`: Linux SocketCAN 接口名，如 `can0`、`can1`
- `bits`: CAN FD 波特率（仲裁段 + 数据段）
- `bus`: 总线状态（ACTIVE/PASSIVE/BUSOFF）
- `%bus`: 总线负载率
- `%tx fifo`: 发送缓冲区使用率
- `rx_fr/s`: 每秒接收帧数
- `tx_fr/s`: 每秒发送帧数
- `rx`: 累计接收帧数
- `tx`: 累计发送帧数
- `rec/tec`: 接收/发送错误计数
- `err`: 累计错误数

### 2.3 candump 打印接收报文
```bash
candump canx
```
将 `x` 替换为实际通道号。

## 3. IMU安装与串口配置（先改串口号）

### 3.1 修改 IMU 串口号（序列号）
先在 Windows 下使用 `CH34xSerCfg` 工具修改 CH9102/CH34x 设备序列号（Serial String）：
- 选中目标设备（例如 `USB-Enhanced-SERIAL CH9102`）
- 将 `Serial String` 改为唯一值（`0003`）
- 点击“写入配置”
- 重新插拔设备使配置生效

目的：避免多设备时串口号漂移，便于 Linux 用 udev 规则固定成别名。

### 3.2 Linux 下创建 IMU 设备别名
工作区已提供脚本：`imu/wheeltec_udev.sh`

```bash
# 查看设备是否接入（常见 VID:PID 为 1a86:55d4）
lsusb

# 导入 udev 规则
cd imu
sudo chmod +x wheeltec_udev.sh
sudo ./wheeltec_udev.sh

# 重新加载规则
sudo udevadm control --reload-rules
sudo udevadm trigger
```

重新插拔 IMU 后检查是否绑定成功：
```bash
ls -l /dev | grep wheeltec_IMU
```

### 3.3 检查 yesense 配置串口
确认 `imu/yesense_std_ros2/config/yesense_config.yaml` 中串口为：
```yaml
serial_port: "/dev/wheeltec_IMU"
```

### 3.4 （可选）serial 库安装
若编译报缺少 serial 相关依赖，可执行：
```bash
sudo apt update
sudo apt install -y ros-$ROS_DISTRO-serial
```

如仍有问题，可按源码方式安装：
```bash
cd imu
git clone https://github.com/ZhaoXiangBox/serial.git
cd serial
mkdir build && cd build
cmake ..
make
sudo make install
```

## 4. 项目依赖
```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-serial
```

## 5. 编译
在工作区根目录执行：
```bash
colcon build --symlink-install
source install/setup.bash
```

## 6. 一键启动
```bash
ros2 launch rmrobot demo_socketcan.launch.py
```

该 launch 会按顺序执行：
1. 调用 `rmrobot/scripts/setup_can_from_toml.sh`，根据 `Dmmotor.toml` 自动拉起需要的 `canX`
2. 启动 `yesense_std_ros2/yesense_node.launch.py`
3. 启动 `rmrobot` 的 `demo_socketcan` 节点

## 7. launch 可选参数
```bash
ros2 launch rmrobot demo_socketcan.launch.py can_toml:=/绝对路径/Dmmotor.toml can_bitrate:=1000000
```

参数说明：
- `can_toml`: 电机配置文件路径，默认安装后的 `share/rmrobot/config/Dmmotor.toml`（源码内对应 `core/config/Dmmotor.toml`）
- `can_bitrate`: CAN 波特率，默认 `1000000`

## 8. 单独执行 CAN 拉起脚本（可选）
```bash
bash core/scripts/setup_can_from_toml.sh core/config/Dmmotor.toml 1000000
```

## 9. 常见问题
- 报错 `launch file was not found`：
  重新编译并 source：
  ```bash
  colcon build --packages-select rmrobot --symlink-install
  source install/setup.bash
  ```
- 报错权限相关（`ip link` / `CAP_NET_ADMIN`）：
  启动时按提示输入 sudo 密码，或以具备相应权限的方式运行。

## 10. 通道未正常打开（补充）
手动打开命令：
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
将 `x` 替换为通道号；`bitrate` 可按需修改（默认 1M）。

