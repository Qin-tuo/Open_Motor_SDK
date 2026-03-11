# khcan 使用说明

## 简介
`khcan` 是一个基于 ROS 2 `ament_cmake` 的 SocketCAN 电机控制包。
当前仓库中实际可用的入口程序只有 `show_status`，它会：
- 从 `config/motor.toml` 读取电机与 CAN 通道配置
- 初始化机器人对象
- 关闭电机、清错、重新使能
- 周期性查询所有电机位置并在终端打印状态

## 当前目录结构
```text
khcan/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── motor.toml
├── include/
├── main/
│   └── show_status.cpp
└── src/
```

## 代码框架对应关系
- `main/show_status.cpp`：程序入口，执行初始化、清错、使能和状态打印循环
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

## 配置说明
程序运行时会通过包安装目录读取：
```text
share/khcan/config/motor.toml
```
源码中的原始配置文件位于：
```text
config/motor.toml
```

`motor.toml` 中每个电机至少包含这些关键信息：
- `num`：逻辑编号
- `name`：电机名称
- `type`：电机型号
- `api_type`：控制协议类型
- `chan`：CAN 通道号，会映射为 `can<chan>`
- `canid`：CAN ID

例如当 `chan = 1` 时，程序会尝试打开 `can1`。

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
  ```bash
  source install/setup.bash
  ```
- 修改了 `config/motor.toml` 但运行结果没变化：
  若未使用 `--symlink-install`，请重新编译并重新 `source`。
- 报错 `canX` 打不开：
  先确认 CAN 驱动、接口名、波特率和权限是否正确。
