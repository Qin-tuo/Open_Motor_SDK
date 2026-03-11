# motor_driver

多协议电机 CAN 驱动与示例程序，统一通过 `BaseRobot` 控制。

## 1. 环境与依赖

- 系统：Linux
- 编译：CMake + C++17
- 设备：USB2CAN（`usb_can` SDK）
- 依赖：ROS2 `ament_cmake` 体系（见 `CMakeLists.txt`）

## 2. 编译

```bash
mkdir -p build
cmake -S . -B build
cmake --build build -j4
```

编译后可执行文件在 `build/` 下：

- `lingzu_robot`
- `show_status`
- `eyou_test`
- `encos_test`

## 3. 可执行程序

### `show_status`

- 作用：循环查询并打印状态（默认 50Hz）。
- 默认配置：`config/motor.toml`
- 注意：该程序只查询状态，不会下发运动轨迹。

```bash
./build/show_status
```

### `lingzu_robot`

- 作用：通用示例入口（`main/main_exec.cpp`）。
- 默认配置：`config/motor.toml`

```bash
./build/lingzu_robot
```

### `eyou_test`

- 作用：Type5（PP11）往复测试。
- 默认配置：`config/motor.toml`
- 支持传参覆盖配置路径。

```bash
./build/eyou_test
./build/eyou_test config/motor.toml
```

### `encos_test`

- 作用：Type6（ENCOS）MIT 往复测试。
- 支持传参指定配置路径（建议显式传入）。

```bash
./build/encos_test config/motor.toml
```

## 4. 配置文件（TOML）

程序读取 `motors` 数组，每个电机一条记录，例如：

```toml
motors = [
  { num = 1, name = "ENCOS_JOINT", type = "EC-A4310-P2-36", api_type = 6, device = "USB2CAN2", chan = 1, canid = 1,
    p_min = -12.5, p_max = 12.5, v_min = -18.0, v_max = 18.0,
    kp_min = 0.0, kp_max = 500.0, kd_min = 0.0, kd_max = 5.0,
    t_min = -30.0, t_max = 30.0, kp_in_use = 15.0, kd_in_use = 0.5,
    pos_min = -12.5, pos_max = 12.5 },
]
```

关键字段说明：

- `num`：电机编号（业务编号）
- `name`：电机名称
- `type`：型号字符串（当前在 `api_type=5` 时用于 PP11 默认参数）
- `api_type`：协议类型
- `device`：USB2CAN 设备名（实际打开时会拼 `/dev/<device>`）
- `chan`：CAN 通道号（常见为 `0` 或 `1`，按你的 USB2CAN 定义）
- `canid`：电机 CAN ID
- `p_min/p_max`、`v_min/v_max`、`kp_min/kp_max`、`kd_min/kd_max`、`t_min/t_max`：协议映射范围
- `kp_in_use/kd_in_use`：当前使用的增益（示例程序会读取）
- `pos_min/pos_max`：上层位置限幅，若无效则回退到 `p_min/p_max`

## 5. 协议类型（`api_type`）

- `1`：Type1（灵足/LimX）
- `2`：Type2（LK）
- `3`：Type3（DM）
- `4`：Type4（RoboMaster C620）
- `5`：Type5（EYOU PP11）
- `6`：Type6（ENCOS）

## 6. 统一模式约定

上层统一使用 `SetMode...` 传入：

- `0`：MIT/混合控制
- `1`：位置模式
- `2`：速度模式
- `3`：力矩/电流模式

说明：

- Type1/Type3/Type5/Type6 按上述语义映射。
- Type2 内部做了协议适配（`0/3` 走混合，`1` 位置，`2` 速度）。
- Type4 仅支持电流环控制，`mode` 仅作兼容保留。

## 7. Type5（PP11）说明

- 当前默认参数逻辑仅针对 `PP11`。
- `type` 字段建议填写 `PP11`。
- 可用以下接口写 PI 参数：
  - `SetType5CurrentPI_N`
  - `SetType5SpeedPI_N`
  - `SetType5PositionPI_N`
  - `SaveType5Params_N`

## 8. 常见问题

### Q1：`show_status` 看不到返回值

先确认以下几点：

1. 程序启动日志中没有 `Failed to open CAN device`。
2. `device/chan/canid` 与硬件一致（尤其是 `chan`，很多设备是 `0` 起始）。
3. 电机与总线为经典 CAN 1Mbps（非 CAN FD）。
4. 终端电阻、供电、CANH/CANL 接线正确。
5. 电机 ID 与配置一致（ENCOS 出厂常见 ID 为 `1`，若改过以实际为准）。

### Q2：为什么 `show_status` 不动电机

`show_status` 只做查询和打印，不发送轨迹指令。要做运动测试请使用 `lingzu_robot`、`eyou_test` 或 `encos_test`。
