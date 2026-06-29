# 设计：每通道 `/canX/status_feedback` 全电机状态发布

日期：2026-06-29
分支：ros_driverall

## 目标

每个 CAN 通道单独发布一个 `/canX/status_feedback` 话题，把各电机类型通用的电机
信息（电流、温度、错误码、模式等）通过 ROS topic 对外发出，供上层按需订阅。

现状：温度等信息已在驱动层解析并存入 `Motor_CAN_Receive_Struct`，但节点只用
`sensor_msgs/JointState` 发布 `position/velocity/effort`，温度/错误码/模式无处发出。

## 决策

- 消息结构：自定义 `MotorStatus[]` 数组消息（每通道一条）。
- 字段范围：所有电机类型都会填充的通用字段集，不含型号专有字段。
- 发布时机：复用现有 tick，与 `joint_states` 同频、同 `header.stamp`。

## 新增消息（khcan 包）

```
# msg/MotorStatus.msg
string  name
uint8   motor_id
uint8   mode
uint8   motor_state
uint8   fault_code      # = recv.fault_message
float32 position        # rad,  recv.current_position_f
float32 speed           # rad/s, recv.current_speed_f
float32 torque          # N·m,  recv.current_torque_f
float32 current         # A,    recv.current_iq_f
float32 temperature     # ℃,    recv.current_temp_f
```

```
# msg/MotorStatusArray.msg
std_msgs/Header  header
MotorStatus[]    motors
```

## 节点改动（main/motor_driver_node.cpp）

- include `khcan/msg/motor_status_array.hpp`。
- `ChannelContext` 增加
  `rclcpp::Publisher<khcan::msg::MotorStatusArray>::SharedPtr status_pub;`
- `create_channel_interfaces()` 增加一行建 `prefix + "/status_feedback"` publisher。
- `publish_channel_state()` 在现有 motor 循环里同时组装一条 `MotorStatusArray`
  （motors 顺序与 joint_states 的 name 一致），循环后 `status_pub->publish(...)`。
  header.stamp 与 joint_states 共用同一个 `now()`。

## 构建改动（CMakeLists.txt）

- `rosidl_generate_interfaces(${PROJECT_NAME} ...)` 增加
  `"msg/MotorStatus.msg"` 与 `"msg/MotorStatusArray.msg"`。
- 需要 `std_msgs`（Header 依赖）：`find_package(std_msgs REQUIRED)`，
  `rosidl_generate_interfaces` 加 `DEPENDENCIES std_msgs`，node 的
  `ament_target_dependencies` 加 `std_msgs`。
- package.xml 增加 `<depend>std_msgs</depend>`（若尚无）。
- node 已链接 `cpp_typesupport_target`，无需额外改动。

## 数据来源

全部字段来自现有 `Motor_CAN_Receive_Struct`，各型号解析时已填充。不改动驱动层
（device.cpp / robot.cpp / types.hpp）。

## 不做（YAGNI）

- 型号专有字段（haitai mit_status / fault_source_command / 固件版本）。
- 独立 timer 与可配置发布频率。
- 改动或替换现有 `joint_states` 话题。

## 验证

- `colcon build` 通过，消息生成成功。
- `ros2 topic echo /can<chan>/status_feedback` 能看到与在线电机数量一致的
  `motors` 数组，温度/错误码/模式字段随电机状态变化。
