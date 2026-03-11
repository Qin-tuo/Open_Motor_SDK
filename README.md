# OS 兼容性说明
本代码默认配置为 **Arm 架构**（如 Raspberry Pi, Jetson 等）。
- **Intel (x86_64) 架构运行时**：需要替换 `lib` 目录下的 `.so` 文件。
- 对应的 Intel 版本库文件可以在 `/lib_sdk` 目录中找到，请手动覆盖。

---
# 前期准备
本程序需要与肥猫机器人公司USB2CAN模块配合使用，请准备好模块与模块说明书、模块SDK，并按照说明书使用install.sh文件安装USB2CAN规则文件，或手动安装规则文件，安装方法：

```cpp
// 进入项目目录下的can文件夹
cd USB2CAN-Demo-Lingzu/can

// 复制规则文件usb_can.rules 到/etc/udev/rules.d/
sudo cp usb_can.rules /etc/udev/rules.d/


// 运行下面的命令，使udev规则生效
sudo udevadm trigger
```

# 接口使用说明

可直接 
mkdir build
cmake ..
make -j2
会得到两个可执行文件：lingzu_robot   show_status
./show_status会返回所有电机当前的位置，50hz，同时所有电机不会移动。
./lingzu_robot 会执行测试程序，测试所有电机目前运动是否正常。 
（注：可执行文件需要 chmod +x ）

## 1. 标准调用流程 (Pipeline)
使用 `BaseRobot` 进行开发的标准流程如下：

1.  **初始化**：通过配置文件路径（如 `.csv`）创建 `BaseRobot` 对象。
2.  **清除故障**：调用 `ClearError_All()` 确保所有电机无故障状态。
3.  **使能电机（可选）**：
    * **主动控制模式**：调用 `EnableAll()` 上电，电机将进入硬连接状态。
    * **被动监控模式**：**不要**调用 `EnableAll()`。此时电机处于无力状态，可以用手掰动，配合 `QueryPos` 接口可作为示教器使用。
4.  **设置模式**：
    * 调用 `SetModeAll_Type1(...)` 设置灵足 (LimX) 电机模式。
    * 调用 `SetModeAll_Type2(...)` 设置 LK (宇树旧版) 电机模式。
    * *注意：不同品牌电机的模式定义不同，请参考下文[控制模式定义]章节。*
5.  **循环控制**：构建 `MotorCmdVec` 向量，调用 `robot.Move(cmds)` 下发指令。
6.  **结束**：调用 `DisableAll()` 下电。

示例代码入口可参考：`main_exec.cpp`

---

## 2. 关键数据结构

### `MotorCmdVec`
用于控制单个电机的指令结构体，包含以下三个成员。根据设置的模式不同，需要填充对应的成员：

```cpp
struct MotorCmdVec {
    float p; // Position (位置，单位：rad)
    float v; // Speed    (速度，单位：rad/s)
    float t; // Torque   (力矩，单位：Nm)
};
```

* **控制速度时**：请设置 `.v` (Type 1 和 Type 2 均适用)。
* **控制位置时**：请设置 `.p` (Type 1 和 Type 2 均适用)。
* **控制力矩时**：请设置 `.t` (Type 1 和 Type 2 均适用)。

---

## 3. 控制模式定义 (Mode Map)

**⚠️ 警告：Type 1 (灵足) 和 Type 2 (LK) 的模式 ID 定义不一致，请务必区分设置！**

### 【Type 1: LimX / 灵足协议】
| 模式 ID | 模式名称 | 说明 |
| :--- | :--- | :--- |
| **0** | **运控模式 (MIT)** | 混合控制 (P, V, T, Kp, Kd) |
| **1** | **位置模式** | 纯位置闭环 |
| **2** | **速度模式** | 纯速度闭环 |
| **3** | **电流/力矩模式** | 直接力矩控制 |

### 【Type 2: LK / 宇树旧协议】
| 模式 ID | 模式名称 | 对应指令 | 说明 |
| :--- | :--- | :--- | :--- |
| **1** | **力矩/混合模式** | `0xA1` | 对应 Type1 的模式 0 (MIT) |
| **2** | **位置模式** | `0xA3` | 对应 Type1 的模式 1 |
| **3** | **速度模式** | `0xA2` | 对应 Type1 的模式 2 |
| *其他* | *读取状态* | `0x9C` | 仅读取，不控制 |

> **特别注意**：
> * Type 1 的 **MIT/运控模式** 是 `0`。
> * Type 2 的 **MIT/力矩模式** 是 `1`。

---

## 4. 核心 API 速查

### 初始化与配置
```cpp
// 构造函数：传入配置文件路径
BaseRobot robot("config_file.csv");

// 清除所有电机错误（建议启动时优先调用）
robot.ClearError_All();

// 设置所有 Type 1 电机的模式
robot.SetModeAll_Type1(1); // 例如：设置为位置模式

// 设置所有 Type 2 电机的模式
robot.SetModeAll_Type2(3); // 例如：设置为速度模式
```

### 启停控制
```cpp
// 使能所有电机
robot.EnableAll();

// 失能所有电机（停止输出）
robot.DisableAll();
```

### 运动控制
```cpp
// 准备指令向量（大小需与电机总数一致）
std::vector<MotorCmdVec> cmds(motor_count);

// 填充指令 (假设电机 0 是 Type 1 位置模式，电机 1 是 Type 2 速度模式)
cmds[0].p = 3.14; // Set Position
cmds[1].v = 5.0;  // Set Speed

// 下发指令
robot.Move(cmds);
```

### 状态反馈
```cpp
// 打印所有电机当前状态到控制台
robot.PrintStatus();

// 获取所有电机当前位置 (返回 vector<float>)
auto positions = robot.GetPosAll();

// 获取所有电机当前位置 (返回 vector<float>)
auto positions = robot.GetPosAll();

```