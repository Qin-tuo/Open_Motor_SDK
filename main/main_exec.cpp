#include "robot.hpp"
#include <cmath>
#include <vector>
#include <thread>
#include <chrono>
#include <algorithm>
#include <iostream>

// 辅助函数：限制值在范围内（可选）
float clip(float n, float lower, float upper) {
    return std::max(lower, std::min(n, upper));
}

int main() {
    std::string config_path = "config/motor.toml";
    BaseRobot robot(config_path);
    
    // 初始化
    robot.ClearError_All();
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); 
    robot.EnableAll();
    robot.SetModeAll_Type1(1); // 位置模式
    robot.SetModeAll_Type2(2); // 速度模式
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); 

    // 获取电机数量
    size_t motor_count = robot.global_motors.size();
    std::vector<MotorCmdVec> cmds(motor_count);

    // ==========================================
    // 1. 手动定义两个位置向量 (请在此处修改数值)
    // ==========================================
    // 默认初始化为全0

// Index 0-2: 左腿, Index 3: 左脚(速度0), Index 4-6: 右腿, Index 7: 右脚(速度0), Index 8-19: 身体(-0.3)
std::vector<float> pose_A = {
    0.0262775f,  0.0646381f,  -0.228058f,  // 0, 1, 2
    0.0f,                                  // 3 (L_FOOT_P 速度初始为0)
    -0.0297298f, 0.0285797f,  0.0f,   // 4, 5, 6
    0.0f,                                  // 7 (R_FOOT_P 速度初始为0)
    -0.3f, -0.3f, -0.3f, 0.0f,            // 8, 9, 10, 11
    -0.3f, -0.3f, -0.3f, 0.0f,            // 12, 13, 14, 15
    -0.3f, -0.3f, -0.3f, -0.3f             // 16, 17, 18, 19
};

// 赋值 Pose B (20维)
// Index 0-2: 左腿, Index 3: 左脚(速度10), Index 4-6: 右腿, Index 7: 右脚(速度10), Index 8-19: 身体(+0.3)
std::vector<float> pose_B = {
    -0.140594f,  0.411808f,   -1.22506f,   // 0, 1, 2
    10.0f,                                 // 3 (L_FOOT_P 目标速度)
    0.14136f,    -0.449018f,  1.0f,   // 4, 5, 6
    10.0f,                                 // 7 (R_FOOT_P 目标速度)
    0.3f, 0.3f, 0.3f, 0.3f,                // 8, 9, 10, 11
    0.3f, 0.3f, 0.3f, 0.3f,                // 12, 13, 14, 15
    0.3f, 0.3f, 0.3f, 0.3f                 // 16, 17, 18, 19
};

    // (可选) 如果你只想测试某几个电机，其他保持0即可
    // ==========================================


    // ==========================================
    // 2. 阶段一：启动缓冲 (Current -> Pose A)
    // ==========================================
    std::cout << "Step 1: Moving slowly to Pose A..." << std::endl;
    auto start_pos = robot.GetPosAll();
    int warmup_steps = 100; // 2秒

    for(int i = 0; i <= warmup_steps; ++i) {
        // 插值因子 k: 0.0 -> 1.0
        float k = (float)i / (float)warmup_steps;

        for(size_t m = 0; m < motor_count; ++m) {
            // 线性插值: 当前 = 起点 + (终点 - 起点) * k
            // 注意保护：如果读取的 start_pos 数量不够，默认用 0
            float s_p = (m < start_pos.size()) ? start_pos[m] : 0.0f;
            float target = s_p + (pose_A[m] - s_p) * k;
            
            cmds[m].p = target;
            cmds[m].v = 0.0f;
            cmds[m].t = 0.0f;
            if(m == 3 || m == 7)
            {
                cmds[m].v = target;
                cmds[m].p = 0;
            };

        }
        robot.Move(cmds);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    std::cout << "Arrived at Pose A. Starting Swing Loop..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 稍作停顿

    // ==========================================
    // 3. 阶段二：在 A 和 B 之间往复摆动
    // ==========================================
    // 使用 Cosine 插值实现平滑摆动
    // ratio 变化规律： 0 (Pose A) -> 1 (Pose B) -> 0 (Pose A) ...
    
    for(int i = 0; i < 5000; i++) {
        float t = i * 0.05f; // 时间变量，调小这个系数可以让动作变慢
        
        // 核心算法：0.5 * (1 - cos(t)) 
        // 当 t=0, cos=1, ratio=0 (完全在 A)
        // 当 t=pi, cos=-1, ratio=1 (完全在 B)
        float ratio = 0.5f * (1.0f - std::cos(t));

        for(size_t m = 0; m < motor_count; ++m) {
            // 在 Pose A 和 Pose B 之间混合
            float val = pose_A[m] * (1.0f - ratio) + pose_B[m] * ratio;

            cmds[m].p = val;
            cmds[m].v = 0.0f; // 纯位置控制建议速度设为0，除非做前馈
            cmds[m].t = 0.0f;
            if(m == 3 || m == 7)
            {
                cmds[m].v = val;
            cmds[m].p = 0;
            };
        }

        robot.Move(cmds);

        if(i % 50 == 0) robot.PrintStatus();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    return 0;
}
