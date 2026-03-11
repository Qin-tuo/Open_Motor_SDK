#include "robot.hpp"
#include <vector>
#include <iostream>
#include <thread>
#include <chrono>
#include <ament_index_cpp/get_package_share_directory.hpp>

int main() {
    // 1. 初始化机器人
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("rmrobot");
    std::string toml_path = package_share_directory + "/config/motor.toml";
    const std::string config_path = toml_path;
    BaseRobot robot(config_path);

    robot.DisableAll(); // 启动时先关闭所有电机，确保安全
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    //robot.SetZero_All();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // 2. 清除错误 (建议启动时执行一次)
    robot.ClearError_All();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // 【选项】是否使能电机？
    robot.EnableAll(); 

    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // ================= 配置区域 =================
    double target_hz = 50.0;  // <--- 修改这里来调整频率 (例如 1.0, 10.0, 50.0)
    // ===========================================

    // 计算周期 (微秒)，例如 10Hz -> 100000us
    auto period_duration = std::chrono::microseconds(static_cast<int64_t>(1000000.0 / target_hz));

    std::cout << "=== Robot Position Monitor (" << target_hz << "Hz) ===" << std::endl;
    std::cout << " loop... Press Ctrl+C to exit." << std::endl;

    while(true) {
        // 记录循环开始时间
        auto start_time = std::chrono::steady_clock::now();

        // [关键步骤] 发送查询指令
        robot.QueryPos_ALL();

        // 等待总线回传数据 (给一点点固定延时让CAN线程处理接收，防止 Query 和 Print 挨太紧)
        // 注意：如果频率很高(如 >100Hz)，这个固定延时可能需要减小
        std::this_thread::sleep_for(std::chrono::milliseconds(2));

        // [打印状态]
        robot.PrintStatus();

        // --- 智能频率控制 ---
        auto end_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        
        // 计算还需要休眠多久才能达到目标频率
        if (elapsed < period_duration) {
            std::this_thread::sleep_for(period_duration - elapsed);
        } else {
            // 如果执行时间已经超过了周期，就不休眠了，直接进下一次（防止阻塞）
            // std::cerr << "[Warning] Loop too slow for target Hz!" << std::endl;
        }
    }

    return 0;
}
