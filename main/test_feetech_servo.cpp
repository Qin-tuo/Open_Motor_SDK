#include "robot.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>

static std::atomic<bool> g_running{true};

static void handle_sigint(int) {
    g_running = false;
}

static bool is_feetech_servo(int api_type) {
    return api_type == 4;
}

static int select_target_motor_index(const BaseRobot& robot) {
    int num1_idx = -1;
    for (int i = 0; i < static_cast<int>(robot.global_motors.size()); ++i) {
        if (robot.global_motors[i].info.num == 1) {
            num1_idx = i;
            break;
        }
    }

    if (num1_idx >= 0) {
        if (is_feetech_servo(robot.global_motors[num1_idx].info.api_type)) {
            return num1_idx;
        }
        std::cerr << "[Warn] Motor num=1 exists but api_type="
                  << robot.global_motors[num1_idx].info.api_type
                  << " is not Feetech Type4." << std::endl;
    }

    for (int i = 0; i < static_cast<int>(robot.global_motors.size()); ++i) {
        if (is_feetech_servo(robot.global_motors[i].info.api_type)) {
            return i;
        }
    }

    return -1;
}

int main() {
    std::signal(SIGINT, handle_sigint);

    const std::string config_path =
        ament_index_cpp::get_package_share_directory("khcan") + "/config/motor.toml";
    BaseRobot robot(config_path);

    if (robot.global_motors.empty()) {
        std::cerr << "[Error] No motor configured in " << config_path << std::endl;
        return 1;
    }

    const int target_idx = select_target_motor_index(robot);
    if (target_idx < 0) {
        std::cerr << "[Error] No Feetech servo found. Supported api_type: 4." << std::endl;
        return 1;
    }

    const auto& info = robot.global_motors[target_idx].info;
    std::cout << "[Info] Target servo: idx=" << target_idx
              << ", num=" << info.num
              << ", name=" << info.name
              << ", type=" << info.type
              << ", api_type=" << info.api_type
              << ", port=" << info.port
              << ", baud=" << info.baud
              << ", id=" << info.canid << std::endl;

    robot.ClearError_N(target_idx);
    robot.Enable_N(target_idx);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    robot.QueryPos_N(target_idx);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    const float travel_min = (info.pos_max > info.pos_min) ? info.pos_min : info.p_min;
    const float travel_max = (info.pos_max > info.pos_min) ? info.pos_max : info.p_max;
    const float center = 0.5f * (travel_min + travel_max);
    float amp = 0.25f;
    if (travel_max > travel_min) {
        amp = std::min(amp, 0.2f * (travel_max - travel_min));
    }

    constexpr float kTwoPi = 6.28318530718f;
    const float freq = 0.2f;
    const float omega = kTwoPi * freq;

    std::cout << "Step 1: Moving slowly to center..." << std::endl;
    const float start_pos = robot.global_motors[target_idx].recv.current_position_f.load();
    const int warmup_steps = 100;
    for (int i = 0; i <= warmup_steps && g_running.load(); ++i) {
        const float k = static_cast<float>(i) / static_cast<float>(warmup_steps);
        const float target = start_pos + (center - start_pos) * k;

        MotorCmdVec cmd{};
        cmd.p = target;
        cmd.v = 1.0f;
        cmd.t = 0.0f;
        robot.Move_N(target_idx, cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    std::cout << "Arrived at center. Starting Feetech small swing test..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    int i = 0;
    while (g_running.load()) {
        const float t = i * 0.02f;
        const float target = center + amp * std::sin(omega * t);

        MotorCmdVec cmd{};
        cmd.p = target;
        cmd.v = 1.5f;
        cmd.t = 0.0f;
        robot.Move_N(target_idx, cmd);

        if (i % 25 == 0) {
            robot.QueryPos_N(target_idx);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));

            const auto& m = robot.global_motors[target_idx];
            std::cout << "[i=" << i << "] target=" << target
                      << " p=" << m.recv.current_position_f.load()
                      << " v=" << m.recv.current_speed_f.load()
                      << " load=" << m.recv.current_torque_f.load()
                      << " temp=" << m.recv.current_temp_f.load()
                      << " fault=" << static_cast<int>(m.recv.fault_message)
                      << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        ++i;
    }

    robot.Disable_N(target_idx);
    return 0;
}
