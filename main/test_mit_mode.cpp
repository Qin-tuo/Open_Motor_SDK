#include "robot.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

static std::atomic<bool> g_running{true};

static void handle_sigint(int) {
    g_running = false;
}

static bool supports_mit_mode(int api_type) {
    return api_type == 1 || api_type == 2 || api_type == 3 || api_type == 5 || api_type == 8;
}

static int mit_mode_for_api_type(int api_type) {
    switch (api_type) {
        case 1: return 0; // Type1 MIT
        case 2: return 1; // Type2 MIT
        case 3: return 0; // Type3 MIT
        case 5: return 0; // Type5 MIT
        case 8: return 0; // Type8 ENCOS MIT
        default: return -1;
    }
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
        if (supports_mit_mode(robot.global_motors[num1_idx].info.api_type)) {
            return num1_idx;
        }
        std::cerr << "[Warn] Motor num=1 exists but api_type="
                  << robot.global_motors[num1_idx].info.api_type
                  << " does not support MIT test in this tool." << std::endl;
    }

    for (int i = 0; i < static_cast<int>(robot.global_motors.size()); ++i) {
        if (supports_mit_mode(robot.global_motors[i].info.api_type)) {
            return i;
        }
    }

    return -1;
}

static void prepare_motor_for_mit(BaseRobot& robot, int target_idx) {
    const auto& info = robot.global_motors[target_idx].info;

    // Type5 clear-error internally maps to reset flow, avoid sending stop first.
    if (info.api_type == 5) {
        robot.ClearError_N(target_idx);
        std::this_thread::sleep_for(std::chrono::milliseconds(120));
    } else {
        robot.Disable_N(target_idx);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        robot.ClearError_N(target_idx);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    const int mit_mode = mit_mode_for_api_type(info.api_type);
    if (mit_mode < 0) {
        return;
    }
    robot.SetMode_N(target_idx, mit_mode);

    float kp = info.kp_in_use;
    float kd = info.kd_in_use;
    if (kp == 0.0f && kd == 0.0f) {
        kp = 10.0f;
        kd = 0.8f;
    }
    robot.SetKpd_N(kp, kd, target_idx);

    std::cout << "[Info] MIT setup: mode=" << mit_mode
              << ", kp=" << kp
              << ", kd=" << kd << std::endl;
}

int main() {
    std::signal(SIGINT, handle_sigint);

    const std::string config_path = "config/motor.toml";
    BaseRobot robot(config_path);

    if (robot.global_motors.empty()) {
        std::cerr << "[Error] No motor configured in " << config_path << std::endl;
        return 1;
    }

    const int target_idx = select_target_motor_index(robot);
    if (target_idx < 0) {
        std::cerr << "[Error] No MIT-capable motor found. Supported api_type: 1/2/3/5/8." << std::endl;
        return 1;
    }

    const auto& info = robot.global_motors[target_idx].info;
    std::cout << "[Info] Target motor: idx=" << target_idx
              << ", num=" << info.num
              << ", name=" << info.name
              << ", type=" << info.type
              << ", api_type=" << info.api_type
              << ", canid=" << info.canid << std::endl;

    prepare_motor_for_mit(robot, target_idx);

    robot.Enable_N(target_idx);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    const float center = 0.0f;
    float amp = 0.60f;      // rad
    const float freq = 2.0f; // Hz

    if (info.pos_max > info.pos_min) {
        const float safe_amp = 0.7f * std::min(std::fabs(center - info.pos_min), std::fabs(info.pos_max - center));
        if (safe_amp > 0.01f) {
            amp = std::min(amp, safe_amp);
        }
    }

    constexpr float kTwoPi = 6.28318530718f;
    const float omega = kTwoPi * freq;

    std::cout << "Step 1: Moving slowly to zero..." << std::endl;

    const float start_pos = robot.global_motors[target_idx].recv.current_position_f.load();
    const int warmup_steps = 100; // 2 s at 20 ms
    for (int i = 0; i <= warmup_steps; ++i) {
        const float k = static_cast<float>(i) / static_cast<float>(warmup_steps);
        const float target = start_pos + (center - start_pos) * k;

        MotorCmdVec cmd{};
        cmd.p = target;
        cmd.v = 0.0f;
        cmd.t = 0.0f;

        robot.Move_N(target_idx, cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    std::cout << "Arrived at zero. Starting MIT swing test..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    int i = 0;
    while (g_running.load()) {
        const float t = i * 0.02f; // 20 ms loop
        const float val = center + amp * std::sin(omega * t);
        const float vel_ff = amp * omega * std::cos(omega * t);

        MotorCmdVec cmd{};
        cmd.p = val;
        cmd.v = vel_ff;
        cmd.t = 0.0f;

        robot.Move_N(target_idx, cmd);

        if (i % 50 == 0) {
            const auto& m = robot.global_motors[target_idx];
            std::cout << "[i=" << i << "] target=" << val
                      << " p=" << m.recv.current_position_f.load()
                      << " v=" << m.recv.current_speed_f.load()
                      << " t=" << m.recv.current_torque_f.load() << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        ++i;
    }

    robot.Disable_N(target_idx);
    return 0;
}
