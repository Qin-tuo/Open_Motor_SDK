#include "robot.hpp"

#include <atomic>
#include <csignal>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>

static std::atomic<bool> g_running{true};

static void handle_sigint(int) {
    g_running = false;
}

int main() {
    std::signal(SIGINT, handle_sigint);

    const std::string config_path = "config/motor.toml";
    BaseRobot robot(config_path);

    // Find only the motor whose business id (num) == 1.
    int target_idx = -1;
    for (int i = 0; i < static_cast<int>(robot.global_motors.size()); ++i) {
        if (robot.global_motors[i].info.num == 1) {
            target_idx = i;
            break;
        }
    }

    if (target_idx < 0) {
        std::cerr << "[Error] No motor with num=1 found in config." << std::endl;
        return 1;
    }

    const auto& info = robot.global_motors[target_idx].info;
    std::cout << "[Info] Target motor: idx=" << target_idx
              << ", num=" << info.num
              << ", name=" << info.name
              << ", api_type=" << info.api_type
              << ", canid=" << info.canid << std::endl;

    // Init only this motor.
    // For api_type=5, ClearError maps to stop in current driver, so avoid
    // sending stop right before control test.
    if (info.api_type == 5) {
        robot.ClearError_N(target_idx);
        std::this_thread::sleep_for(std::chrono::milliseconds(120));
    } else {
        robot.Disable_N(target_idx);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        robot.ClearError_N(target_idx);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // Keep mode selection compatible with existing protocols.
    if (info.api_type == 1) {
        robot.SetMode_N(target_idx, 1); // position mode
    } else if (info.api_type == 2) {
        robot.SetMode_N(target_idx, 2); // speed mode
    } else if (info.api_type == 5) {
        robot.SetMode_N(target_idx, 0); // MIT mode
        float kp = info.kp_in_use;
        float kd = info.kd_in_use;
        if (kp == 0.0f && kd == 0.0f) {
            kp = 20.0f;
            kd = 0.8f;
        }
        robot.SetKpd_N(kp, kd, target_idx);
        std::cout << "[Info] MIT gains: kp=" << kp << ", kd=" << kd << std::endl;
    }

    robot.Enable_N(target_idx);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Single-axis oscillation around zero.
    const float center = 0.0f;
    const float amp = 0.60f;     // rad
    const float freq = 0.40f;    // Hz
    constexpr float kTwoPi = 6.28318530718f;
    const float omega = kTwoPi * freq;

    std::cout << "Step 1: Moving slowly to zero..." << std::endl;

    const float start_pos = 0.0f;

    const int warmup_steps = 100; // 2 s at 20 ms
    for (int i = 0; i <= warmup_steps; ++i) {
        const float k = static_cast<float>(i) / static_cast<float>(warmup_steps);
        const float target = start_pos + (center - start_pos) * k;

        MotorCmdVec cmd {};
        cmd.p = target;
        cmd.v = 0.0f;
        cmd.t = 0.0f;

        // If this motor uses speed mode, map target to speed command.
        if (info.api_type == 2) {
            cmd.v = target;
            cmd.p = 0.0f;
        } else if (info.api_type == 5) {
            cmd.v = 0.0f;
        }

        robot.Move_N(target_idx, cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    std::cout << "Arrived at zero. Starting swing around zero..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    int i = 0;
    while (g_running.load()) {
        const float t = i * 0.02f; // 20 ms loop
        const float val = center + amp * std::sin(omega * t);
        const float vel_ff = amp * omega * std::cos(omega * t);

        MotorCmdVec cmd {};
        cmd.p = val;
        cmd.v = 0.0f;
        cmd.t = 0.0f;

        if (info.api_type == 2) {
            cmd.v = val;
            cmd.p = 0.0f;
        } else if (info.api_type == 5) {
            cmd.v = vel_ff;
        }

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
