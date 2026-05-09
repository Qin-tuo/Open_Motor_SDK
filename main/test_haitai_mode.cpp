#include "robot.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

static std::atomic<bool> g_running{true};

static void handle_sigint(int) {
    g_running = false;
}

static bool is_haitai_motor(int api_type) {
    return api_type == 7;
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
        if (is_haitai_motor(robot.global_motors[num1_idx].info.api_type)) {
            return num1_idx;
        }
        std::cerr << "[Warn] Motor num=1 exists but api_type="
                  << robot.global_motors[num1_idx].info.api_type
                  << " is not Haitai." << std::endl;
    }

    for (int i = 0; i < static_cast<int>(robot.global_motors.size()); ++i) {
        if (is_haitai_motor(robot.global_motors[i].info.api_type)) {
            return i;
        }
    }

    return -1;
}

static void prepare_motor_for_haitai(BaseRobot& robot, int target_idx) {
    robot.Disable_N(target_idx);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    robot.ClearError_N(target_idx);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    robot.SetMode_N(target_idx, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    robot.QueryPos_N(target_idx);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::cout << "[Info] Haitai setup: mode=0 (absolute position / 0xC2)" << std::endl;
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
        std::cerr << "[Error] No Haitai motor found. Supported api_type: 7." << std::endl;
        return 1;
    }

    const auto& info = robot.global_motors[target_idx].info;
    std::cout << "[Info] Target motor: idx=" << target_idx
              << ", num=" << info.num
              << ", name=" << info.name
              << ", type=" << info.type
              << ", api_type=" << info.api_type
              << ", canid=" << info.canid << std::endl;

    robot.QueryVersion_N(target_idx);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    const auto& recv = robot.global_motors[target_idx].recv;
    if (recv.version_valid) {
        std::cout << "[Info] Haitai Version: Boot=0x"
                  << std::hex << std::uppercase << std::setw(4) << std::setfill('0') << recv.boot_version
                  << " App=0x" << std::setw(4) << recv.app_version
                  << " HW=0x" << std::setw(4) << recv.hw_version
                  << " CAN Proto=0x" << std::setw(2) << static_cast<int>(recv.can_proto_version)
                  << std::dec << std::nouppercase << std::setfill(' ') << std::endl;
    } else {
        std::cerr << "[Warn] Haitai version query timed out." << std::endl;
    }

    prepare_motor_for_haitai(robot, target_idx);

    robot.Enable_N(target_idx);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    robot.QueryPos_N(target_idx);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    const float center = 0.0f;
    float amp = 0.5f;
    const float freq = 0.3f;

    if (info.pos_max > info.pos_min) {
        const float safe_amp = 0.7f * std::min(std::fabs(center - info.pos_min),
                                               std::fabs(info.pos_max - center));
        if (safe_amp > 0.01f) {
            amp = std::min(amp, safe_amp);
        }
    }

    constexpr float kTwoPi = 6.28318530718f;
    const float omega = kTwoPi * freq;

    std::cout << "Step 1: Moving slowly to zero..." << std::endl;

    robot.QueryPos_N(target_idx);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
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

    std::cout << "Arrived at zero. Starting Haitai absolute-position swing test..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    int i = 0;
    while (g_running.load()) {
        const float t = i * 0.02f;
        const float val = center + amp * std::sin(omega * t);

        MotorCmdVec cmd{};
        cmd.p = val;
        cmd.v = 0.0f;
        cmd.t = 0.0f;

        robot.Move_N(target_idx, cmd);

        if (i % 50 == 0) {
            robot.QueryPos_N(target_idx);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));

            const auto& m = robot.global_motors[target_idx];
            std::cout << "[i=" << i << "] target=" << val
                      << " p=" << m.recv.current_position_f.load()
                      << " v=" << m.recv.current_speed_f.load()
                      << " iq=" << m.recv.current_iq_f.load()
                      << " fault=" << static_cast<int>(m.recv.fault_message)
                      << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        ++i;
    }

    robot.Disable_N(target_idx);
    return 0;
}
