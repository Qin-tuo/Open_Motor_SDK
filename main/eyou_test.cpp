#include "robot.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {

std::atomic<bool> g_running{true};

void handleSignal(int) {
    g_running = false;
}

float clampValue(float value, float lower, float upper) {
    return std::max(lower, std::min(value, upper));
}

float resolvePosMin(const Motor_CAN_Info_Struct& info) {
    return (info.pos_max > info.pos_min) ? info.pos_min : info.p_min;
}

float resolvePosMax(const Motor_CAN_Info_Struct& info) {
    return (info.pos_max > info.pos_min) ? info.pos_max : info.p_max;
}

} // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    const std::string config_path = (argc > 1) ? argv[1] : "config/motor.toml";
    BaseRobot robot(config_path);

    if (robot.global_motors.empty()) {
        std::cerr << "[Error] No motors loaded from config: " << config_path << std::endl;
        return 1;
    }

    bool has_type5 = false;
    for (const auto& motor : robot.global_motors) {
        if (motor.info.api_type == 5) {
            has_type5 = true;
            break;
        }
    }

    if (!has_type5) {
        std::cerr << "[Error] No TYPE5(EYOU) motors found in config: " << config_path << std::endl;
        return 1;
    }

    std::cout << "[Info] Using config: " << config_path << std::endl;
    std::cout << "[Info] Preparing EYOU TYPE5 position reciprocating test..." << std::endl;

    robot.ClearError_All();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    robot.SetModeAll_Type5(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    robot.EnableAll();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    for (int i = 0; i < 3; ++i) {
        robot.QueryPos_ALL();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    const std::vector<float> start_positions = robot.GetPosAll();
    const size_t motor_count = robot.global_motors.size();
    std::vector<float> centers(motor_count, 0.0f);
    std::vector<float> amplitudes(motor_count, 0.0f);
    std::vector<float> profile_velocities(motor_count, 0.0f);
    std::vector<float> torque_limits(motor_count, 0.0f);
    std::vector<MotorCmdVec> cmds(motor_count);

    for (size_t index = 0; index < motor_count; ++index) {
        const auto& info = robot.global_motors[index].info;
        const float pos_min = resolvePosMin(info);
        const float pos_max = resolvePosMax(info);
        const float current_pos = (index < start_positions.size()) ? start_positions[index] : 0.0f;

        if (info.api_type != 5) {
            centers[index] = current_pos;
            amplitudes[index] = 0.0f;
            cmds[index].p = current_pos;
            cmds[index].v = 0.0f;
            cmds[index].t = 0.0f;
            continue;
        }

        float center = current_pos;
        if (pos_max > pos_min) {
            center = clampValue(current_pos, pos_min, pos_max);
        }

        float amplitude = 0.3f;
        if (pos_max > pos_min) {
            const float free_range = pos_max - pos_min;
            amplitude = std::min(0.35f * free_range, 0.5f);
            amplitude = std::max(amplitude, 0.1f);
            amplitude = std::min(amplitude, 0.5f * free_range);

            const float room_positive = pos_max - center;
            const float room_negative = center - pos_min;
            amplitude = std::min(amplitude, std::max(0.0f, std::min(room_positive, room_negative)));
        }

        const float rated_torque = (info.type5_rated_torque_nm > 0.0f) ? info.type5_rated_torque_nm : 3.0f;
        float profile_velocity = info.type5_profile_velocity;
        if (profile_velocity <= 0.0f) {
            profile_velocity = 2.0f;
        }

        centers[index] = center;
        amplitudes[index] = amplitude;
        profile_velocities[index] = std::min(profile_velocity, 2.5f);
        torque_limits[index] = std::max(1.0f, rated_torque * 0.5f);

        cmds[index].p = center;
        cmds[index].v = profile_velocities[index];
        cmds[index].t = torque_limits[index];
    }

    std::cout << "[Info] Moving TYPE5 motors to start centers..." << std::endl;
    for (int step = 0; step <= 100 && g_running; ++step) {
        const float blend = static_cast<float>(step) / 100.0f;
        for (size_t index = 0; index < motor_count; ++index) {
            const auto& info = robot.global_motors[index].info;
            const float initial_pos = (index < start_positions.size()) ? start_positions[index] : 0.0f;
            if (info.api_type != 5) {
                cmds[index].p = initial_pos;
                cmds[index].v = 0.0f;
                cmds[index].t = 0.0f;
                continue;
            }

            cmds[index].p = initial_pos + (centers[index] - initial_pos) * blend;
            cmds[index].v = profile_velocities[index];
            cmds[index].t = torque_limits[index];
        }

        robot.Move(cmds);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    std::cout << "[Info] Start reciprocating motion. Press Ctrl+C to stop." << std::endl;
    int iteration = 0;
    while (g_running) {
        const float phase = iteration * 0.05f;
        const float ratio = std::sin(phase);

        for (size_t index = 0; index < motor_count; ++index) {
            const auto& info = robot.global_motors[index].info;
            if (info.api_type != 5) {
                continue;
            }

            cmds[index].p = centers[index] + amplitudes[index] * ratio;
            cmds[index].v = profile_velocities[index];
            cmds[index].t = torque_limits[index];
        }

        robot.Move(cmds);

        if ((iteration % 50) == 0) {
            robot.QueryPos_ALL();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            robot.PrintStatus();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        ++iteration;
    }

    std::cout << "\n[Info] Stopping test and disabling motors..." << std::endl;
    robot.DisableAll();
    return 0;
}
