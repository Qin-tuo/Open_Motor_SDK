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

float resolveTorqueAbsLimit(const Motor_CAN_Info_Struct& info) {
    return std::max(std::fabs(info.t_min), std::fabs(info.t_max));
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

    std::vector<int> type6_indices;
    for (int i = 0; i < static_cast<int>(robot.global_motors.size()); ++i) {
        if (robot.global_motors[i].info.api_type == 6) {
            type6_indices.push_back(i);
        }
    }

    if (type6_indices.empty()) {
        std::cerr << "[Error] No TYPE6(ENCOS) motors found in config: " << config_path << std::endl;
        return 1;
    }

    std::cout << "[Info] Using config: " << config_path << std::endl;
    std::cout << "[Info] TYPE6 motor count: " << type6_indices.size() << std::endl;
    std::cout << "[Info] Preparing ENCOS TYPE6 MIT reciprocating test..." << std::endl;

    robot.ClearError_All();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    robot.SetModeAll_Type6(0); // MIT 模式
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
    std::vector<float> speed_limits(motor_count, 0.0f);
    std::vector<float> torque_ffs(motor_count, 0.0f);
    std::vector<MotorCmdVec> cmds(motor_count);

    for (size_t index = 0; index < motor_count; ++index) {
        const auto& info = robot.global_motors[index].info;
        const float current_pos = (index < start_positions.size()) ? start_positions[index] : 0.0f;
        const float pos_min = resolvePosMin(info);
        const float pos_max = resolvePosMax(info);

        if (info.api_type != 6) {
            centers[index] = current_pos;
            amplitudes[index] = 0.0f;
            speed_limits[index] = 0.0f;
            torque_ffs[index] = 0.0f;
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
            amplitude = std::min(0.2f * free_range, 0.45f);
            amplitude = std::max(amplitude, 0.08f);
            amplitude = std::min(amplitude, 0.5f * free_range);

            const float room_positive = pos_max - center;
            const float room_negative = center - pos_min;
            amplitude = std::min(amplitude, std::max(0.0f, std::min(room_positive, room_negative)));
        }

        float speed_limit = std::max(std::fabs(info.v_min), std::fabs(info.v_max));
        if (speed_limit <= 1e-6f) {
            speed_limit = 3.0f;
        }
        speed_limit = std::max(0.5f, std::min(5.0f, 0.25f * speed_limit));

        float torque_ff = 0.0f;
        float torque_abs_limit = resolveTorqueAbsLimit(info);
        if (torque_abs_limit > 1e-6f) {
            torque_ff = clampValue(torque_ff, -0.2f * torque_abs_limit, 0.2f * torque_abs_limit);
        }

        const float kp_cmd = (info.kp_in_use > 1e-6f) ? info.kp_in_use : 12.0f;
        const float kd_cmd = (info.kd_in_use > 1e-6f) ? info.kd_in_use : 0.4f;
        robot.SetKpd_N(kp_cmd, kd_cmd, static_cast<int>(index));

        centers[index] = center;
        amplitudes[index] = amplitude;
        speed_limits[index] = speed_limit;
        torque_ffs[index] = torque_ff;
        cmds[index].p = center;
        cmds[index].v = 0.0f;
        cmds[index].t = torque_ff;
    }

    std::cout << "[Info] Moving TYPE6 motors to start centers..." << std::endl;
    for (int step = 0; step <= 100 && g_running; ++step) {
        const float blend = static_cast<float>(step) / 100.0f;
        for (size_t index = 0; index < motor_count; ++index) {
            const auto& info = robot.global_motors[index].info;
            const float initial_pos = (index < start_positions.size()) ? start_positions[index] : 0.0f;
            if (info.api_type != 6) {
                cmds[index].p = initial_pos;
                cmds[index].v = 0.0f;
                cmds[index].t = 0.0f;
                continue;
            }

            cmds[index].p = initial_pos + (centers[index] - initial_pos) * blend;
            cmds[index].v = 0.0f;
            cmds[index].t = torque_ffs[index];
        }

        robot.Move(cmds);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    std::cout << "[Info] Start TYPE6 MIT reciprocating motion. Press Ctrl+C to stop." << std::endl;
    int iteration = 0;
    while (g_running) {
        const float phase = iteration * 0.05f;
        const float ratio = std::sin(phase);

        for (size_t index = 0; index < motor_count; ++index) {
            const auto& info = robot.global_motors[index].info;
            if (info.api_type != 6) {
                continue;
            }

            cmds[index].p = centers[index] + amplitudes[index] * ratio;
            cmds[index].v = 0.0f;
            cmds[index].t = torque_ffs[index];
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
