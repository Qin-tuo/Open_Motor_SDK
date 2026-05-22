#include "robot.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <exception>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace {

std::atomic<bool> g_running{true};

void handle_sigint(int) {
    g_running = false;
}

struct Options {
    std::string config_path;
    float wheel_speed_rad_s = 2.0f;
    float wheel_current_limit_a = 3.0f;
    float swing_amp_rad = 0.20f;
    float swing_freq_hz = 0.25f;
    float zero_seconds = 3.0f;
    float loop_hz = 100.0f;
    bool help_requested = false;
};

std::string default_config_path() {
    try {
        return ament_index_cpp::get_package_share_directory("khcan") + "/config/motor.toml";
    } catch (const std::exception&) {
        return "config/motor.toml";
    }
}

std::string normalize_type(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::toupper(c));
    });
    return value;
}

bool is_rs01_speed_motor(const Motor_CAN_Info_Struct& info) {
    const std::string type = normalize_type(info.type);
    return info.api_type == 1 && (type == "RS01" || type == "LZRS01");
}

float positive_or(float value, float fallback) {
    return (std::isfinite(value) && value > 0.0f) ? value : fallback;
}

void print_usage(const char* argv0) {
    std::cout
        << "Usage: " << argv0 << " [options]\n"
        << "  --config PATH             motor.toml path\n"
        << "  --wheel-speed RAD_S       RS01 speed command, default 2.0\n"
        << "  --wheel-current AMP       RS01 current limit, default 3.0\n"
        << "  --amp RAD                 joint swing amplitude, default 0.20\n"
        << "  --freq HZ                 joint swing frequency, default 0.25\n"
        << "  --zero-seconds SEC        homing-to-zero duration, default 3.0\n"
        << "  --loop-hz HZ              control loop rate, default 100\n";
}

bool parse_args(int argc, char** argv, Options& opts) {
    opts.config_path = default_config_path();

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto require_value = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "[Error] Missing value for " << name << std::endl;
                return nullptr;
            }
            return argv[++i];
        };

        try {
            if (arg == "--config") {
                const char* value = require_value("--config");
                if (!value) return false;
                opts.config_path = value;
            } else if (arg == "--wheel-speed") {
                const char* value = require_value("--wheel-speed");
                if (!value) return false;
                opts.wheel_speed_rad_s = std::stof(value);
            } else if (arg == "--wheel-current") {
                const char* value = require_value("--wheel-current");
                if (!value) return false;
                opts.wheel_current_limit_a = std::stof(value);
            } else if (arg == "--amp") {
                const char* value = require_value("--amp");
                if (!value) return false;
                opts.swing_amp_rad = std::stof(value);
            } else if (arg == "--freq") {
                const char* value = require_value("--freq");
                if (!value) return false;
                opts.swing_freq_hz = std::stof(value);
            } else if (arg == "--zero-seconds") {
                const char* value = require_value("--zero-seconds");
                if (!value) return false;
                opts.zero_seconds = std::stof(value);
            } else if (arg == "--loop-hz") {
                const char* value = require_value("--loop-hz");
                if (!value) return false;
                opts.loop_hz = std::stof(value);
            } else if (arg == "--help" || arg == "-h") {
                opts.help_requested = true;
                print_usage(argv[0]);
                return false;
            } else {
                std::cerr << "[Error] Unknown option: " << arg << std::endl;
                print_usage(argv[0]);
                return false;
            }
        } catch (const std::exception& e) {
            std::cerr << "[Error] Failed to parse " << arg << ": " << e.what() << std::endl;
            return false;
        }
    }

    opts.wheel_current_limit_a = positive_or(opts.wheel_current_limit_a, 3.0f);
    opts.swing_amp_rad = positive_or(opts.swing_amp_rad, 0.20f);
    opts.swing_freq_hz = positive_or(opts.swing_freq_hz, 0.25f);
    opts.zero_seconds = positive_or(opts.zero_seconds, 3.0f);
    opts.loop_hz = positive_or(opts.loop_hz, 100.0f);
    return true;
}

void send_rs01_speed(BaseRobot& robot, const std::vector<int>& indices,
                     float speed_rad_s, float current_limit_a) {
    for (int idx : indices) {
        MotorCmdVec cmd{};
        cmd.p = 0.0f;
        cmd.v = speed_rad_s;
        cmd.t = current_limit_a;
        robot.Move_N(idx, cmd);
    }
}

void send_joint_positions(BaseRobot& robot, const std::vector<int>& indices,
                          const std::vector<float>& positions,
                          const std::vector<float>& velocities) {
    for (std::size_t i = 0; i < indices.size(); ++i) {
        MotorCmdVec cmd{};
        cmd.p = positions[i];
        cmd.v = velocities[i];
        cmd.t = 0.0f;
        robot.Move_N(indices[i], cmd);
    }
}

void stop_all(BaseRobot& robot, const std::vector<int>& rs01_indices,
              const std::vector<int>& swing_indices) {
    for (int i = 0; i < 20; ++i) {
        send_rs01_speed(robot, rs01_indices, 0.0f, 0.0f);

        std::vector<float> positions(swing_indices.size(), 0.0f);
        std::vector<float> velocities(swing_indices.size(), 0.0f);
        send_joint_positions(robot, swing_indices, positions, velocities);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    robot.DisableAll();
}

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, handle_sigint);

    Options opts;
    if (!parse_args(argc, argv, opts)) {
        return opts.help_requested ? 0 : 1;
    }

    BaseRobot robot(opts.config_path);
    if (robot.global_motors.empty()) {
        std::cerr << "[Error] No motors loaded from " << opts.config_path << std::endl;
        return 1;
    }

    std::vector<int> rs01_indices;
    std::vector<int> swing_indices;
    for (int i = 0; i < static_cast<int>(robot.global_motors.size()); ++i) {
        const auto& info = robot.global_motors[i].info;
        if (is_rs01_speed_motor(info)) {
            rs01_indices.push_back(i);
        } else {
            swing_indices.push_back(i);
        }
    }

    if (rs01_indices.empty()) {
        std::cerr << "[Error] No RS01 api_type=1 motor found." << std::endl;
        return 1;
    }
    if (swing_indices.empty()) {
        std::cerr << "[Error] No non-RS01 joint motor found for swing test." << std::endl;
        return 1;
    }
    if (swing_indices.size() != 8) {
        std::cerr << "[Warn] Expected 8 swing motors, found " << swing_indices.size() << "." << std::endl;
    }

    std::cout << "[Info] Config: " << opts.config_path << std::endl;
    std::cout << "[Info] RS01 speed motors:";
    for (int idx : rs01_indices) {
        const auto& info = robot.global_motors[idx].info;
        std::cout << " idx=" << idx << "(" << info.name << ", id=" << info.canid << ")";
    }
    std::cout << std::endl;

    std::cout << "[Info] Swing motors:";
    for (int idx : swing_indices) {
        const auto& info = robot.global_motors[idx].info;
        std::cout << " idx=" << idx << "(" << info.name << ", id=" << info.canid << ")";
    }
    std::cout << std::endl;

    robot.DisableAll();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    robot.ClearError_All();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    for (int idx : rs01_indices) {
        robot.SetMode_N(idx, 2); // Type1 speed mode.
    }
    for (int idx : swing_indices) {
        robot.SetMode_N(idx, 0); // ENCOS MIT/force-position mixed mode.
        const auto& info = robot.global_motors[idx].info;
        const float kp = (std::fabs(info.kp_in_use) > 1e-6f) ? info.kp_in_use : 20.0f;
        const float kd = (std::fabs(info.kd_in_use) > 1e-6f) ? info.kd_in_use : 0.8f;
        robot.SetKpd_N(kp, kd, idx);
    }

    robot.EnableAll();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    for (int i = 0; i < 20; ++i) {
        robot.QueryPos_ALL();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::vector<float> start_positions;
    start_positions.reserve(swing_indices.size());
    for (int idx : swing_indices) {
        start_positions.push_back(robot.global_motors[idx].recv.current_position_f.load());
    }

    const float dt = 1.0f / opts.loop_hz;
    const auto period = std::chrono::microseconds(static_cast<int64_t>(1000000.0f * dt));
    const int zero_steps = std::max(1, static_cast<int>(std::lround(opts.zero_seconds * opts.loop_hz)));

    std::cout << "[Info] Returning " << swing_indices.size()
              << " joint motors to zero over " << opts.zero_seconds << "s." << std::endl;

    for (int step = 0; g_running.load() && step <= zero_steps; ++step) {
        const auto loop_start = std::chrono::steady_clock::now();
        const float u = static_cast<float>(step) / static_cast<float>(zero_steps);
        const float smooth = u * u * (3.0f - 2.0f * u);

        std::vector<float> positions;
        std::vector<float> velocities;
        positions.reserve(swing_indices.size());
        velocities.reserve(swing_indices.size());
        for (float start : start_positions) {
            positions.push_back(start * (1.0f - smooth));
            velocities.push_back(0.0f);
        }

        send_joint_positions(robot, swing_indices, positions, velocities);
        send_rs01_speed(robot, rs01_indices, 0.0f, opts.wheel_current_limit_a);

        const auto elapsed = std::chrono::steady_clock::now() - loop_start;
        if (elapsed < period) {
            std::this_thread::sleep_for(period - elapsed);
        }
    }

    if (!g_running.load()) {
        stop_all(robot, rs01_indices, swing_indices);
        return 0;
    }

    std::cout << "[Info] Starting swing. RS01 speed=" << opts.wheel_speed_rad_s
              << " rad/s, current limit=" << opts.wheel_current_limit_a
              << " A, amp=" << opts.swing_amp_rad
              << " rad, freq=" << opts.swing_freq_hz << " Hz." << std::endl;

    constexpr float kTwoPi = 6.28318530718f;
    const float omega = kTwoPi * opts.swing_freq_hz;
    int step = 0;

    while (g_running.load()) {
        const auto loop_start = std::chrono::steady_clock::now();
        const float t = static_cast<float>(step) * dt;
        const float target = opts.swing_amp_rad * std::sin(omega * t);
        const float velocity = opts.swing_amp_rad * omega * std::cos(omega * t);

        std::vector<float> positions(swing_indices.size(), target);
        std::vector<float> velocities(swing_indices.size(), velocity);

        send_joint_positions(robot, swing_indices, positions, velocities);
        send_rs01_speed(robot, rs01_indices, opts.wheel_speed_rad_s, opts.wheel_current_limit_a);

        if (step % static_cast<int>(std::max(1.0f, opts.loop_hz)) == 0) {
            std::cout << "[Info] target=" << target << " rad";
            for (int idx : swing_indices) {
                const auto& motor = robot.global_motors[idx];
                std::cout << " | " << motor.info.name
                          << ": p=" << motor.recv.current_position_f.load()
                          << ", v=" << motor.recv.current_speed_f.load();
            }
            std::cout << std::endl;
        }

        const auto elapsed = std::chrono::steady_clock::now() - loop_start;
        if (elapsed < period) {
            std::this_thread::sleep_for(period - elapsed);
        }
        ++step;
    }

    std::cout << "\n[Info] Stopping motors..." << std::endl;
    stop_all(robot, rs01_indices, swing_indices);
    return 0;
}
