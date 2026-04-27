#include "robot.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <limits>
#include <iostream>
#include <string>
#include <thread>

static std::atomic<bool> g_running{true};

static void handle_sigint(int) {
    g_running = false;
}

static float read_env_float(const char* key, float default_val) {
    const char* v = std::getenv(key);
    if (!v) return default_val;
    try {
        return std::stof(std::string(v));
    } catch (...) {
        return default_val;
    }
}

static bool read_env_bool(const char* key, bool default_val) {
    const char* v = std::getenv(key);
    if (!v) return default_val;
    const std::string s(v);
    return (s == "1" || s == "true" || s == "TRUE" || s == "on" || s == "ON");
}

static int select_target_motor_index(const BaseRobot& robot) {
    int num1_idx = -1;
    for (int i = 0; i < static_cast<int>(robot.global_motors.size()); ++i) {
        if (robot.global_motors[i].info.num == 1) {
            num1_idx = i;
            break;
        }
    }

    if (num1_idx >= 0 && robot.global_motors[num1_idx].info.api_type == 6) {
        return num1_idx;
    }

    for (int i = 0; i < static_cast<int>(robot.global_motors.size()); ++i) {
        if (robot.global_motors[i].info.api_type == 6) {
            return i;
        }
    }
    return -1;
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
        std::cerr << "[Error] No PFL28 motor found. Please set api_type=6 in config/motor.toml." << std::endl;
        return 1;
    }

    const auto& info = robot.global_motors[target_idx].info;
    std::cout << "[Info] Target motor: idx=" << target_idx
              << ", num=" << info.num
              << ", name=" << info.name
              << ", type=" << info.type
              << ", api_type=" << info.api_type
              << ", canid=" << info.canid << std::endl;

    float pos_min = (info.pos_max > info.pos_min) ? info.pos_min : info.p_min;
    float pos_max = (info.pos_max > info.pos_min) ? info.pos_max : info.p_max;
    if (!(pos_max > pos_min)) {
        pos_min = 0.0f;
        pos_max = 9.5f;
    }

    const float span = pos_max - pos_min;
    const bool allow_neg_i = (read_env_float("PFL28_ALLOW_NEG_CURRENT", 0.0f) > 0.5f);
    auto clamp_current = [&](float i_cmd) {
        if (allow_neg_i) {
            const float max_abs = (info.t_max > 0.0f) ? info.t_max : 2.5f;
            i_cmd = std::max(-max_abs, std::min(max_abs, i_cmd));
            return i_cmd;
        }
        if (info.t_max > info.t_min) {
            i_cmd = std::max(info.t_min, std::min(info.t_max, i_cmd));
        }
        if (i_cmd < 0.0f) i_cmd = 0.0f;
        return i_cmd;
    };

    const float base_current = clamp_current(read_env_float("PFL28_TEST_CURRENT", 2.0f));
    const float up_current = clamp_current(read_env_float("PFL28_UP_CURRENT", base_current));
    const float down_default = allow_neg_i ? -std::fabs(base_current) : 2.5f;
    const float down_current = clamp_current(read_env_float("PFL28_DOWN_CURRENT", down_default));

    const float warmup_sec = std::max(0.0f, read_env_float("PFL28_WARMUP_SEC", 6.0f));
    const float force_zero_sec = std::max(0.0f, read_env_float("PFL28_FORCE_ZERO_SEC", 2.0f));
    const float feedback_wait_sec = std::max(0.2f, read_env_float("PFL28_FEEDBACK_WAIT_SEC", 2.0f));
    const float stall_abort_sec = std::max(0.0f, read_env_float("PFL28_STALL_ABORT_SEC", 4.0f));
    const float phase_timeout_sec = std::max(0.5f, read_env_float("PFL28_PHASE_TIMEOUT_SEC", 6.0f));
    const float switch_eps = std::max(0.001f, read_env_float("PFL28_SWITCH_EPS", 0.03f));
    const int stable_need = std::max(1, static_cast<int>(read_env_float("PFL28_STABLE_COUNT", 8.0f)));
    const int cmd_period_ms = std::max(10, static_cast<int>(read_env_float("PFL28_CMD_PERIOD_MS", 20.0f)));
    const float dt = static_cast<float>(cmd_period_ms) / 1000.0f;
    const float max_step = std::max(0.0f, read_env_float("PFL28_MAX_STEP", 0.05f)); // per tick
    const bool use_canfd = read_env_bool("PFL28_USE_CANFD", true);

    float high_pos = read_env_float("PFL28_HIGH_POS", pos_max - 0.2f * span);
    float low_pos = read_env_float("PFL28_LOW_POS", pos_min + 0.2f * span);
    high_pos = std::max(pos_min, std::min(pos_max, high_pos));
    low_pos = std::max(pos_min, std::min(pos_max, low_pos));

    robot.Enable_N(target_idx);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    std::cout << "[Info] Start PFL28 step test: low=" << low_pos
              << ", high=" << high_pos
              << ", up_current=" << up_current
              << ", down_current=" << down_current
              << ", allow_neg_i=" << (allow_neg_i ? 1 : 0)
              << ", warmup=" << warmup_sec
              << "s, force_zero=" << force_zero_sec
              << "s, wait_fb=" << feedback_wait_sec
              << "s, stall_abort=" << stall_abort_sec
              << "s, timeout=" << phase_timeout_sec
              << "s, eps=" << switch_eps
              << ", max_step=" << max_step
              << ", period_ms=" << cmd_period_ms
              << "ms, tx=" << (use_canfd ? "canfd" : "classic-can")
              << ". Ctrl+C to stop." << std::endl;
    if (cmd_period_ms > 50) {
        std::cout << "[Warn] cmd period is " << cmd_period_ms
                  << "ms. This may be too slow for actuator watchdog. "
                  << "Recommend 10~20ms." << std::endl;
    }

    if (warmup_sec > 0.0f) {
        std::cout << "[Info] Waiting for actuator auto-homing..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(warmup_sec * 1000.0f)));
    }

    if (force_zero_sec > 0.0f && g_running.load()) {
        std::cout << "[Info] Forcing set_pos(0.0) for " << force_zero_sec << "s before loop." << std::endl;
        const int z_steps = std::max(1, static_cast<int>(force_zero_sec / dt));
        for (int zi = 0; zi < z_steps && g_running.load(); ++zi) {
            MotorCmdVec zcmd{};
            zcmd.p = 0.0f;
            zcmd.v = 0.0f;
            zcmd.t = std::fabs(down_current);
            robot.Move_N(target_idx, zcmd);
            std::this_thread::sleep_for(std::chrono::milliseconds(cmd_period_ms));
        }
    }

    float cmd_pos_slew = std::numeric_limits<float>::quiet_NaN();
    if (g_running.load()) {
        std::cout << "[Info] Waiting for first full state frame (pos+cur)..." << std::endl;
        const int sync_steps = std::max(1, static_cast<int>(feedback_wait_sec / dt));
        for (int si = 0; si < sync_steps && g_running.load(); ++si) {
            const auto& m = robot.global_motors[target_idx];
            const float fb_pos = m.recv.current_position_f.load();
            const bool got_full_state =
                (m.recv.motor_id == static_cast<uint8_t>(info.canid)) &&
                (m.recv.motor_state == 1) &&
                std::isfinite(fb_pos);
            if (got_full_state) {
                cmd_pos_slew = std::max(pos_min, std::min(pos_max, fb_pos));
                break;
            }

            // Keep requesting state with zero current before motion loop starts.
            MotorCmdVec sync_cmd{};
            sync_cmd.p = 0.5f * (low_pos + high_pos);
            sync_cmd.v = 0.0f;
            sync_cmd.t = 0.0f;
            robot.Move_N(target_idx, sync_cmd);
            std::this_thread::sleep_for(std::chrono::milliseconds(cmd_period_ms));
        }

        if (std::isfinite(cmd_pos_slew)) {
            std::cout << "[Info] Feedback synced at pos=" << cmd_pos_slew << std::endl;
        } else {
            cmd_pos_slew = std::max(pos_min, std::min(pos_max, low_pos));
            std::cout << "[Warn] No full state frame received in " << feedback_wait_sec
                      << "s. Fallback cmd start pos=" << cmd_pos_slew
                      << ". Check power/enable chain if motion is still rejected."
                      << std::endl;
        }
    }

    bool phase_up = true;
    float phase_elapsed = 0.0f;
    float no_drive_elapsed = 0.0f;
    float stall_elapsed = 0.0f;
    int stable_count = 0;
    int i = 0;
    bool fault6_hint_printed = false;
    bool nodrive_hint_printed = false;
    float last_fb_pos = std::numeric_limits<float>::quiet_NaN();

    while (g_running.load()) {
        const float target_pos = phase_up ? high_pos : low_pos;
        const float target_current = phase_up ? up_current : down_current;
        const char* phase_name = phase_up ? "up" : "down";

        if (max_step > 0.0f) {
            const float delta = target_pos - cmd_pos_slew;
            if (delta > max_step) cmd_pos_slew += max_step;
            else if (delta < -max_step) cmd_pos_slew -= max_step;
            else cmd_pos_slew = target_pos;
        } else {
            cmd_pos_slew = target_pos;
        }

        MotorCmdVec cmd{};
        cmd.p = cmd_pos_slew;
        cmd.v = 0.0f;
        cmd.t = target_current;
        robot.Move_N(target_idx, cmd);

        const auto& m = robot.global_motors[target_idx];
        const float fb_pos = m.recv.current_position_f.load();
        const float fb_cur = m.recv.current_torque_f.load();
        const float err = std::fabs(fb_pos - target_pos);
        const bool expect_drive = (err > 0.3f) && (std::fabs(target_current) >= 0.8f);
        const bool no_drive = expect_drive && (std::fabs(fb_cur) < 0.02f);

        if (no_drive) no_drive_elapsed += dt;
        else no_drive_elapsed = 0.0f;

        if (std::isfinite(last_fb_pos) && no_drive) {
            if (std::fabs(fb_pos - last_fb_pos) < 0.002f) stall_elapsed += dt;
            else stall_elapsed = 0.0f;
        } else {
            stall_elapsed = 0.0f;
        }
        last_fb_pos = fb_pos;

        if (err <= switch_eps) {
            ++stable_count;
        } else {
            stable_count = 0;
        }

        if (i % 25 == 0) {
            std::cout << "[" << phase_name << " i=" << i << "] cmd_pos=" << target_pos
                      << " cmd_slew=" << cmd_pos_slew
                      << " fb_pos=" << fb_pos
                      << " fb_cur=" << fb_cur
                      << " err=" << err
                      << " fault=" << static_cast<int>(m.recv.fault_message) << std::endl;
        }

        if (!fault6_hint_printed && m.recv.fault_message == 6) {
            std::cout << "[Hint] fault=6 on PFL28: command rejected. Check limits: pos in [0, 9.5], current in [0, 2.5], "
                      << "and avoid negative current unless firmware explicitly supports it. "
                      << "If it persists, recalibrate zero/stroke with vendor tool."
                      << std::endl;
            fault6_hint_printed = true;
        }
        if (!nodrive_hint_printed && no_drive_elapsed > 2.0f) {
            std::cout << "[Hint] No drive current observed for >2s while position error is large. "
                      << "Check: 1) command period <=20ms, 2) actuator power/ESTOP/enable chain, "
                      << "3) auto-homing finished, 4) current command near upper range (e.g. 2.0~2.5A)."
                      << std::endl;
            nodrive_hint_printed = true;
        }
        if (stall_abort_sec > 0.0f && stall_elapsed >= stall_abort_sec) {
            std::cout << "[Error] Position stuck for " << stall_elapsed
                      << "s with near-zero current and large position error. "
                      << "Likely actuator is not really enabled or power/ESTOP chain is open."
                      << std::endl;
            break;
        }

        phase_elapsed += dt;
        if (stable_count >= stable_need || phase_elapsed >= phase_timeout_sec) {
            if (phase_elapsed >= phase_timeout_sec && stable_count < stable_need) {
                std::cout << "[Warn] phase timeout at " << phase_name
                          << ", pos_err=" << err
                          << ". Switching direction." << std::endl;
            }
            phase_up = !phase_up;
            phase_elapsed = 0.0f;
            stable_count = 0;
            std::cout << "[Info] switch phase -> " << (phase_up ? "up" : "down") << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(cmd_period_ms));
        ++i;
    }

    MotorCmdVec stop_cmd{};
    stop_cmd.p = robot.global_motors[target_idx].recv.current_position_f.load();
    stop_cmd.v = 0.0f;
    stop_cmd.t = 0.0f;
    robot.Move_N(target_idx, stop_cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    robot.Disable_N(target_idx);
    return 0;
}
