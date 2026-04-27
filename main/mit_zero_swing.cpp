#include "robot.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kDefaultAmplitudeDeg = 15.0f;
constexpr float kDefaultCycleHz = 0.2f;
constexpr float kDefaultControlHz = 500.0f;

std::atomic<bool> g_running{true};

void handle_signal(int) {
  g_running.store(false);
}

float parse_positive_float(const char* text, float fallback, const char* name) {
  try {
    const float value = std::stof(text);
    if (value > 0.0f) return value;
  } catch (const std::exception&) {
  }
  std::cerr << "[mit_zero_swing] Invalid " << name << "='" << text
            << "', fallback to " << fallback << "\n";
  return fallback;
}

int mit_mode_for_api_type(int api_type) {
  if (api_type == 2) return 1;
  return 0;
}

bool all_motor_types_are_supported(const BaseRobot& robot) {
  for (const auto& motor : robot.global_motors) {
    if (motor.info.api_type < 1 || motor.info.api_type > 5) return false;
  }
  return true;
}

void warn_positionless_motor_types(const BaseRobot& robot) {
  for (const auto& motor : robot.global_motors) {
    if (motor.info.api_type == 4) {
      std::cerr << "[mit_zero_swing] Warning: " << motor.info.name
                << " uses api_type=4. The current driver only sends torque/current for this type, "
                << "so zero-centered position swing is not available for it.\n";
    }
  }
}

}  // namespace

int main(int argc, char* argv[]) {
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  float amplitude_deg = kDefaultAmplitudeDeg;
  float cycle_hz = kDefaultCycleHz;
  float control_hz = kDefaultControlHz;

  if (argc > 1) amplitude_deg = parse_positive_float(argv[1], amplitude_deg, "amplitude_deg");
  if (argc > 2) cycle_hz = parse_positive_float(argv[2], cycle_hz, "cycle_hz");
  if (argc > 3) control_hz = parse_positive_float(argv[3], control_hz, "control_hz");

  const float amplitude_rad = amplitude_deg * kDegToRad;
  const float omega = 2.0f * kPi * cycle_hz;
  const auto period =
      std::chrono::microseconds(static_cast<int64_t>(1000000.0f / control_hz));

  const std::string share_dir = ament_index_cpp::get_package_share_directory("khcan");
  const std::string config_path = share_dir + "/config/motor.toml";
  BaseRobot robot(config_path);
  if (robot.global_motors.empty()) {
    std::cerr << "[mit_zero_swing] No motors loaded from " << config_path << std::endl;
    return 1;
  }

  if (!all_motor_types_are_supported(robot)) {
    std::cerr << "[mit_zero_swing] Unsupported api_type in " << config_path
              << std::endl;
    return 1;
  }
  warn_positionless_motor_types(robot);

  std::vector<int> modes;
  modes.reserve(robot.global_motors.size());
  for (const auto& motor : robot.global_motors) {
    modes.push_back(mit_mode_for_api_type(motor.info.api_type));
  }
  robot.SetModes(modes);
  robot.ClearError_All();
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  robot.EnableAll();
  std::this_thread::sleep_for(std::chrono::milliseconds(80));

  std::vector<MotorCmdVec> commands(robot.global_motors.size(), MotorCmdVec{0.0f, 0.0f, 0.0f});
  auto start = std::chrono::steady_clock::now();
  auto next_tick = start;
  auto last_log = start;

  std::cout << "[mit_zero_swing] MIT zero-centered swing started: amplitude="
            << amplitude_deg << "deg cycle_hz=" << cycle_hz
            << " control_hz=" << control_hz
            << ". Press Ctrl+C to stop." << std::endl;

  while (g_running.load()) {
    const auto now = std::chrono::steady_clock::now();
    const float t = std::chrono::duration<float>(now - start).count();
    const float target_pos = amplitude_rad * std::sin(omega * t);

    for (auto& command : commands) {
      command.p = target_pos;
      command.v = 0.0f;
      command.t = 0.0f;
    }
    robot.Move(commands);

    if (now - last_log >= std::chrono::seconds(1)) {
      std::cout << "[mit_zero_swing] target_rad=" << std::fixed << std::setprecision(4)
                << target_pos << std::endl;
      last_log = now;
    }

    next_tick += period;
    std::this_thread::sleep_until(next_tick);
    if (std::chrono::steady_clock::now() > next_tick + period) {
      next_tick = std::chrono::steady_clock::now();
    }
  }

  std::cout << "[mit_zero_swing] Stopping motors..." << std::endl;
  robot.DisableAll();
  return 0;
}
