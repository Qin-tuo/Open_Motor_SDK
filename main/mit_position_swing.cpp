#include "robot.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kDefaultAmplitudeDeg = 20.0f;
constexpr float kDefaultCycleHz = 0.2f;
constexpr float kDefaultControlHz = 100.0f;
constexpr int kPositionQueryCount = 30;
constexpr auto kPositionQueryDelay = std::chrono::milliseconds(20);

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
  std::cerr << "[mit_position_swing] Invalid " << name << "='" << text
            << "', fallback to " << fallback << "\n";
  return fallback;
}

bool all_motors_are_high_torque(const BaseRobot& robot) {
  for (const auto& motor : robot.global_motors) {
    if (motor.info.api_type != 5) return false;
  }
  return true;
}

std::vector<float> read_current_positions(BaseRobot& robot) {
  for (int i = 0; i < kPositionQueryCount; ++i) {
    robot.QueryPos_ALL();
    std::this_thread::sleep_for(kPositionQueryDelay);
  }
  return robot.GetPosAll();
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
    std::cerr << "[mit_position_swing] No motors loaded from " << config_path << std::endl;
    return 1;
  }

  if (!all_motors_are_high_torque(robot)) {
    std::cerr << "[mit_position_swing] This script only supports api_type=5 HighTorque motors."
              << std::endl;
    return 1;
  }

  std::vector<int> modes(robot.global_motors.size(), 0);
  robot.SetModes(modes);
  robot.ClearError_All();
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  robot.EnableAll();
  std::this_thread::sleep_for(std::chrono::milliseconds(80));

  std::cout << "[mit_position_swing] Reading current positions as motion centers..."
            << std::endl;
  const std::vector<float> centers = read_current_positions(robot);
  if (centers.size() != robot.global_motors.size()) {
    std::cerr << "[mit_position_swing] Position readback size mismatch." << std::endl;
    return 1;
  }

  for (std::size_t i = 0; i < centers.size(); ++i) {
    std::cout << "  idx=" << i << " name=" << robot.global_motors[i].info.name
              << " center_rad=" << std::fixed << std::setprecision(4) << centers[i]
              << " swing_rad=+/-" << amplitude_rad << std::endl;
  }

  std::vector<MotorCmdVec> commands(robot.global_motors.size(), MotorCmdVec{0.0f, 0.0f, 0.0f});
  auto start = std::chrono::steady_clock::now();
  auto next_tick = start;
  auto last_log = start;

  std::cout << "[mit_position_swing] MIT swing started: amplitude=" << amplitude_deg
            << "deg cycle_hz=" << cycle_hz << " control_hz=" << control_hz
            << ". Press Ctrl+C to stop." << std::endl;

  while (g_running.load()) {
    const auto now = std::chrono::steady_clock::now();
    const float t = std::chrono::duration<float>(now - start).count();
    const float offset = amplitude_rad * std::sin(omega * t);

    for (std::size_t i = 0; i < commands.size(); ++i) {
      commands[i].p = centers[i] + offset;
      commands[i].v = 0.0f;
      commands[i].t = 0.0f;
    }
    robot.Move(commands);

    if (now - last_log >= std::chrono::seconds(1)) {
      std::cout << "[mit_position_swing] offset_rad=" << std::fixed << std::setprecision(4)
                << offset << std::endl;
      last_log = now;
    }

    next_tick += period;
    std::this_thread::sleep_until(next_tick);
    if (std::chrono::steady_clock::now() > next_tick + period) {
      next_tick = std::chrono::steady_clock::now();
    }
  }

  std::cout << "[mit_position_swing] Stopping motors..." << std::endl;
  robot.DisableAll();
  return 0;
}
