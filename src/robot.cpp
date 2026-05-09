#include "robot.hpp"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

namespace {

bool valid_motor_index(const std::vector<Motor_CAN_Struct>& motors, int idx) {
    return idx >= 0 && static_cast<std::size_t>(idx) < motors.size();
}

bool valid_device_index(const std::vector<DeviceX*>& devices, int idx) {
    return idx >= 0 && static_cast<std::size_t>(idx) < devices.size() && devices[idx] != nullptr;
}

}  // namespace

BaseRobot::BaseRobot(const std::string& config_file) {
    auto infos = MotorConfigLoader::loadConfig(config_file);
    if (infos.empty()) {
        std::cerr << "[Error] No motors loaded from " << config_file << std::endl;
        return;
    }

    int max_canid = 0;
    int device_counter = 0;
    for (auto& info : infos) {
        if (device_name_map_idx.find(info.device_name) == device_name_map_idx.end()) {
            device_name_map_idx[info.device_name] = device_counter++;
        }
        max_canid = std::max(max_canid, info.canid);
    }

    global_motors.resize(infos.size());
    std::vector<std::vector<uint>> topo_matrix;
    topo_matrix.reserve(infos.size());

    for (std::size_t i = 0; i < infos.size(); ++i) {
        auto& info = infos[i];
        info.device_index = device_name_map_idx[info.device_name];

        global_motors[i].info = info;
        global_motors[i].send.kd = info.kd_in_use;
        global_motors[i].send.kp = info.kp_in_use;

        topo_matrix.push_back({
            static_cast<uint>(info.device_index),
            0u,
            static_cast<uint>(info.canid),
            static_cast<uint>(i),
        });
    }

    mapper = TopoMapper({static_cast<uint>(device_counter), 1u, static_cast<uint>(max_canid + 1)},
                        topo_matrix);

    devices.resize(static_cast<std::size_t>(device_counter), nullptr);
    std::vector<int> device_api_types(static_cast<std::size_t>(device_counter), -1);

    for (std::size_t i = 0; i < global_motors.size(); ++i) {
        const auto& motor = global_motors[i];
        const int dev_idx = motor.info.device_index;
        if (dev_idx < 0 || static_cast<std::size_t>(dev_idx) >= devices.size()) {
            std::cerr << "[Error] Invalid device index for motor " << motor.info.name << std::endl;
            continue;
        }

        if (device_api_types[dev_idx] < 0) {
            device_api_types[dev_idx] = motor.info.api_type;
        } else if (device_api_types[dev_idx] != motor.info.api_type) {
            std::cerr << "[Warn] Mixed api_type on " << motor.info.device_name
                      << ": first=" << device_api_types[dev_idx]
                      << ", current=" << motor.info.api_type << std::endl;
        }

        if (devices[dev_idx] == nullptr) {
            std::cout << "[Init] Creating Device Driver for " << motor.info.device_name
                      << " | API Type: " << motor.info.api_type << std::endl;
            devices[dev_idx] = new DeviceX();
            devices[dev_idx]->Init(motor.info.device_name, dev_idx, &global_motors, &mapper);
        }
    }
}

BaseRobot::~BaseRobot() {
    try {
        DisableAll();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } catch (...) {
    }

    for (auto* device : devices) {
        delete device;
    }
}

int BaseRobot::SetKpd_N(float kp, float kd, int N) {
    if (!valid_motor_index(global_motors, N)) {
        std::cerr << "[Error] Motor index " << N << " out of range." << std::endl;
        return -1;
    }

    auto& motor = global_motors[N];
    motor.info.kp_in_use = kp;
    motor.info.kd_in_use = kd;
    motor.send.kp = kp;
    motor.send.kd = kd;
    return 0;
}

void BaseRobot::SetKpd_all(float kp, float kd) {
    for (std::size_t i = 0; i < global_motors.size(); ++i) {
        SetKpd_N(kp, kd, static_cast<int>(i));
    }
    std::cout << "[Info] Set KP=" << kp << ", KD=" << kd << " for all motors." << std::endl;
}

void BaseRobot::EnableAll() {
    for (std::size_t i = 0; i < global_motors.size(); ++i) {
        Enable_N(static_cast<int>(i));
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

void BaseRobot::DisableAll() {
    for (std::size_t i = 0; i < global_motors.size(); ++i) {
        Disable_N(static_cast<int>(i));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void BaseRobot::Enable_N(int N) {
    if (!valid_motor_index(global_motors, N)) {
        std::cerr << "[Error] Enable_N: motor index " << N << " out of range." << std::endl;
        return;
    }

    const int dev_idx = global_motors[N].info.device_index;
    if (!valid_device_index(devices, dev_idx)) {
        std::cerr << "[Error] Enable_N: device not found for motor " << N << std::endl;
        return;
    }

    devices[dev_idx]->EnableMotor(N);
}

void BaseRobot::Disable_N(int N) {
    if (!valid_motor_index(global_motors, N)) return;

    const int dev_idx = global_motors[N].info.device_index;
    if (valid_device_index(devices, dev_idx)) {
        devices[dev_idx]->DisableMotor(N);
    }
}

void BaseRobot::ClearError_N(int N) {
    if (!valid_motor_index(global_motors, N)) {
        std::cerr << "[Error] ClearError_N: motor index " << N << " out of range." << std::endl;
        return;
    }

    const int dev_idx = global_motors[N].info.device_index;
    if (!valid_device_index(devices, dev_idx)) {
        std::cerr << "[Error] ClearError_N: device not found for motor " << N << std::endl;
        return;
    }

    devices[dev_idx]->ClearError(N);
    std::cout << "[Info] Clear Error sent to Motor index: " << N
              << " (" << global_motors[N].info.name << ")" << std::endl;
}

void BaseRobot::SetZero_N(int N) {
    if (!valid_motor_index(global_motors, N)) {
        std::cerr << "[Error] SetZero_N: motor index " << N << " out of range." << std::endl;
        return;
    }

    const int dev_idx = global_motors[N].info.device_index;
    if (!valid_device_index(devices, dev_idx)) {
        std::cerr << "[Error] SetZero_N: device not found for motor " << N << std::endl;
        return;
    }

    devices[dev_idx]->SetZero(N);
    std::cout << "[Info] Set Zero sent to Motor index: " << N
              << " (" << global_motors[N].info.name << ")" << std::endl;
}

void BaseRobot::ClearError_All() {
    for (std::size_t i = 0; i < global_motors.size(); ++i) {
        ClearError_N(static_cast<int>(i));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "[Info] Clear Error command sent to all motors." << std::endl;
}

void BaseRobot::SetZero_All() {
    for (std::size_t i = 0; i < global_motors.size(); ++i) {
        SetZero_N(static_cast<int>(i));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "[Info] Set Zero command sent to all motors." << std::endl;
}

void BaseRobot::SetMode_N(int N, int mode) {
    if (!valid_motor_index(global_motors, N)) {
        std::cerr << "[Error] SetMode_N: motor index " << N << " out of range." << std::endl;
        return;
    }

    const int dev_idx = global_motors[N].info.device_index;
    if (valid_device_index(devices, dev_idx)) {
        devices[dev_idx]->SetMode(N, mode);
    } else {
        std::cerr << "[Error] SetMode_N: device not found for motor " << N << std::endl;
    }
}

void BaseRobot::SetModes(std::vector<int>& modes) {
    const std::size_t count = std::min(modes.size(), global_motors.size());
    for (std::size_t i = 0; i < count; ++i) {
        SetMode_N(static_cast<int>(i), modes[i]);
        std::this_thread::sleep_for(std::chrono::microseconds(1));
    }

    std::cout << "[Info] Batch SetModes completed for " << count << " motors." << std::endl;
}

void BaseRobot::SetModeAll_TypeX(int X, int mode) {
    for (std::size_t i = 0; i < global_motors.size(); ++i) {
        if (global_motors[i].info.api_type != X) continue;
        SetMode_N(static_cast<int>(i), mode);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void BaseRobot::Move_N(int N, const MotorCmdVec& target) {
    if (!valid_motor_index(global_motors, N)) return;

    auto& motor = global_motors[N];
    motor.send.position = Clip(target.p, motor.info.pos_min, motor.info.pos_max);
    motor.send.speed = target.v;
    motor.send.torque = target.t;

    const int dev_idx = motor.info.device_index;
    if (valid_device_index(devices, dev_idx)) {
        devices[dev_idx]->SendCommand(N);
    }
}

void BaseRobot::Move(const std::vector<MotorCmdVec>& targets) {
    const std::size_t count = std::min(targets.size(), global_motors.size());
    for (std::size_t i = 0; i < count; ++i) {
        auto& motor = global_motors[i];
        const auto& cmd = targets[i];
        motor.send.position = Clip(cmd.p, motor.info.pos_min, motor.info.pos_max);
        motor.send.speed = cmd.v;
        motor.send.torque = cmd.t;

        const int dev_idx = motor.info.device_index;
        if (valid_device_index(devices, dev_idx)) {
            int idx = static_cast<int>(i);
            devices[dev_idx]->SendCommand(idx);
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}

void BaseRobot::PrintStatus() {
    std::cout << "\n--- Robot Status (10Hz) | Total Motors: "
              << global_motors.size() << " ---\n";
    for (std::size_t i = 0; i < global_motors.size(); ++i) {
        const auto& m = global_motors[i];
        if (m.info.name.empty()) continue;

        std::cout << "Idx: " << i
                  << " | ID: " << m.info.num
                  << " [" << m.info.name << "]"
                  << " | Cmd: " << std::fixed << std::setprecision(2) << m.send.position
                  << " | Fault: " << static_cast<int>(m.recv.fault_message)
                  << " | ReadPos: " << m.recv.current_position_f.load()
                  << " | ReadVel: " << m.recv.current_speed_f.load()
                  << std::endl;
    }
    std::cout << "--------------------\n";
}

std::vector<float> BaseRobot::GetPosAll() {
    std::vector<float> positions;
    positions.reserve(global_motors.size());
    for (const auto& m : global_motors) {
        positions.push_back(m.recv.current_position_f.load());
    }
    return positions;
}

std::vector<float> BaseRobot::GetPosN(int n) {
    std::vector<float> positions;
    if (n <= 0) return positions;

    const std::size_t count = std::min(static_cast<std::size_t>(n), global_motors.size());
    positions.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        positions.push_back(global_motors[i].recv.current_position_f.load());
    }
    return positions;
}

void BaseRobot::QueryPos_N(int N) {
    if (!valid_motor_index(global_motors, N)) {
        std::cerr << "[Error] QueryPos_N: motor index " << N << " out of range." << std::endl;
        return;
    }

    const int dev_idx = global_motors[N].info.device_index;
    if (valid_device_index(devices, dev_idx)) {
        devices[dev_idx]->QueryPos(N);
    } else {
        std::cerr << "[Error] QueryPos_N: device not found for motor " << N << std::endl;
    }
}

void BaseRobot::QueryPos_ALL() {
    for (std::size_t i = 0; i < global_motors.size(); ++i) {
        const int dev_idx = global_motors[i].info.device_index;
        if (valid_device_index(devices, dev_idx)) {
            int idx = static_cast<int>(i);
            devices[dev_idx]->QueryPos(idx);
            std::this_thread::sleep_for(std::chrono::microseconds(200));
        }
    }
}

void BaseRobot::QueryVersion_N(int N) {
    if (!valid_motor_index(global_motors, N)) {
        std::cerr << "[Error] QueryVersion_N: motor index " << N << " out of range." << std::endl;
        return;
    }

    const int dev_idx = global_motors[N].info.device_index;
    if (valid_device_index(devices, dev_idx)) {
        devices[dev_idx]->QueryVersion(N);
    } else {
        std::cerr << "[Error] QueryVersion_N: device not found for motor " << N << std::endl;
    }
}
