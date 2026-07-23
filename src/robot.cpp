#include "robot.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace {

bool valid_index(std::size_t count, int index) {
    return index >= 0 && static_cast<std::size_t>(index) < count;
}

}  // namespace

BaseRobot::BaseRobot(const std::string& config_file, bool disable_on_destroy)
    : disable_on_destroy_(disable_on_destroy) {
    auto infos = MotorConfigLoader::loadConfig(config_file);

    for (const auto& info : infos) {
        if (device_name_to_index_.count(info.device_name) == 0) {
            const int index = static_cast<int>(device_name_to_index_.size());
            device_name_to_index_.emplace(info.device_name, index);
        }
    }

    motors_.resize(infos.size());
    for (std::size_t i = 0; i < infos.size(); ++i) {
        auto info = infos[i];
        info.device_index = device_name_to_index_.at(info.device_name);
        motors_[i].info = info;
        motors_[i].send.kp = info.kp_in_use;
        motors_[i].send.kd = info.kd_in_use;

        if (!mapper_.add(info.device_index, motor_uses_extended_frame(info.api_type),
                         info.canid, static_cast<int>(i))) {
            throw std::runtime_error("Duplicate motor feedback route for " + info.name);
        }
    }

    last_command_ns_.assign(motors_.size(), 0);
    command_armed_.assign(motors_.size(), false);
    devices_.resize(device_name_to_index_.size());

    for (const auto& motor : motors_) {
        const int device_index = motor.info.device_index;
        if (devices_[device_index]) continue;

        auto device = std::make_unique<DeviceX>();
        std::cout << "[Init] Creating CAN device " << motor.info.device_name << std::endl;
        if (!device->Init(motor.info.device_name, device_index, &motors_, &mapper_,
                          &motor_mutex_)) {
            throw std::runtime_error("Failed to initialize " + motor.info.device_name);
        }
        devices_[device_index] = std::move(device);
    }

}

BaseRobot::~BaseRobot() {
    if (!disable_on_destroy_) return;
    try {
        DisableAll();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } catch (...) {
    }
}

std::size_t BaseRobot::MotorCount() const {
    return motors_.size();
}

bool BaseRobot::GetMotorInfo(int index, Motor_CAN_Info_Struct& info) const {
    std::lock_guard<std::mutex> lock(motor_mutex_);
    if (!valid_index(motors_.size(), index)) return false;
    info = motors_[index].info;
    return true;
}

bool BaseRobot::GetMotorSnapshot(int index, Motor_CAN_Struct& motor) const {
    std::lock_guard<std::mutex> lock(motor_mutex_);
    if (!valid_index(motors_.size(), index)) return false;
    motor = motors_[index];
    return true;
}

bool BaseRobot::GetCommand_N(int index, MotorCmdVec& command) const {
    std::lock_guard<std::mutex> lock(motor_mutex_);
    if (!valid_index(motors_.size(), index)) return false;
    command = {motors_[index].send.position, motors_[index].send.speed,
               motors_[index].send.torque};
    return true;
}

DeviceX* BaseRobot::deviceForMotor(int index) const {
    if (!valid_index(motors_.size(), index)) return nullptr;
    const int device_index = motors_[index].info.device_index;
    if (!valid_index(devices_.size(), device_index)) return nullptr;
    return devices_[device_index].get();
}

bool BaseRobot::EnableAll() {
    bool ok = true;
    for (int i = 0; i < static_cast<int>(motors_.size()); ++i) {
        ok = Enable_N(i) && ok;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return ok;
}

bool BaseRobot::DisableAll() {
    bool ok = true;
    for (int i = 0; i < static_cast<int>(motors_.size()); ++i) {
        ok = Disable_N(i) && ok;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return ok;
}

bool BaseRobot::Enable_N(int index) {
    DeviceX* device = deviceForMotor(index);
    return device && device->EnableMotor(index);
}

bool BaseRobot::Disable_N(int index) {
    DeviceX* device = deviceForMotor(index);
    const bool ok = device && device->DisableMotor(index);
    if (ok && valid_index(command_armed_.size(), index)) {
        std::lock_guard<std::mutex> lock(motor_mutex_);
        command_armed_[index] = false;
    }
    return ok;
}

int BaseRobot::SetKpd_N(float kp, float kd, int index) {
    std::lock_guard<std::mutex> lock(motor_mutex_);
    if (!valid_index(motors_.size(), index) || !std::isfinite(kp) || !std::isfinite(kd)) {
        return -1;
    }
    auto& motor = motors_[index];
    if (kp < motor.info.kp_min || kp > motor.info.kp_max ||
        kd < motor.info.kd_min || kd > motor.info.kd_max) {
        return -1;
    }
    motor.info.kp_in_use = kp;
    motor.info.kd_in_use = kd;
    motor.send.kp = kp;
    motor.send.kd = kd;
    return 0;
}

bool BaseRobot::SetKpd_all(float kp, float kd) {
    bool ok = true;
    for (int i = 0; i < static_cast<int>(motors_.size()); ++i) {
        ok = (SetKpd_N(kp, kd, i) == 0) && ok;
    }
    return ok;
}

bool BaseRobot::ClearError_N(int index) {
    DeviceX* device = deviceForMotor(index);
    return device && device->ClearError(index);
}

bool BaseRobot::SetZero_N(int index) {
    DeviceX* device = deviceForMotor(index);
    return device && device->SetZero(index);
}

bool BaseRobot::ClearError_All() {
    bool ok = true;
    for (int i = 0; i < static_cast<int>(motors_.size()); ++i) {
        ok = ClearError_N(i) && ok;
    }
    return ok;
}

bool BaseRobot::SetZero_All() {
    bool ok = true;
    for (int i = 0; i < static_cast<int>(motors_.size()); ++i) {
        ok = SetZero_N(i) && ok;
    }
    return ok;
}

bool BaseRobot::Stage_N(int index, const MotorCmdVec& target) {
    std::lock_guard<std::mutex> lock(motor_mutex_);
    if (!valid_index(motors_.size(), index)) return false;

    auto command = motors_[index].send;
    command.position = target.p;
    command.speed = target.v;
    command.torque = target.t;
    if (!sanitize_motor_command(motors_[index].info, command)) return false;
    motors_[index].send = command;
    return true;
}

bool BaseRobot::Flush_N(int index) {
    DeviceX* device = deviceForMotor(index);
    if (!device || !device->SendCommand(index)) return false;

    std::lock_guard<std::mutex> lock(motor_mutex_);
    last_command_ns_[index] = steady_time_ns();
    command_armed_[index] = true;
    return true;
}

bool BaseRobot::Move_N(int index, const MotorCmdVec& target) {
    return Stage_N(index, target) && Flush_N(index);
}

bool BaseRobot::Move(const std::vector<MotorCmdVec>& targets) {
    const std::size_t count = std::min(targets.size(), motors_.size());
    bool ok = targets.size() == motors_.size();
    for (std::size_t i = 0; i < count; ++i) {
        ok = Move_N(static_cast<int>(i), targets[i]) && ok;
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    return ok;
}

bool BaseRobot::SetModes(const std::vector<int>& modes) {
    const std::size_t count = std::min(modes.size(), motors_.size());
    bool ok = modes.size() == motors_.size();
    for (std::size_t i = 0; i < count; ++i) {
        ok = SetMode_N(static_cast<int>(i), modes[i]) && ok;
    }
    return ok;
}

bool BaseRobot::SetMode_N(int index, int mode) {
    DeviceX* device = deviceForMotor(index);
    return device && device->SetMode(index, mode);
}

bool BaseRobot::ConfigureHaitaiMitLimits_N(int index) {
    DeviceX* device = deviceForMotor(index);
    return device && device->ConfigureHaitaiMitLimits(index);
}

bool BaseRobot::SetModeAll_TypeX(int api_type, int mode) {
    bool ok = true;
    for (int i = 0; i < static_cast<int>(motors_.size()); ++i) {
        Motor_CAN_Info_Struct info;
        if (GetMotorInfo(i, info) && info.api_type == api_type) {
            ok = SetMode_N(i, mode) && ok;
        }
    }
    return ok;
}

bool BaseRobot::QueryPos_ALL() {
    bool ok = true;
    for (int i = 0; i < static_cast<int>(motors_.size()); ++i) {
        ok = QueryPos_N(i) && ok;
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    return ok;
}

bool BaseRobot::QueryPos_N(int index) {
    DeviceX* device = deviceForMotor(index);
    return device && device->QueryPos(index);
}

bool BaseRobot::QueryVersion_N(int index) {
    DeviceX* device = deviceForMotor(index);
    return device && device->QueryVersion(index);
}

int BaseRobot::FindMotorIndexByName(const std::string& name) const {
    std::lock_guard<std::mutex> lock(motor_mutex_);
    for (std::size_t i = 0; i < motors_.size(); ++i) {
        if (motors_[i].info.name == name) return static_cast<int>(i);
    }
    return -1;
}

bool BaseRobot::IsMotorReady(int index) const {
    DeviceX* device = deviceForMotor(index);
    return device && device->SocketReady();
}

void BaseRobot::SetCommandTimeout(std::chrono::milliseconds timeout) {
    std::lock_guard<std::mutex> lock(motor_mutex_);
    command_timeout_ns_ = timeout.count() <= 0
        ? 0
        : static_cast<uint64_t>(timeout.count()) * 1000000ULL;
}

int BaseRobot::CheckCommandTimeouts() {
    const uint64_t now_ns = steady_time_ns();
    std::vector<int> expired;
    {
        std::lock_guard<std::mutex> lock(motor_mutex_);
        if (command_timeout_ns_ == 0) return 0;
        for (std::size_t i = 0; i < command_armed_.size(); ++i) {
            if (command_armed_[i] && now_ns - last_command_ns_[i] > command_timeout_ns_) {
                expired.push_back(static_cast<int>(i));
            }
        }
    }

    int disabled = 0;
    for (int index : expired) {
        if (Disable_N(index)) {
            ++disabled;
        } else {
            std::cerr << "[Error] Command watchdog failed to disable motor index "
                      << index << std::endl;
        }
    }
    return disabled;
}

void BaseRobot::PrintStatus() const {
    std::lock_guard<std::mutex> lock(motor_mutex_);
    std::cout << "\n--- Robot Status | Total Motors: " << motors_.size() << " ---\n";
    for (std::size_t i = 0; i < motors_.size(); ++i) {
        const auto& motor = motors_[i];
        std::cout << "Idx: " << i << " | ID: " << motor.info.num
                  << " [" << motor.info.name << "]"
                  << " | Cmd: " << std::fixed << std::setprecision(2) << motor.send.position
                  << " | Fault: " << static_cast<int>(motor.recv.fault_message.load())
                  << " | Mode: " << static_cast<int>(motor.recv.mode)
                  << " | ReadPos: " << motor.recv.current_position_f.load()
                  << " | ReadVel: " << motor.recv.current_speed_f.load() << '\n';
    }
}

std::vector<float> BaseRobot::GetPosAll() const {
    std::lock_guard<std::mutex> lock(motor_mutex_);
    std::vector<float> positions;
    positions.reserve(motors_.size());
    for (const auto& motor : motors_) positions.push_back(motor.recv.current_position_f.load());
    return positions;
}

std::vector<float> BaseRobot::GetPosN(int count) const {
    std::lock_guard<std::mutex> lock(motor_mutex_);
    if (count <= 0) return {};
    const std::size_t size = std::min(static_cast<std::size_t>(count), motors_.size());
    std::vector<float> positions;
    positions.reserve(size);
    for (std::size_t i = 0; i < size; ++i) {
        positions.push_back(motors_[i].recv.current_position_f.load());
    }
    return positions;
}
