#include "feetech_servo_device.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>

#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace {

constexpr int kSCSPositionMax = 1023;
constexpr float kSCS0037RangeRad = 4.712389f; // 270 deg
constexpr float kRpmToRadS = 6.28318530718f / 60.0f;
constexpr int kReadTimeoutMs = 100;

constexpr uint8_t kInstRead = 0x02;
constexpr uint8_t kInstWrite = 0x03;

constexpr uint8_t kServoIdAddr = 5;
constexpr uint8_t kTorqueEnableAddr = 40;
constexpr uint8_t kGoalPositionAddr = 42;
constexpr uint8_t kEepromLockAddr = 48;
constexpr uint8_t kPresentPositionAddr = 56;
constexpr uint8_t kPresentSpeedAddr = 58;
constexpr uint8_t kPresentLoadAddr = 60;
constexpr uint8_t kPresentTemperatureAddr = 63;

bool valid_motor_index(const std::vector<Motor_CAN_Struct>* motors, int idx) {
    return motors && idx >= 0 && static_cast<std::size_t>(idx) < motors->size();
}

speed_t baud_to_termios(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 500000: return B500000;
        case 1000000: return B1000000;
        default: return B500000;
    }
}

uint8_t checksum(const uint8_t* data, std::size_t length) {
    uint8_t sum = 0;
    for (std::size_t i = 0; i < length; ++i) {
        sum = static_cast<uint8_t>(sum + data[i]);
    }
    return static_cast<uint8_t>(~sum);
}

void write_be_u16(uint8_t* out, int value) {
    const uint16_t v = static_cast<uint16_t>(std::max(0, std::min(0xFFFF, value)));
    out[0] = static_cast<uint8_t>((v >> 8) & 0xFF);
    out[1] = static_cast<uint8_t>(v & 0xFF);
}

int read_be_u16(const uint8_t* data) {
    return (static_cast<int>(data[0]) << 8) | static_cast<int>(data[1]);
}

}  // namespace

FeetechServoDevice::~FeetechServoDevice() {
    closeSerial();
}

bool FeetechServoDevice::Init(const std::string& port, int baud, int dev_idx,
                              std::vector<Motor_CAN_Struct>* data_ptr,
                              TopoMapper* mapper_ptr) {
    port_ = port;
    baud_ = baud;
    device_global_index_ = dev_idx;
    p_motors_data_ = data_ptr;
    p_mapper_ = mapper_ptr;

    is_open_ = openSerial();
    if (!is_open_) {
        std::cerr << "[Error] Failed to open Feetech serial port " << port_
                  << " at " << baud_ << " baud." << std::endl;
        return false;
    }

    std::cout << "[Info] Feetech serial ready on " << port_
              << " @ " << baud_ << std::endl;
    return true;
}

bool FeetechServoDevice::openSerial() {
    closeSerial();
    if (port_.empty()) {
        return false;
    }

    fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        std::cerr << "[Error] open(" << port_ << "): " << std::strerror(errno) << std::endl;
        return false;
    }

    termios options {};
    if (tcgetattr(fd_, &options) != 0) {
        std::cerr << "[Error] tcgetattr(" << port_ << "): " << std::strerror(errno) << std::endl;
        closeSerial();
        return false;
    }

    cfmakeraw(&options);
    const speed_t termios_baud = baud_to_termios(baud_);
    cfsetispeed(&options, termios_baud);
    cfsetospeed(&options, termios_baud);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8 | CREAD | CLOCAL;
    options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &options) != 0) {
        std::cerr << "[Error] tcsetattr(" << port_ << "): " << std::strerror(errno) << std::endl;
        closeSerial();
        return false;
    }

    tcflush(fd_, TCIOFLUSH);
    return true;
}

void FeetechServoDevice::closeSerial() {
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
    is_open_ = false;
}

int FeetechServoDevice::positionToCount(const Motor_CAN_Info_Struct& info,
                                        float position_rad) const {
    const float pos_min = (info.p_max > info.p_min) ? info.p_min : 0.0f;
    const float pos_max = (info.p_max > info.p_min) ? info.p_max : kSCS0037RangeRad;
    position_rad = std::max(pos_min, std::min(pos_max, position_rad));
    const float ratio = (position_rad - pos_min) / (pos_max - pos_min);
    return static_cast<int>(std::lround(ratio * static_cast<float>(kSCSPositionMax)));
}

float FeetechServoDevice::countToPosition(const Motor_CAN_Info_Struct& info, int count) const {
    const float pos_min = (info.p_max > info.p_min) ? info.p_min : 0.0f;
    const float pos_max = (info.p_max > info.p_min) ? info.p_max : kSCS0037RangeRad;
    count = std::max(0, std::min(kSCSPositionMax, count));
    const float ratio = static_cast<float>(count) / static_cast<float>(kSCSPositionMax);
    return pos_min + ratio * (pos_max - pos_min);
}

int FeetechServoDevice::speedToCount(float speed_rad_s) const {
    if (!std::isfinite(speed_rad_s) || speed_rad_s <= 0.0f) {
        return 0;
    }
    const float rpm = speed_rad_s / kRpmToRadS;
    const float clipped_rpm = std::max(0.0f, std::min(1023.0f, rpm));
    return static_cast<int>(std::lround(clipped_rpm));
}

bool FeetechServoDevice::writePacket(uint8_t id, uint8_t instruction,
                                     const uint8_t* params, std::size_t length) {
    if (fd_ < 0 || length > 250) {
        return false;
    }

    std::array<uint8_t, 256> packet {};
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = id;
    packet[3] = static_cast<uint8_t>(length + 2);
    packet[4] = instruction;
    if (length > 0 && params) {
        std::copy(params, params + length, packet.begin() + 5);
    }
    packet[5 + length] = checksum(packet.data() + 2, length + 3);

    tcflush(fd_, TCIFLUSH);
    const std::size_t packet_len = length + 6;
    std::size_t sent = 0;
    while (sent < packet_len) {
        const ssize_t n = write(fd_, packet.data() + sent, packet_len - sent);
        if (n > 0) {
            sent += static_cast<std::size_t>(n);
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
            continue;
        }
        return false;
    }
    tcdrain(fd_);
    return true;
}

bool FeetechServoDevice::readStatusPacket(uint8_t expected_id, uint8_t* error,
                                          uint8_t* data, std::size_t length) {
    if (fd_ < 0 || length > 250) {
        return false;
    }

    std::array<uint8_t, 256> packet {};
    const std::size_t expected_len = length + 6;
    std::size_t got = 0;

    while (got < expected_len) {
        fd_set read_set;
        FD_ZERO(&read_set);
        FD_SET(fd_, &read_set);

        timeval timeout {};
        timeout.tv_sec = 0;
        timeout.tv_usec = kReadTimeoutMs * 1000;

        const int ready = select(fd_ + 1, &read_set, nullptr, nullptr, &timeout);
        if (ready <= 0) {
            return false;
        }

        const ssize_t n = read(fd_, packet.data() + got, expected_len - got);
        if (n > 0) {
            got += static_cast<std::size_t>(n);
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
            continue;
        }
        return false;
    }

    if (packet[0] != 0xFF || packet[1] != 0xFF || packet[2] != expected_id ||
        packet[3] != static_cast<uint8_t>(length + 2)) {
        return false;
    }

    if (checksum(packet.data() + 2, length + 3) != packet[expected_len - 1]) {
        return false;
    }

    if (error) {
        *error = packet[4];
    }
    if (length > 0 && data) {
        std::copy(packet.begin() + 5, packet.begin() + 5 + static_cast<long>(length), data);
    }
    return true;
}

void FeetechServoDevice::drainInput(int timeout_ms) {
    if (fd_ < 0) {
        return;
    }

    std::array<uint8_t, 64> scratch {};
    while (true) {
        fd_set read_set;
        FD_ZERO(&read_set);
        FD_SET(fd_, &read_set);

        timeval timeout {};
        timeout.tv_sec = 0;
        timeout.tv_usec = timeout_ms * 1000;

        const int ready = select(fd_ + 1, &read_set, nullptr, nullptr, &timeout);
        if (ready <= 0) {
            return;
        }

        const ssize_t n = read(fd_, scratch.data(), scratch.size());
        if (n <= 0 && !(errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
            return;
        }
    }
}

bool FeetechServoDevice::writeRegister(int id, uint8_t address,
                                       const uint8_t* data, std::size_t length) {
    if (id < 0 || id > 253 || length > 248) {
        return false;
    }

    std::array<uint8_t, 249> params {};
    params[0] = address;
    if (length > 0 && data) {
        std::copy(data, data + length, params.begin() + 1);
    }

    if (!writePacket(static_cast<uint8_t>(id), kInstWrite, params.data(), length + 1)) {
        return false;
    }

    // Some Feetech servos/controllers do not ACK write commands depending on
    // their status-return-level setting. Reads below still verify live comms.
    drainInput(2);
    return true;
}

bool FeetechServoDevice::readRegister(int id, uint8_t address,
                                      uint8_t* data, std::size_t length) {
    if (id < 0 || id > 253 || length == 0 || length > 250 || !data) {
        return false;
    }

    const uint8_t params[2] = {address, static_cast<uint8_t>(length)};
    if (!writePacket(static_cast<uint8_t>(id), kInstRead, params, sizeof(params))) {
        return false;
    }

    uint8_t error = 0;
    return readStatusPacket(static_cast<uint8_t>(id), &error, data, length) && error == 0;
}

bool FeetechServoDevice::writeRegisterByte(int id, uint8_t address, uint8_t value) {
    return writeRegister(id, address, &value, 1);
}

int FeetechServoDevice::readRegisterByte(int id, uint8_t address) {
    uint8_t value = 0;
    if (!readRegister(id, address, &value, 1)) {
        return -1;
    }
    return value;
}

int FeetechServoDevice::readRegisterWord(int id, uint8_t address,
                                         bool signed_value, uint8_t sign_bit) {
    uint8_t data[2] = {0, 0};
    if (!readRegister(id, address, data, sizeof(data))) {
        return -1;
    }

    int value = read_be_u16(data);
    if (signed_value && (value & (1 << sign_bit))) {
        value = -(value & ~(1 << sign_bit));
    }
    return value;
}

bool FeetechServoDevice::writePosition(int id, int position, int speed) {
    uint8_t data[6] = {0, 0, 0, 0, 0, 0};
    write_be_u16(data + 0, position);
    write_be_u16(data + 2, 0);
    write_be_u16(data + 4, speed);
    return writeRegister(id, kGoalPositionAddr, data, sizeof(data));
}

bool FeetechServoDevice::enableTorque(int id, bool enable) {
    return writeRegisterByte(id, kTorqueEnableAddr, enable ? 1 : 0);
}

bool FeetechServoDevice::SetServoId(int old_id, int new_id) {
    if (!is_open_ || old_id < 0 || old_id > 253 || new_id < 0 || new_id > 253 ||
        old_id == 254 || new_id == 254) {
        return false;
    }

    if (!writeRegisterByte(old_id, kEepromLockAddr, 0)) {
        std::cerr << "[Error] Failed to unlock Feetech EEPROM: id=" << old_id << std::endl;
        return false;
    }

    if (!writeRegisterByte(old_id, kServoIdAddr, static_cast<uint8_t>(new_id))) {
        std::cerr << "[Error] Failed to write Feetech ID: old_id=" << old_id
                  << ", new_id=" << new_id << std::endl;
        return false;
    }

    if (!writeRegisterByte(new_id, kEepromLockAddr, 1)) {
        std::cerr << "[Warn] Feetech ID was written, but EEPROM re-lock did not confirm: new_id="
                  << new_id << std::endl;
    }
    return true;
}

void FeetechServoDevice::EnableMotor(int& motor_index) {
    if (!is_open_ || !valid_motor_index(p_motors_data_, motor_index)) return;
    const auto& info = (*p_motors_data_)[motor_index].info;
    if (enableTorque(info.canid, true)) {
        (*p_motors_data_)[motor_index].recv.motor_state = 1;
    } else {
        (*p_motors_data_)[motor_index].recv.fault_message = 1;
    }
}

void FeetechServoDevice::DisableMotor(int& motor_index) {
    if (!is_open_ || !valid_motor_index(p_motors_data_, motor_index)) return;
    const auto& info = (*p_motors_data_)[motor_index].info;
    if (enableTorque(info.canid, false)) {
        (*p_motors_data_)[motor_index].recv.motor_state = 0;
    } else {
        (*p_motors_data_)[motor_index].recv.fault_message = 1;
    }
}

void FeetechServoDevice::ClearError(int& motor_index) {
    if (!valid_motor_index(p_motors_data_, motor_index)) return;
    (*p_motors_data_)[motor_index].recv.fault_message = 0;
}

void FeetechServoDevice::SetZero(int& motor_index) {
    if (!valid_motor_index(p_motors_data_, motor_index)) return;
    std::cout << "[Info] Feetech SetZero is software-neutral; keep TOML pos_min/pos_max as travel limits."
              << std::endl;
}

void FeetechServoDevice::SetMode(int& motor_index, int mode) {
    if (!valid_motor_index(p_motors_data_, motor_index)) return;
    (*p_motors_data_)[motor_index].send.mode = static_cast<uint8_t>(std::max(0, mode));
}

void FeetechServoDevice::SendCommand(int& motor_index) {
    if (!is_open_ || !valid_motor_index(p_motors_data_, motor_index)) return;
    auto& motor = (*p_motors_data_)[motor_index];
    const int id = motor.info.canid;
    const int position = positionToCount(motor.info, motor.send.position);
    const int speed = speedToCount(motor.send.speed);

    if (!writePosition(id, position, speed)) {
        motor.recv.fault_message = 1;
        std::cerr << "[Warn] Feetech WritePos failed: id=" << id << std::endl;
    }
}

void FeetechServoDevice::QueryPos(int& motor_index) {
    if (!is_open_ || !valid_motor_index(p_motors_data_, motor_index)) return;
    auto& motor = (*p_motors_data_)[motor_index];
    const int id = motor.info.canid;

    const int pos = readRegisterWord(id, kPresentPositionAddr, false, 0);
    if (pos >= 0) {
        motor.recv.current_position_f.store(countToPosition(motor.info, pos));
        motor.recv.motor_id = static_cast<uint8_t>(id);
    }

    const int speed = readRegisterWord(id, kPresentSpeedAddr, true, 15);
    if (speed != -1) {
        motor.recv.current_speed_f.store(static_cast<float>(speed) * kRpmToRadS);
    }

    const int load = readRegisterWord(id, kPresentLoadAddr, true, 10);
    if (load != -1) {
        motor.recv.current_torque_f.store(static_cast<float>(load) / 1000.0f);
    }

    const int temp = readRegisterByte(id, kPresentTemperatureAddr);
    if (temp != -1) {
        motor.recv.current_temp_f.store(static_cast<float>(temp));
    }
}

void FeetechServoDevice::QueryVersion(int& motor_index) {
    (void)motor_index;
}
