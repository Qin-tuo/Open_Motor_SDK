#include "device.hpp"
#include <algorithm>
#include <cmath>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <thread>

// =========================================================
//  Constants
// =========================================================
static const float RAD_TO_DEG = 57.2957795f;
static const float DEG_TO_RAD = 0.017453293f;

static const float P_RIOT = 6.28318530718f / 65536.0f;
static const float V_ROIT = 57.29578f;
static const float IQ_ROIT = 0.008056640625f;

static const float LK_CURRENT_SEND_FACTOR = 100.0f;
static const float LK_SPEED_SEND_FACTOR = 57.29578f * 100.0f;
static const float LK_POS_SEND_FACTOR = 57.29578f * 100.0f;

static const float C620_POS_TO_RAD = 6.28318530718f / 8191.0f;
static const float C620_RPM_TO_RAD_S = 6.28318530718f / 60.0f;

static bool tryBringUpInterface(const std::string& iface, int bitrate = 1000000) {
    std::string cmd = "ip link set " + iface + " down 2>/dev/null && ";
    cmd += "ip link set " + iface + " type can bitrate " + std::to_string(bitrate) + " 2>/dev/null && ";
    cmd += "ip link set " + iface + " up 2>/dev/null";
    int r = std::system(cmd.c_str());
    return (r == 0);
}

static bool has_cap_net_admin() {
    if (geteuid() == 0) return true;

    std::ifstream f("/proc/self/status");
    if (!f.is_open()) return false;

    std::string line;
    while (std::getline(f, line)) {
        if (line.rfind("CapEff:", 0) == 0) {
            std::istringstream iss(line);
            std::string key, hexval;
            if (iss >> key >> hexval) {
                unsigned long long val = 0;
                try {
                    val = std::stoull(hexval, nullptr, 16);
                } catch (...) {
                    return false;
                }
                const unsigned int CAP_NET_ADMIN = 12;
                if (val & (1ULL << CAP_NET_ADMIN)) return true;
            }
            break;
        }
    }
    return false;
}

DeviceX::~DeviceX() {
    is_running = false;
    // Close socket first to unblock rx thread's blocking read().
    const int fd = socket_fd;
    socket_fd = -1;
    if (fd >= 0) {
        close(fd);
    }
    if (rx_thread.joinable()) {
        rx_thread.join();
    }
}

bool DeviceX::openSocket(const std::string& iface) {
    socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd < 0) {
        std::perror("socket");
        return false;
    }

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
    if (ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0) {
        std::perror("ioctl");
        close(socket_fd);
        socket_fd = -1;
        return false;
    }

    struct ifreq ifr2;
    std::memset(&ifr2, 0, sizeof(ifr2));
    std::strncpy(ifr2.ifr_name, iface.c_str(), IFNAMSIZ - 1);
    if (ioctl(socket_fd, SIOCGIFFLAGS, &ifr2) < 0) {
        std::perror("ioctl SIOCGIFFLAGS");
        close(socket_fd);
        socket_fd = -1;
        return false;
    }

    if (!(ifr2.ifr_flags & IFF_UP)) {
        std::cerr << "[Warn] Interface " << iface << " is down." << std::endl;

        if (!has_cap_net_admin()) {
            std::cerr << "[Error] Process lacks CAP_NET_ADMIN (or is not root). Cannot bring up interface programmatically.\n"
                      << "  - Run as root: sudo <your_app>\n"
                      << "  - Or grant capabilities: sudo setcap 'cap_net_raw,cap_net_admin+ep' <your_app>\n";
            close(socket_fd);
            socket_fd = -1;
            return false;
        }

        std::cerr << "[Info] Attempting to bring up " << iface << " (process has CAP_NET_ADMIN)..." << std::endl;
        bool ok = false;
        for (int attempt = 0; attempt < 3; ++attempt) {
            if (tryBringUpInterface(iface)) {
                if (ioctl(socket_fd, SIOCGIFFLAGS, &ifr2) == 0 && (ifr2.ifr_flags & IFF_UP)) {
                    ok = true;
                    break;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        if (!ok) {
            std::cerr << "[Error] Failed to bring up interface " << iface << "." << std::endl;
            close(socket_fd);
            socket_fd = -1;
            return false;
        }
        std::cout << "[Info] Interface " << iface << " is now up." << std::endl;
    }

    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::perror("bind");
        close(socket_fd);
        socket_fd = -1;
        return false;
    }

    // Prevent indefinite blocking on read/write when bus/interface is unhealthy.
    timeval rcv_timeout {};
    rcv_timeout.tv_sec = 0;
    rcv_timeout.tv_usec = 100000; // 100 ms
    if (setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &rcv_timeout, sizeof(rcv_timeout)) < 0) {
        std::perror("setsockopt SO_RCVTIMEO");
    }

    timeval snd_timeout {};
    snd_timeout.tv_sec = 0;
    snd_timeout.tv_usec = 100000; // 100 ms
    if (setsockopt(socket_fd, SOL_SOCKET, SO_SNDTIMEO, &snd_timeout, sizeof(snd_timeout)) < 0) {
        std::perror("setsockopt SO_SNDTIMEO");
    }

    int enable_fd = 1;
    if (setsockopt(socket_fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd, sizeof(enable_fd)) < 0) {
        std::perror("setsockopt CAN_RAW_FD_FRAMES");
    }

    return true;
}

bool DeviceX::Init(const std::string& iface, int dev_idx,
                   std::vector<Motor_CAN_Struct>* data_ptr, TopoMapper* mapper_ptr) {
    iface_name = iface;
    device_global_index = dev_idx;
    p_motors_data = data_ptr;
    p_mapper = mapper_ptr;

    if (!openSocket(iface_name)) {
        std::cerr << "[Error] Failed to open socketcan interface: " << iface_name << std::endl;
        return false;
    }

    is_running = true;
    rx_thread = std::thread(&DeviceX::ReceiveLoop, this);
    std::cout << "[Info] SocketCAN ready on " << iface_name << std::endl;
    return true;
}

void DeviceX::sendExtendedFrame(uint32_t type, uint16_t data_area, uint8_t motor_id, const uint8_t* data) {
    if (socket_fd < 0) return;

    struct can_frame frame {};
    frame.can_id = ((type & 0x1F) << 24) | ((data_area & 0xFFFF) << 8) | (motor_id & 0xFF);
    frame.can_id |= CAN_EFF_FLAG;
    frame.can_dlc = 8;
    std::memcpy(frame.data, data, 8);

    int n = send(socket_fd, &frame, sizeof(frame), MSG_DONTWAIT);
    if (n != static_cast<int>(sizeof(frame))) {
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == ETIMEDOUT || errno == EINTR)) {
            return;
        }
        if (errno == ENETDOWN) {
            std::cerr << "sendExtendedFrame: Network is down on " << iface_name << ". Closing socket until interface is up." << std::endl;
            close(socket_fd);
            socket_fd = -1;
        } else {
            std::perror("sendExtendedFrame");
        }
    }
}

void DeviceX::sendStandardFrame(uint32_t can_id, const uint8_t* data, uint8_t dlc) {
    if (socket_fd < 0) return;

    struct can_frame frame {};
    frame.can_id = (can_id & CAN_SFF_MASK);
    frame.can_dlc = dlc;
    std::memcpy(frame.data, data, dlc);

    int n = send(socket_fd, &frame, sizeof(frame), MSG_DONTWAIT);
    if (n != static_cast<int>(sizeof(frame))) {
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == ETIMEDOUT || errno == EINTR)) {
            return;
        }
        if (errno == ENETDOWN) {
            std::cerr << "sendStandardFrame: Network is down on " << iface_name << ". Closing socket until interface is up." << std::endl;
            close(socket_fd);
            socket_fd = -1;
        } else {
            std::perror("sendStandardFrame");
        }
    }
}

void DeviceX::sendRawFrame(uint8_t chan, uint32_t type, uint16_t data_area, uint8_t motor_id, uint8_t* data) {
    (void)chan;
    sendExtendedFrame(type, data_area, motor_id, data);
}

// =========================================================
//  DeviceX public dispatch
// =========================================================

void DeviceX::EnableMotor(int& motor_index) {
    if (!p_motors_data || motor_index < 0 || motor_index >= (int)p_motors_data->size()) return;

    int type = (*p_motors_data)[motor_index].info.api_type;

    if (type == 1) EnableMotor_Type1(motor_index);
    else if (type == 2) EnableMotor_Type2(motor_index);
    else if (type == 3) EnableMotor_Type3(motor_index);
    else if (type == 4) EnableMotor_Type4(motor_index);
}

void DeviceX::DisableMotor(int& motor_index) {
    if (!p_motors_data || motor_index < 0 || motor_index >= (int)p_motors_data->size()) return;

    int type = (*p_motors_data)[motor_index].info.api_type;

    if (type == 1) DisableMotor_Type1(motor_index);
    else if (type == 2) DisableMotor_Type2(motor_index);
    else if (type == 3) DisableMotor_Type3(motor_index);
    else if (type == 4) DisableMotor_Type4(motor_index);
}

void DeviceX::ClearError(int& motor_index) {
    if (!p_motors_data || motor_index < 0 || motor_index >= (int)p_motors_data->size()) return;

    int type = (*p_motors_data)[motor_index].info.api_type;

    if (type == 1) ClearError_Type1(motor_index);
    else if (type == 2) ClearError_Type2(motor_index);
    else if (type == 3) ClearError_Type3(motor_index);
    else if (type == 4) ClearError_Type4(motor_index);
}

void DeviceX::SetZero(int& motor_index) {
    if (!p_motors_data || motor_index < 0 || motor_index >= (int)p_motors_data->size()) return;

    int type = (*p_motors_data)[motor_index].info.api_type;

    if (type == 1) SetZero_Type1(motor_index);
    else if (type == 2) SetZero_Type2(motor_index);
    else if (type == 3) SetZero_Type3(motor_index);
    else if (type == 4) SetZero_Type4(motor_index);
}

void DeviceX::SetMode(int& motor_index, int mode) {
    if (!p_motors_data || motor_index < 0 || motor_index >= (int)p_motors_data->size()) return;

    int type = (*p_motors_data)[motor_index].info.api_type;

    if (type == 1) SetMode_Type1(motor_index, mode);
    else if (type == 2) SetMode_Type2(motor_index, mode);
    else if (type == 3) SetMode_Type3(motor_index, mode);
    else if (type == 4) SetMode_Type4(motor_index, mode);
}

void DeviceX::SendCommand(int& motor_index) {
    if (!p_motors_data || motor_index < 0 || motor_index >= (int)p_motors_data->size()) {
        std::cerr << "[ERROR] SendCommand Failed! Index Out of Range or Null Ptr." << std::endl;
        return;
    }

    int type = (*p_motors_data)[motor_index].info.api_type;

    if (type == 1) SendCommand_Type1(motor_index);
    else if (type == 2) SendCommand_Type2(motor_index);
    else if (type == 3) SendCommand_Type3(motor_index);
    else if (type == 4) SendCommand_Type4(motor_index);
}

void DeviceX::QueryPos(int& motor_index) {
    if (!p_motors_data || motor_index < 0 || motor_index >= (int)p_motors_data->size()) return;

    int type = (*p_motors_data)[motor_index].info.api_type;

    if (type == 1) QueryPos_Type1(motor_index);
    else if (type == 2) QueryPos_Type2(motor_index);
    else if (type == 3) QueryPos_Type3(motor_index);
    else if (type == 4) QueryPos_Type4(motor_index);
}

// =========================================================
//  Type 1 (LimX)
// =========================================================

void DeviceX::EnableMotor_Type1(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    uint8_t data[8] = {0};
    sendExtendedFrame(3, 0xFD, info.canid, data);
}

void DeviceX::DisableMotor_Type1(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    uint8_t data[8] = {0};
    sendExtendedFrame(4, 0xFD, info.canid, data);
}

void DeviceX::ClearError_Type1(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    uint8_t data[8] = {0};
    data[0] = 1;
    sendExtendedFrame(4, 0xFD, info.canid, data);
}

void DeviceX::SetZero_Type1(int& motor_index) {
    // 灵足协议:
    // 1) 通信类型=6, Byte0=1: 设置当前位置为机械零位（RAM 生效）
    // 2) 通信类型=22: 电机数据保存帧（写入掉电保持）
    const auto& info = (*p_motors_data)[motor_index].info;
    uint8_t data[8] = {0};

    // Step1: set zero
    data[0] = 1;
    sendExtendedFrame(6, 0xFD, info.canid, data);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    // Step2: save parameters to non-volatile storage
    std::memset(data, 0, sizeof(data));
    sendExtendedFrame(22, 0xFD, info.canid, data);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    // Some controllers may drop the first save frame under bus load.
    sendExtendedFrame(22, 0xFD, info.canid, data);
}

void DeviceX::SetMode_Type1(int& motor_index, int mode) {
    const auto& info = (*p_motors_data)[motor_index].info;

    uint8_t data[8] = {0};
    uint16_t index = 0x7005;
    uint8_t mode_val = static_cast<uint8_t>(mode);

    std::memcpy(&data[0], &index, 2);
    std::memcpy(&data[4], &mode_val, 1);

    sendExtendedFrame(18, 0xFD, info.canid, data);
}

void DeviceX::SendCommand_Type1(int& motor_index) {
    const Motor_CAN_Struct& motor = (*p_motors_data)[motor_index];
    const auto& cmd = motor.send;
    const auto& info = motor.info;

    uint8_t data[8] = {0};

    uint16_t t_int = float_to_uint(cmd.torque, info.t_min, info.t_max, 16);
    int p_int = float_to_uint(cmd.position, info.p_min, info.p_max, 16);
    int v_int = float_to_uint(cmd.speed, info.v_min, info.v_max, 16);
    int kp_int = float_to_uint(cmd.kp, info.kp_min, info.kp_max, 16);
    int kd_int = float_to_uint(cmd.kd, info.kd_min, info.kd_max, 16);

    data[0] = p_int >> 8;
    data[1] = p_int & 0xFF;
    data[2] = v_int >> 8;
    data[3] = v_int & 0xFF;
    data[4] = kp_int >> 8;
    data[5] = kp_int & 0xFF;
    data[6] = kd_int >> 8;
    data[7] = kd_int & 0xFF;

    sendExtendedFrame(1, t_int, info.canid, data);
}

void DeviceX::QueryPos_Type1(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;

    uint8_t data[8] = {0};
    uint16_t index = 0x7019;
    std::memcpy(&data[0], &index, 2);

    sendExtendedFrame(17, 0xFD, info.canid, data);
}

// =========================================================
//  Type 2 (LK)
// =========================================================

void DeviceX::EnableMotor_Type2(int& motor_index) {
    const Motor_CAN_Struct& motor = (*p_motors_data)[motor_index];

    uint8_t data[8] = {0};
    data[0] = 0x88;
    sendStandardFrame(0x140 + motor.info.canid, data);
}

void DeviceX::DisableMotor_Type2(int& motor_index) {
    const Motor_CAN_Struct& motor = (*p_motors_data)[motor_index];

    uint8_t data[8] = {0};
    data[0] = 0x80;
    sendStandardFrame(0x140 + motor.info.canid, data);
}

void DeviceX::ClearError_Type2(int& motor_index) {
    DisableMotor_Type2(motor_index);
}

void DeviceX::SetZero_Type2(int& motor_index) {
    (void)motor_index;
}

void DeviceX::SetMode_Type2(int& motor_index, int mode) {
    (*p_motors_data)[motor_index].send.mode = static_cast<uint8_t>(mode);
}

void DeviceX::SendCommand_Type2(int& motor_index) {
    const Motor_CAN_Struct& motor = (*p_motors_data)[motor_index];
    const int id = motor.info.canid;
    const uint8_t current_mode = motor.send.mode;

    uint8_t data[8] = {0};

    if (current_mode == 1) {
        float current_p = motor.recv.current_position_f.load();
        float current_v = motor.recv.current_speed_f.load();

        float kp = motor.send.kp;
        float kd = motor.send.kd;
        float target_p = motor.send.position;
        float target_v = motor.send.speed;
        float t_ff = motor.send.torque;

        float iq_f = kp * (target_p - current_p) + kd * (target_v - current_v) + t_ff;
        int16_t iqControl = static_cast<int16_t>(iq_f * LK_CURRENT_SEND_FACTOR);

        if (iqControl > 2048) iqControl = 2048;
        if (iqControl < -2048) iqControl = -2048;

        data[0] = 0xA1;
        std::memcpy(&data[4], &iqControl, 2);
    } else if (current_mode == 2) {
        float target_rad = motor.send.position;
        int32_t angleControl = static_cast<int32_t>(target_rad * LK_POS_SEND_FACTOR);

        data[0] = 0xA3;
        std::memcpy(&data[4], &angleControl, 4);
    } else if (current_mode == 3) {
        float target_vel_rad = motor.send.speed;
        int32_t speedControl = static_cast<int32_t>(target_vel_rad * LK_SPEED_SEND_FACTOR);
        int16_t iqLimit = 2000;

        data[0] = 0xA2;
        std::memcpy(&data[2], &iqLimit, 2);
        std::memcpy(&data[4], &speedControl, 4);
    } else {
        data[0] = 0x9C;
    }

    sendStandardFrame(0x140 + id, data);
}

void DeviceX::QueryPos_Type2(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    uint8_t data[8] = {0};
    data[0] = 0x9C;
    sendStandardFrame(0x140 + info.canid, data);
}

// =========================================================
//  Type 3 (DM)
// =========================================================

void DeviceX::EnableMotor_Type3(int& motor_index) {
    const auto& motor = (*p_motors_data)[motor_index];

    uint16_t canid = static_cast<uint16_t>(motor.info.canid);
    uint8_t mode = motor.send.mode;
    uint32_t can_id = (mode == 1) ? (0x100 + canid) : ((mode == 2) ? (0x200 + canid) : canid);

    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    sendStandardFrame(can_id, data);
}

void DeviceX::DisableMotor_Type3(int& motor_index) {
    const auto& motor = (*p_motors_data)[motor_index];

    uint16_t canid = static_cast<uint16_t>(motor.info.canid);
    uint8_t mode = motor.send.mode;
    uint32_t can_id = (mode == 1) ? (0x100 + canid) : ((mode == 2) ? (0x200 + canid) : canid);

    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    sendStandardFrame(can_id, data);
}

void DeviceX::ClearError_Type3(int& motor_index) {
    const auto& motor = (*p_motors_data)[motor_index];

    uint16_t canid = static_cast<uint16_t>(motor.info.canid);
    uint8_t mode = motor.send.mode;
    uint32_t can_id = (mode == 1) ? (0x100 + canid) : ((mode == 2) ? (0x200 + canid) : canid);

    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
    sendStandardFrame(can_id, data);
}

void DeviceX::SetZero_Type3(int& motor_index) {
    const auto& motor = (*p_motors_data)[motor_index];

    uint16_t canid = static_cast<uint16_t>(motor.info.canid);
    uint8_t mode = motor.send.mode;
    uint32_t can_id = (mode == 1) ? (0x100 + canid) : ((mode == 2) ? (0x200 + canid) : canid);

    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    sendStandardFrame(can_id, data);
}

void DeviceX::SetMode_Type3(int& motor_index, int mode) {
    if (mode < 0) mode = 0;
    if (mode > 3) mode = 3;
    (*p_motors_data)[motor_index].send.mode = static_cast<uint8_t>(mode);
}

void DeviceX::SendCommand_Type3(int& motor_index) {
    const Motor_CAN_Struct& motor = (*p_motors_data)[motor_index];
    const auto& cmd = motor.send;
    const auto& info = motor.info;

    uint8_t data[8] = {0};

    const float kp_min = (info.kp_max > info.kp_min) ? info.kp_min : 0.0f;
    const float kp_max = (info.kp_max > info.kp_min) ? info.kp_max : 500.0f;
    const float kd_min = (info.kd_max > info.kd_min) ? info.kd_min : 0.0f;
    const float kd_max = (info.kd_max > info.kd_min) ? info.kd_max : 5.0f;

    if (cmd.mode == 0 || cmd.mode == 3) {
        const bool torque_only = (cmd.mode == 3);

        uint16_t p_int = (uint16_t)float_to_uint(cmd.position, info.p_min, info.p_max, 16);
        uint16_t v_int = (uint16_t)float_to_uint(cmd.speed, info.v_min, info.v_max, 12);
        uint16_t kp_int = (uint16_t)float_to_uint(torque_only ? 0.0f : cmd.kp, kp_min, kp_max, 12);
        uint16_t kd_int = (uint16_t)float_to_uint(torque_only ? 0.0f : cmd.kd, kd_min, kd_max, 12);
        uint16_t t_int = (uint16_t)float_to_uint(cmd.torque, info.t_min, info.t_max, 12);

        uint32_t can_id = (uint16_t)info.canid;
        data[0] = (uint8_t)(p_int >> 8);
        data[1] = (uint8_t)(p_int & 0xFF);
        data[2] = (uint8_t)(v_int >> 4);
        data[3] = (uint8_t)(((v_int & 0x0F) << 4) | (kp_int >> 8));
        data[4] = (uint8_t)(kp_int & 0xFF);
        data[5] = (uint8_t)(kd_int >> 4);
        data[6] = (uint8_t)(((kd_int & 0x0F) << 4) | (t_int >> 8));
        data[7] = (uint8_t)(t_int & 0xFF);
        sendStandardFrame(can_id, data);
    } else if (cmd.mode == 1) {
        float p = cmd.position;
        float v = cmd.speed;
        uint32_t can_id = 0x100 + (uint16_t)info.canid;
        std::memcpy(&data[0], &p, 4);
        std::memcpy(&data[4], &v, 4);
        sendStandardFrame(can_id, data);
    } else if (cmd.mode == 2) {
        float v = cmd.speed;
        uint32_t can_id = 0x200 + (uint16_t)info.canid;
        std::memcpy(&data[0], &v, 4);
        sendStandardFrame(can_id, data, 4);
    }
}

void DeviceX::QueryPos_Type3(int& motor_index) {
    const auto& motor = (*p_motors_data)[motor_index];

    uint8_t data[8] = {0};
    const uint16_t canid = static_cast<uint16_t>(motor.info.canid);
    data[0] = static_cast<uint8_t>(canid & 0xFF);
    data[1] = static_cast<uint8_t>((canid >> 8) & 0xFF);
    data[2] = 0xCC;

    sendStandardFrame(0x7FF, data);
}

// =========================================================
//  Type 4 (RoboMaster C620)
// =========================================================

void DeviceX::EnableMotor_Type4(int& motor_index) {
    (*p_motors_data)[motor_index].send.mode = 0;
}

void DeviceX::DisableMotor_Type4(int& motor_index) {
    const auto& motor = (*p_motors_data)[motor_index];
    const int id = motor.info.canid;
    if (id < 1 || id > 8) return;

    uint32_t base_id = (id <= 4) ? 0x200 : 0x1FF;
    uint8_t data[8] = {0};
    sendStandardFrame(base_id, data);
}

void DeviceX::ClearError_Type4(int& motor_index) {
    (void)motor_index;
}

void DeviceX::SetZero_Type4(int& motor_index) {
    (void)motor_index;
}

void DeviceX::SetMode_Type4(int& motor_index, int mode) {
    (*p_motors_data)[motor_index].send.mode = static_cast<uint8_t>(mode);
}

void DeviceX::SendCommand_Type4(int& motor_index) {
    const auto& motor = (*p_motors_data)[motor_index];
    const int id = motor.info.canid;
    if (id < 1 || id > 8) return;

    const uint32_t base_id = (id <= 4) ? 0x200 : 0x1FF;

    uint8_t data[8] = {0};

    for (const auto& m : *p_motors_data) {
        if (m.info.api_type != 4) continue;
        if (m.info.device_index != motor.info.device_index) continue;
        if (m.info.chan != motor.info.chan) continue;

        const int canid = m.info.canid;
        if (canid < 1 || canid > 8) continue;
        if ((base_id == 0x200 && canid > 4) || (base_id == 0x1FF && canid < 5)) continue;

        float torque_cmd = m.send.torque;
        int16_t iq_cmd = 0;

        if (m.info.t_max > m.info.t_min) {
            const float clipped = std::max(m.info.t_min, std::min(m.info.t_max, torque_cmd));
            const float ratio = (clipped - m.info.t_min) / (m.info.t_max - m.info.t_min);
            float raw = ratio * (16384.0f - (-16384.0f)) + (-16384.0f);
            raw = std::max(-16384.0f, std::min(16384.0f, raw));
            iq_cmd = (int16_t)std::lround(raw);
        } else {
            float raw = std::max(-16384.0f, std::min(16384.0f, torque_cmd));
            iq_cmd = (int16_t)std::lround(raw);
        }

        const int slot = (base_id == 0x200) ? (canid - 1) : (canid - 5);
        data[slot * 2 + 0] = (uint8_t)((iq_cmd >> 8) & 0xFF);
        data[slot * 2 + 1] = (uint8_t)(iq_cmd & 0xFF);
    }

    sendStandardFrame(base_id, data);
}

void DeviceX::QueryPos_Type4(int& motor_index) {
    (void)motor_index;
}

// =========================================================
//  Receive Loop
// =========================================================

void DeviceX::ReceiveLoop() {
    while (is_running) {
        struct can_frame frame {};
        int n = read(socket_fd, &frame, sizeof(frame));
        if (n < 0) {
            if (errno == EINTR) continue;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        if (n != static_cast<int>(sizeof(frame))) {
            continue;
        }

        int parsed_motor_id = -1;
        uint8_t rx_channel = 0;

        if (frame.can_id & CAN_EFF_FLAG) {
            uint32_t canID = frame.can_id & CAN_EFF_MASK;
            parsed_motor_id = (canID >> 8) & 0xFF;
        } else {
            uint32_t canID = frame.can_id & CAN_SFF_MASK;
            if (canID >= 0x140 && canID < 0x160) {
                parsed_motor_id = canID - 0x140;
            } else if (canID >= 0x180 && canID < 0x1A0) {
                parsed_motor_id = canID - 0x180;
            } else if (canID >= 0x201 && canID <= 0x208) {
                parsed_motor_id = canID - 0x200;
            } else {
                parsed_motor_id = frame.data[0] & 0x0F;
            }
        }

        if (parsed_motor_id < 0) continue;

        int g_idx = p_mapper->get_id({(uint)device_global_index, (uint)rx_channel, (uint)parsed_motor_id});

        if (g_idx < 0 || g_idx >= (int)p_motors_data->size()) continue;

        Motor_CAN_Struct& motor = (*p_motors_data)[g_idx];

        if (motor.info.api_type == 1 && (frame.can_id & CAN_EFF_FLAG)) {
            uint32_t canID = frame.can_id & CAN_EFF_MASK;
            uint8_t type_field = (canID >> 24) & 0x1F;

            if (type_field == 2) {
                uint16_t p_int = (frame.data[0] << 8) | frame.data[1];
                uint16_t v_int = (frame.data[2] << 8) | frame.data[3];
                uint16_t t_int = (frame.data[4] << 8) | frame.data[5];
                uint16_t temp_int = (frame.data[6] << 8) | frame.data[7];

                motor.recv.current_position_f.store(uint_to_float(p_int, motor.info.p_min, motor.info.p_max, 16));
                motor.recv.current_speed_f.store(uint_to_float(v_int, motor.info.v_min, motor.info.v_max, 16));
                motor.recv.current_torque_f.store(uint_to_float(t_int, motor.info.t_min, motor.info.t_max, 16));
                motor.recv.current_temp_f.store((float)temp_int / 10.0f);

                motor.recv.fault_message = (canID >> 16) & 0x3F;
                motor.recv.mode = (canID >> 22) & 0x03;
            } else if (type_field == 17) {
                if (frame.data[0] == 0x19 && frame.data[1] == 0x70) {
                    float temp_val = 0.0f;
                    std::memcpy(&temp_val, &frame.data[4], 4);
                    motor.recv.current_position_f.store(temp_val);
                }
            } else if (type_field == 21) {
                uint32_t fault_word = 0;
                std::memcpy(&fault_word, &frame.data[0], 4);
                motor.recv.fault_message = static_cast<uint8_t>(fault_word);
            }
        } else if (motor.info.api_type == 2 && !(frame.can_id & CAN_EFF_FLAG)) {
            uint8_t status = frame.data[0];
            if (status == 0x9C || status == 0xA1 || status == 0xA2 || status == 0xA3) {
                int8_t temp = static_cast<int8_t>(frame.data[1]);
                int16_t iq_raw = static_cast<int16_t>((frame.data[3] << 8) | frame.data[2]);
                int16_t spd_raw = static_cast<int16_t>((frame.data[5] << 8) | frame.data[4]);
                uint16_t pos_raw = static_cast<uint16_t>((frame.data[7] << 8) | frame.data[6]);

                motor.recv.current_temp_f.store((float)temp);
                motor.recv.current_iq_f.store((float)iq_raw * IQ_ROIT);
                motor.recv.current_speed_f.store((float)spd_raw / V_ROIT);
                motor.recv.current_position_f.store((float)pos_raw * P_RIOT);

                motor.recv.motor_id = static_cast<uint8_t>(parsed_motor_id);
                motor.recv.mode = status;
                motor.recv.fault_message = 0;
            }
        } else if (motor.info.api_type == 3) {
            uint8_t id_and_err = frame.data[0];
            uint8_t err = (id_and_err >> 4) & 0x0F;
            uint8_t motor_id = id_and_err & 0x0F;

            uint16_t p_int = (uint16_t)((frame.data[1] << 8) | frame.data[2]);
            uint16_t v_int = (uint16_t)((frame.data[3] << 4) | (frame.data[4] >> 4));
            uint16_t t_int = (uint16_t)(((frame.data[4] & 0x0F) << 8) | frame.data[5]);

            motor.recv.current_position_f.store(uint_to_float((int)p_int, motor.info.p_min, motor.info.p_max, 16));
            motor.recv.current_speed_f.store(uint_to_float((int)v_int, motor.info.v_min, motor.info.v_max, 12));
            motor.recv.current_torque_f.store(uint_to_float((int)t_int, motor.info.t_min, motor.info.t_max, 12));
            motor.recv.current_temp_f.store((float)frame.data[6]);

            motor.recv.motor_id = motor_id;
            motor.recv.fault_message = err;
        } else if (motor.info.api_type == 4) {
            uint16_t pos_raw = (uint16_t)((frame.data[0] << 8) | frame.data[1]);
            int16_t spd_rpm = (int16_t)((frame.data[2] << 8) | frame.data[3]);
            int16_t iq_raw = (int16_t)((frame.data[4] << 8) | frame.data[5]);
            uint8_t temp = frame.data[6];

            motor.recv.current_position_f.store((float)pos_raw * C620_POS_TO_RAD);
            motor.recv.current_speed_f.store((float)spd_rpm * C620_RPM_TO_RAD_S);
            motor.recv.current_iq_f.store((float)iq_raw * (20.0f / 16384.0f));

            if (motor.info.t_max > motor.info.t_min) {
                float torque = ((float)iq_raw - (-16384.0f)) / (16384.0f - (-16384.0f));
                torque = torque * (motor.info.t_max - motor.info.t_min) + motor.info.t_min;
                motor.recv.current_torque_f.store(torque);
            } else {
                motor.recv.current_torque_f.store((float)iq_raw);
            }

            motor.recv.current_temp_f.store((float)temp);
            motor.recv.motor_id = (uint8_t)parsed_motor_id;
            motor.recv.fault_message = 0;
        }
    }
}
