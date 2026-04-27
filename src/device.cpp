#include "device.hpp"
#include <algorithm>
#include <cctype>
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
static const float TWO_PI = 6.28318530718f;
static const uint8_t HQ_MASTER_ID = 0;
static const bool HQ_REPLY_REQUIRED = true;
static const float HQ_POS_SCALE = 10000.0f;
static const float HQ_VEL_SCALE = 4000.0f; // int16 velocity lsb=0.00025 turns/s
static const float HQ_TORQUE_SCALE = 100.0f;
static const float HQ_KP_KD_SCALE = 10.0f; // int16 kp/kd lsb=0.1
static const float HQ_POS_SCALE_I32 = 100000.0f;
static const float HQ_VEL_SCALE_I32 = 100000.0f;
static const float HQ_TORQUE_SCALE_I32 = 1000.0f;

static inline int16_t clamp_to_i16_no_sentinel(float val) {
    // 0x8000 is reserved by HighTorque protocol as "unlimited".
    if (val > 32767.0f) val = 32767.0f;
    if (val < -32767.0f) val = -32767.0f;
    return static_cast<int16_t>(std::lround(val));
}

static inline void write_le_i16(uint8_t* dst, int16_t value) {
    const uint16_t raw = static_cast<uint16_t>(value);
    dst[0] = static_cast<uint8_t>(raw & 0xFF);
    dst[1] = static_cast<uint8_t>((raw >> 8) & 0xFF);
}

static inline void write_le_u16(uint8_t* dst, uint16_t value) {
    dst[0] = static_cast<uint8_t>(value & 0xFF);
    dst[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

static inline int16_t read_le_i16(const uint8_t* src) {
    const uint16_t raw = static_cast<uint16_t>(src[0]) |
                         (static_cast<uint16_t>(src[1]) << 8);
    return static_cast<int16_t>(raw);
}

static inline int32_t read_le_i32(const uint8_t* src) {
    const uint32_t raw = static_cast<uint32_t>(src[0]) |
                         (static_cast<uint32_t>(src[1]) << 8) |
                         (static_cast<uint32_t>(src[2]) << 16) |
                         (static_cast<uint32_t>(src[3]) << 24);
    return static_cast<int32_t>(raw);
}

static inline float read_le_f32(const uint8_t* src) {
    float value = 0.0f;
    std::memcpy(&value, src, sizeof(value));
    return value;
}

static inline uint32_t build_hq_can_id(uint8_t motor_id, bool require_reply) {
    uint8_t src = static_cast<uint8_t>(HQ_MASTER_ID & 0x7F);
    if (require_reply) src |= 0x80;
    const uint8_t dst = static_cast<uint8_t>(motor_id & 0x7F);
    return (static_cast<uint32_t>(src) << 8) | dst;
}

static bool use_canfd_brs() {
    const char* env = std::getenv("HQ_CANFD_BRS");
    if (!env) return false;
    return std::string(env) == "1";
}

static bool pfl28_use_canfd_brs() {
    // In field setups, long cabling often makes 5M data-phase unstable.
    // Keep BRS off by default for robustness; users can enable via env.
    const char* env = std::getenv("PFL28_CANFD_BRS");
    if (!env) return false;
    return std::string(env) == "1";
}

static bool pfl28_use_canfd_frame() {
    // Official manual specifies CAN FD transport for PowerFlow L28.
    // Keep CAN FD as default; allow temporary fallback via env for field debugging.
    const char* env = std::getenv("PFL28_USE_CANFD");
    if (!env) return true;
    const std::string v(env);
    return (v == "1" || v == "true" || v == "TRUE" || v == "on" || v == "ON");
}

static bool pfl28_allow_neg_current() {
    const char* env = std::getenv("PFL28_ALLOW_NEG_CURRENT");
    if (!env) return false;
    return std::string(env) == "1";
}

struct HQTypeAdapt {
    float tq_k;
    float tq_d;
};

static std::string normalize_hq_type(std::string type) {
    std::string out;
    out.reserve(type.size());
    for (char ch : type) {
        if (ch == '-' || ch == ' ') {
            out.push_back('_');
            continue;
        }
        out.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(ch))));
    }
    return out;
}

static HQTypeAdapt get_hq_type_adapt(const std::string& type_name) {
    const std::string key = normalize_hq_type(type_name);
    if (key == "M3536_32") return {0.458100f, 0.0f};
    if (key == "M4438_30") return {0.525600f, 0.0f};
    if (key == "M4438_32") return {0.558400f, 0.0f};
    if (key == "M4538_19") return {0.445000f, 0.0f};
    if (key == "M5043_20") return {0.966000f, 0.0f};
    if (key == "M5046_20") return {0.528000f, 0.0f};
    if (key == "M5047_09") return {0.533000f, 0.0f};
    if (key == "M5047_36") return {0.803000f, 0.0f};
    if (key == "M6056_36") return {0.677000f, 0.0f};
    if (key == "M7256_35") return {0.677000f, 0.0f};
    if (key == "M60SG_35") return {0.794200f, 0.0f};
    if (key == "M60BM_35") return {0.794200f, 0.0f};
    if (key == "MGENERAL") return {0.500000f, 0.0f};
    if (key == "MNONE" || key == "HQ" || key.empty()) return {1.000000f, 0.0f};
    return {1.000000f, 0.0f};
}

static float hq_adjust_torque_by_type(float tq_nm, const HQTypeAdapt& adapt) {
    if (std::fabs(adapt.tq_k) < 1e-6f) return tq_nm;
    return (tq_nm - adapt.tq_d) / adapt.tq_k;
}

static float hq_restore_torque_by_type(float tq_driver, const HQTypeAdapt& adapt) {
    return tq_driver * adapt.tq_k + adapt.tq_d;
}

static float hq_adjust_pid_by_type(float pid, const HQTypeAdapt& adapt) {
    if (std::fabs(adapt.tq_k) < 1e-6f) return pid;
    return pid / adapt.tq_k;
}

static bool tryBringUpInterface(const std::string& iface,
                                int bitrate = 1000000,
                                bool enable_canfd = false,
                                int dbitrate = 1000000) {
    std::string cmd = "ip link set " + iface + " down 2>/dev/null && ";
    cmd += "ip link set " + iface + " type can bitrate " + std::to_string(bitrate);
    if (enable_canfd) {
        cmd += " dbitrate " + std::to_string(dbitrate) + " fd on";
    }
    cmd += " 2>/dev/null && ";
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

bool DeviceX::openSocket(const std::string& iface, bool enable_canfd, int dbitrate) {
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
            if (tryBringUpInterface(iface, 1000000, enable_canfd, dbitrate)) {
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

    int sndbuf_bytes = 512 * 1024;
    if (setsockopt(socket_fd, SOL_SOCKET, SO_SNDBUF, &sndbuf_bytes, sizeof(sndbuf_bytes)) < 0) {
        std::perror("setsockopt SO_SNDBUF");
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

    bool need_canfd = false;
    int dbitrate = 1000000;
    if (p_motors_data) {
        for (const auto& motor : *p_motors_data) {
            if (motor.info.device_index != dev_idx) {
                continue;
            }
            if (motor.info.api_type == 5) {
                need_canfd = true;
            } else if (motor.info.api_type == 6) {
                if (pfl28_use_canfd_frame()) {
                    need_canfd = true;
                    dbitrate = 5000000;
                }
            }
        }
    }

    if (!openSocket(iface_name, need_canfd, dbitrate)) {
        std::cerr << "[Error] Failed to open socketcan interface: " << iface_name << std::endl;
        return false;
    }

    is_running = true;
    rx_thread = std::thread(&DeviceX::ReceiveLoop, this);
    std::cout << "[Info] SocketCAN ready on " << iface_name << std::endl;
    return true;
}

bool DeviceX::sendFrameWithRetry(const void* frame, std::size_t frame_size, const char* tag) {
    if (socket_fd < 0 || frame == nullptr || frame_size == 0) {
        return false;
    }

    constexpr int kMaxRetries = 3;
    for (int attempt = 0; attempt <= kMaxRetries; ++attempt) {
        const int n = send(socket_fd, frame, frame_size, MSG_DONTWAIT);
        if (n == static_cast<int>(frame_size)) {
            return true;
        }

        if (n >= 0) {
            std::cerr << tag << ": short send (" << n << "/" << frame_size << ")" << std::endl;
            return false;
        }

        const int err = errno;
        if (err == EAGAIN || err == EWOULDBLOCK || err == ETIMEDOUT || err == EINTR) {
            if (attempt < kMaxRetries) {
                std::this_thread::sleep_for(std::chrono::microseconds(200));
                continue;
            }
            return false;
        }

        if (err == ENOBUFS) {
            enobufs_drop_count.fetch_add(1, std::memory_order_relaxed);
            if (attempt < kMaxRetries) {
                std::this_thread::sleep_for(std::chrono::microseconds(400 * (attempt + 1)));
                continue;
            }

            const auto now_ms = static_cast<unsigned long long>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count());
            auto expected_last_ms = last_enobufs_log_ms.load(std::memory_order_relaxed);
            if (now_ms - expected_last_ms >= 1000ULL &&
                last_enobufs_log_ms.compare_exchange_strong(
                    expected_last_ms, now_ms, std::memory_order_relaxed)) {
                std::cerr << tag << ": No buffer space available on " << iface_name
                          << " (dropped=" << enobufs_drop_count.load(std::memory_order_relaxed) << "). "
                          << "Try lowering query/send rate or increasing can tx queue length." << std::endl;
            }
            return false;
        }

        if (err == ENETDOWN) {
            std::cerr << tag << ": Network is down on " << iface_name
                      << ". Closing socket until interface is up." << std::endl;
            close(socket_fd);
            socket_fd = -1;
            return false;
        }

        std::perror(tag);
        return false;
    }

    return false;
}

void DeviceX::sendExtendedFrame(uint32_t type, uint16_t data_area, uint8_t motor_id, const uint8_t* data) {
    if (socket_fd < 0) return;

    struct can_frame frame {};
    frame.can_id = ((type & 0x1F) << 24) | ((data_area & 0xFFFF) << 8) | (motor_id & 0xFF);
    frame.can_id |= CAN_EFF_FLAG;
    frame.can_dlc = 8;
    std::memcpy(frame.data, data, 8);
    sendFrameWithRetry(&frame, sizeof(frame), "sendExtendedFrame");
}

void DeviceX::sendExtendedIdFrame(uint32_t can_id, const uint8_t* data, uint8_t dlc) {
    if (socket_fd < 0 || dlc > 8) return;

    struct can_frame frame {};
    frame.can_id = (can_id & CAN_EFF_MASK) | CAN_EFF_FLAG;
    frame.can_dlc = dlc;
    std::memcpy(frame.data, data, dlc);
    sendFrameWithRetry(&frame, sizeof(frame), "sendExtendedIdFrame");
}

void DeviceX::sendExtendedIdFdFrame(uint32_t can_id, const uint8_t* data, uint8_t len) {
    if (socket_fd < 0 || len > 64) return;

    struct canfd_frame frame {};
    frame.can_id = (can_id & CAN_EFF_MASK) | CAN_EFF_FLAG;
    frame.len = len;
    frame.flags = use_canfd_brs() ? CANFD_BRS : 0;
    std::memcpy(frame.data, data, len);
    sendFrameWithRetry(&frame, sizeof(frame), "sendExtendedIdFdFrame");
}

void DeviceX::sendStandardFrame(uint32_t can_id, const uint8_t* data, uint8_t dlc) {
    if (socket_fd < 0) return;

    struct can_frame frame {};
    frame.can_id = (can_id & CAN_SFF_MASK);
    frame.can_dlc = dlc;
    std::memcpy(frame.data, data, dlc);
    sendFrameWithRetry(&frame, sizeof(frame), "sendStandardFrame");
}

void DeviceX::sendStandardFdFrame(uint32_t can_id, const uint8_t* data, uint8_t len, bool brs) {
    if (socket_fd < 0 || len > CANFD_MAX_DLEN) return;

    struct canfd_frame frame {};
    frame.can_id = (can_id & CAN_SFF_MASK);
    frame.len = len;
    frame.flags = brs ? CANFD_BRS : 0;
    std::memcpy(frame.data, data, len);
    sendFrameWithRetry(&frame, sizeof(frame), "sendStandardFdFrame");
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
    else if (type == 5) EnableMotor_Type5(motor_index);
    else if (type == 6) EnableMotor_Type6(motor_index);
}

void DeviceX::DisableMotor(int& motor_index) {
    if (!p_motors_data || motor_index < 0 || motor_index >= (int)p_motors_data->size()) return;

    int type = (*p_motors_data)[motor_index].info.api_type;

    if (type == 1) DisableMotor_Type1(motor_index);
    else if (type == 2) DisableMotor_Type2(motor_index);
    else if (type == 3) DisableMotor_Type3(motor_index);
    else if (type == 4) DisableMotor_Type4(motor_index);
    else if (type == 5) DisableMotor_Type5(motor_index);
    else if (type == 6) DisableMotor_Type6(motor_index);
}

void DeviceX::ClearError(int& motor_index) {
    if (!p_motors_data || motor_index < 0 || motor_index >= (int)p_motors_data->size()) return;

    int type = (*p_motors_data)[motor_index].info.api_type;

    if (type == 1) ClearError_Type1(motor_index);
    else if (type == 2) ClearError_Type2(motor_index);
    else if (type == 3) ClearError_Type3(motor_index);
    else if (type == 4) ClearError_Type4(motor_index);
    else if (type == 5) ClearError_Type5(motor_index);
    else if (type == 6) ClearError_Type6(motor_index);
}

void DeviceX::SetZero(int& motor_index) {
    if (!p_motors_data || motor_index < 0 || motor_index >= (int)p_motors_data->size()) return;

    int type = (*p_motors_data)[motor_index].info.api_type;

    if (type == 1) SetZero_Type1(motor_index);
    else if (type == 2) SetZero_Type2(motor_index);
    else if (type == 3) SetZero_Type3(motor_index);
    else if (type == 4) SetZero_Type4(motor_index);
    else if (type == 5) SetZero_Type5(motor_index);
    else if (type == 6) SetZero_Type6(motor_index);
}

void DeviceX::SetMode(int& motor_index, int mode) {
    if (!p_motors_data || motor_index < 0 || motor_index >= (int)p_motors_data->size()) return;

    int type = (*p_motors_data)[motor_index].info.api_type;

    if (type == 1) SetMode_Type1(motor_index, mode);
    else if (type == 2) SetMode_Type2(motor_index, mode);
    else if (type == 3) SetMode_Type3(motor_index, mode);
    else if (type == 4) SetMode_Type4(motor_index, mode);
    else if (type == 5) SetMode_Type5(motor_index, mode);
    else if (type == 6) SetMode_Type6(motor_index, mode);
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
    else if (type == 5) SendCommand_Type5(motor_index);
    else if (type == 6) SendCommand_Type6(motor_index);
}

void DeviceX::QueryPos(int& motor_index) {
    if (!p_motors_data || motor_index < 0 || motor_index >= (int)p_motors_data->size()) return;

    int type = (*p_motors_data)[motor_index].info.api_type;

    if (type == 1) QueryPos_Type1(motor_index);
    else if (type == 2) QueryPos_Type2(motor_index);
    else if (type == 3) QueryPos_Type3(motor_index);
    else if (type == 4) QueryPos_Type4(motor_index);
    else if (type == 5) QueryPos_Type5(motor_index);
    else if (type == 6) QueryPos_Type6(motor_index);
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
//  Type 5 (HighTorque/高擎)
// =========================================================

void DeviceX::EnableMotor_Type5(int& motor_index) {
    // HighTorque protocol does not expose a dedicated "enable" command in the
    // provided CAN document. We perform a state query to confirm communication.
    QueryPos_Type5(motor_index);
}

void DeviceX::DisableMotor_Type5(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    const uint32_t can_id = build_hq_can_id(static_cast<uint8_t>(info.canid), HQ_REPLY_REQUIRED);
    // ref/livelybot_fdcan.c: set_motor_stop_int16()
    const uint8_t tdata[] = {0x01, 0x00, 0x00, 0x14, 0x04, 0x00, 0x11, 0x0F};
    sendExtendedIdFrame(can_id, tdata, sizeof(tdata));
}

void DeviceX::ClearError_Type5(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    const uint32_t can_id = build_hq_can_id(static_cast<uint8_t>(info.canid), HQ_REPLY_REQUIRED);
    // ref/livelybot_fdcan.c: set_motor_reset_int8()
    const uint8_t reset_cmd[] = {
        0x40, 0x01, 0x08, 0x64, 0x20, 0x72, 0x65, 0x73, 0x65, 0x74, 0x0A, 0x50
    };
    sendExtendedIdFdFrame(can_id, reset_cmd, sizeof(reset_cmd));
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    QueryPos_Type5(motor_index);
}

void DeviceX::SetZero_Type5(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    const uint32_t can_id = build_hq_can_id(static_cast<uint8_t>(info.canid), HQ_REPLY_REQUIRED);

    // ref/livelybot_fdcan.c: set_pos_rezero()
    const uint8_t rezero_cmd[] = {
        0x40, 0x01, 0x15, 0x64, 0x20, 0x63, 0x66, 0x67,
        0x2D, 0x73, 0x65, 0x74, 0x2D, 0x6F, 0x75, 0x74,
        0x70, 0x75, 0x74, 0x20, 0x30, 0x2E, 0x30, 0x0A
    };
    sendExtendedIdFdFrame(can_id, rezero_cmd, sizeof(rezero_cmd));

    // ref/livelybot_fdcan.c: set_conf_write()
    const uint8_t conf_write_cmd[] = {
        0x40, 0x01, 0x0B, 0x63, 0x6F, 0x6E, 0x66, 0x20,
        0x77, 0x72, 0x69, 0x74, 0x65, 0x0A, 0x50, 0x50
    };
    sendExtendedIdFdFrame(can_id, conf_write_cmd, sizeof(conf_write_cmd));
}

void DeviceX::SetMode_Type5(int& motor_index, int mode) {
    if (mode < 0) mode = 0;
    if (mode > 3) mode = 3;
    (*p_motors_data)[motor_index].send.mode = static_cast<uint8_t>(mode);
}

void DeviceX::SendCommand_Type5(int& motor_index) {
    const Motor_CAN_Struct& motor = (*p_motors_data)[motor_index];
    const auto& info = motor.info;
    const auto& cmd = motor.send;
    const HQTypeAdapt adapt = get_hq_type_adapt(info.type);

    const float pos_scale = HQ_POS_SCALE;
    const float vel_scale = HQ_VEL_SCALE;
    const float tq_scale = HQ_TORQUE_SCALE;

    auto clamp_phys_torque = [&](float tq_nm) {
        if (info.t_max > info.t_min) {
            return std::max(info.t_min, std::min(info.t_max, tq_nm));
        }
        return tq_nm;
    };

    auto encode_torque_raw = [&](float tq_nm) {
        const float tq_adj = hq_adjust_torque_by_type(clamp_phys_torque(tq_nm), adapt);
        return clamp_to_i16_no_sentinel(tq_adj * tq_scale);
    };

    const float pos_turn = cmd.position / TWO_PI;
    const float vel_turn_s = cmd.speed / TWO_PI;
    const int16_t pos_raw = clamp_to_i16_no_sentinel(pos_turn * pos_scale);
    const int16_t vel_raw = clamp_to_i16_no_sentinel(vel_turn_s * vel_scale);
    const int16_t tq_ff_raw = encode_torque_raw(cmd.torque);

    const uint32_t can_id = build_hq_can_id(static_cast<uint8_t>(info.canid), HQ_REPLY_REQUIRED);
    const uint16_t unlimited = 0x8000;

    uint8_t tdata[8] = {0};
    switch (cmd.mode) {
        case 1: { // position + torque, velocity unlimited
            tdata[0] = 0x07;
            tdata[1] = 0x07;
            write_le_i16(&tdata[2], pos_raw);
            write_le_u16(&tdata[4], unlimited);
            write_le_i16(&tdata[6], tq_ff_raw);
            sendExtendedIdFrame(can_id, tdata, 8);
            break;
        }
        case 2: { // velocity + torque, position unlimited
            tdata[0] = 0x07;
            tdata[1] = 0x07;
            write_le_u16(&tdata[2], unlimited);
            write_le_i16(&tdata[4], vel_raw);
            write_le_i16(&tdata[6], tq_ff_raw);
            sendExtendedIdFrame(can_id, tdata, 8);
            break;
        }
        case 3: { // torque only
            const int16_t tq_raw = encode_torque_raw(cmd.torque);
            tdata[0] = 0x05;
            tdata[1] = 0x13;
            write_le_i16(&tdata[2], tq_raw);
            sendExtendedIdFrame(can_id, tdata, 4);
            break;
        }
        case 0:
        default: { // MIT mode 2 (int16): mode set + pos/vel/tqe + kp/kd + state query
            const int16_t tq_raw = encode_torque_raw(cmd.torque);
            const float kp_adj = hq_adjust_pid_by_type(cmd.kp, adapt);
            const float kd_adj = hq_adjust_pid_by_type(cmd.kd, adapt);
            const int16_t kp_raw = clamp_to_i16_no_sentinel(kp_adj * HQ_KP_KD_SCALE);
            const int16_t kd_raw = clamp_to_i16_no_sentinel(kd_adj * HQ_KP_KD_SCALE);

            // Reference sequence (ref/livelybot_fdcan.c set_pos_vel_tqe_kp_kd_int16_2):
            // 1) 0x01,0x00,0x15: set mode register to MIT2
            // 2) 0x07,0x20: write pos/vel/tqe int16
            // 3) 0x06,0x2B: write kp/kd int16
            // 4) 0x14,0x04,0x00,0x11,0x0F: query state int16 (mode/fault/pos/vel/tqe)
            uint8_t fd_data[24] = {0};
            fd_data[0] = 0x01;
            fd_data[1] = 0x00;
            fd_data[2] = 0x15;

            fd_data[3] = 0x07;
            fd_data[4] = 0x20;
            write_le_i16(&fd_data[5], pos_raw);
            write_le_i16(&fd_data[7], vel_raw);
            write_le_i16(&fd_data[9], tq_raw);

            fd_data[11] = 0x06;
            fd_data[12] = 0x2B;
            write_le_i16(&fd_data[13], kp_raw);
            write_le_i16(&fd_data[15], kd_raw);

            fd_data[17] = 0x14;
            fd_data[18] = 0x04;
            fd_data[19] = 0x00;
            fd_data[20] = 0x11;
            fd_data[21] = 0x0F;

            fd_data[22] = 0x50;
            fd_data[23] = 0x50;
            sendExtendedIdFdFrame(can_id, fd_data, sizeof(fd_data));
            break;
        }
    }
}

void DeviceX::QueryPos_Type5(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    const uint32_t can_id = build_hq_can_id(static_cast<uint8_t>(info.canid), HQ_REPLY_REQUIRED);
    // ref/livelybot_fdcan.c: read_motor_state_int16()
    const uint8_t tdata[] = {0x14, 0x04, 0x00, 0x11, 0x0F};
    sendExtendedIdFrame(can_id, tdata, sizeof(tdata));
}

// =========================================================
//  Type 6 (AgiBot PowerFlow L28/PFL28)
// =========================================================

void DeviceX::EnableMotor_Type6(int& motor_index) {
    // PFL28 powers up in enabled state.
    (void)motor_index;
}

void DeviceX::DisableMotor_Type6(int& motor_index) {
    // No dedicated disable command in PFL28 public protocol.
    // Send a zero-current hold at the latest known position as a safe fallback.
    if (!p_motors_data) return;
    auto& motor = (*p_motors_data)[motor_index];
    motor.send.position = motor.recv.current_position_f.load();
    motor.send.torque = 0.0f;
    SendCommand_Type6(motor_index);
}

void DeviceX::ClearError_Type6(int& motor_index) {
    // No clear-error frame is documented for PFL28.
    (void)motor_index;
}

void DeviceX::SetZero_Type6(int& motor_index) {
    // No set-zero frame is documented for PFL28.
    (void)motor_index;
}

void DeviceX::SetMode_Type6(int& motor_index, int mode) {
    // PFL28 uses position/current command frame; keep mode for compatibility.
    (*p_motors_data)[motor_index].send.mode = static_cast<uint8_t>(mode);
}

void DeviceX::SendCommand_Type6(int& motor_index) {
    const auto& motor = (*p_motors_data)[motor_index];
    const auto& info = motor.info;
    const bool allow_neg_i = pfl28_allow_neg_current();

    const float pos_min = (info.p_max > info.p_min) ? info.p_min : 0.0f;
    const float pos_max = (info.p_max > info.p_min) ? info.p_max : 9.5f;
    const float cur_max = (info.t_max > info.t_min) ? info.t_max : 2.5f;
    float cur_min = (info.t_max > info.t_min) ? info.t_min : 0.0f;
    if (allow_neg_i && cur_min >= 0.0f) {
        cur_min = -cur_max;
    }

    float pos_cmd = motor.send.position;
    float cur_cmd = motor.send.torque;

    if (!std::isfinite(pos_cmd)) pos_cmd = 0.0f;
    if (!std::isfinite(cur_cmd)) cur_cmd = 0.0f;

    pos_cmd = std::max(pos_min, std::min(pos_max, pos_cmd));
    cur_cmd = std::max(cur_min, std::min(cur_max, cur_cmd));

    uint8_t data[8] = {0};
    std::memcpy(&data[0], &pos_cmd, sizeof(float)); // little-endian float
    std::memcpy(&data[4], &cur_cmd, sizeof(float)); // little-endian float

    if (pfl28_use_canfd_frame()) {
        sendStandardFdFrame(static_cast<uint32_t>(info.canid), data, sizeof(data), pfl28_use_canfd_brs());
    } else {
        sendStandardFrame(static_cast<uint32_t>(info.canid), data, sizeof(data));
    }
}

void DeviceX::QueryPos_Type6(int& motor_index) {
    // PFL28 returns state after each control frame; no standalone query frame documented.
    // We keep this as no-op to avoid accidental motion during status polling loops.
    (void)motor_index;
}

// =========================================================
//  Receive Loop
// =========================================================

void DeviceX::ReceiveLoop() {
    while (is_running) {
        struct canfd_frame frame {};
        int n = read(socket_fd, &frame, sizeof(frame));
        if (n < 0) {
            if (errno == EINTR) continue;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        if (n != CAN_MTU && n != CANFD_MTU) {
            continue;
        }

        const uint8_t frame_len = frame.len;
        if (frame_len == 0 || frame_len > CANFD_MAX_DLEN) continue;

        int parsed_motor_id = -1;
        uint8_t rx_channel = 0;
        const bool is_eff = (frame.can_id & CAN_EFF_FLAG) != 0;
        const uint32_t canID = frame.can_id & (is_eff ? CAN_EFF_MASK : CAN_SFF_MASK);

        if (is_eff) {
            parsed_motor_id = (canID >> 8) & 0xFF;
        } else {
            if (canID >= 0x140 && canID < 0x160) {
                parsed_motor_id = canID - 0x140;
            } else if (canID >= 0x180 && canID < 0x1A0) {
                parsed_motor_id = canID - 0x180;
            } else if (canID >= 0x201 && canID <= 0x208) {
                parsed_motor_id = canID - 0x200;
            } else if (canID > 0 && canID <= 0x7F) {
                // PFL28/L28 P2P frames: CAN ID is node id.
                parsed_motor_id = static_cast<int>(canID);
            } else if ((((canID >> 8) & 0x7F) > 0) && ((canID & 0x7F) == 0 || (canID & 0x7F) == 0x7F)) {
                // HighTorque reply id format: [src(7bit)][dst(7bit)].
                parsed_motor_id = static_cast<int>((canID >> 8) & 0x7F);
            } else if (frame_len > 0) {
                parsed_motor_id = frame.data[0] & 0x0F;
            }
        }

        if (parsed_motor_id < 0) continue;

        int g_idx = p_mapper->get_id({(uint)device_global_index, (uint)rx_channel, (uint)parsed_motor_id});

        // Fallback for protocols that carry motor id in payload nibble.
        if (g_idx < 0 && !is_eff && frame_len > 0) {
            const int alt_motor_id = frame.data[0] & 0x0F;
            if (alt_motor_id != parsed_motor_id) {
                const int alt_idx = p_mapper->get_id(
                    {(uint)device_global_index, (uint)rx_channel, (uint)alt_motor_id});
                if (alt_idx >= 0) {
                    parsed_motor_id = alt_motor_id;
                    g_idx = alt_idx;
                }
            }
        }

        if (g_idx < 0 || g_idx >= (int)p_motors_data->size()) continue;

        Motor_CAN_Struct& motor = (*p_motors_data)[g_idx];

        if (motor.info.api_type == 1 && is_eff) {
            uint8_t type_field = (canID >> 24) & 0x1F;

            if (type_field == 2 && frame_len >= 8) {
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
            } else if (type_field == 17 && frame_len >= 8) {
                if (frame.data[0] == 0x19 && frame.data[1] == 0x70) {
                    float temp_val = 0.0f;
                    std::memcpy(&temp_val, &frame.data[4], 4);
                    motor.recv.current_position_f.store(temp_val);
                }
            } else if (type_field == 21 && frame_len >= 4) {
                uint32_t fault_word = 0;
                std::memcpy(&fault_word, &frame.data[0], 4);
                motor.recv.fault_message = static_cast<uint8_t>(fault_word);
            }
        } else if (motor.info.api_type == 2 && !(frame.can_id & CAN_EFF_FLAG)) {
            uint8_t status = frame.data[0];
            if (frame_len >= 8 && (status == 0x9C || status == 0xA1 || status == 0xA2 || status == 0xA3)) {
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
        } else if (motor.info.api_type == 3 && frame_len >= 8) {
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
        } else if (motor.info.api_type == 4 && frame_len >= 7) {
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
        } else if (motor.info.api_type == 6 && !(frame.can_id & CAN_EFF_FLAG) && frame_len >= 8) {
            float pos_feedback = 0.0f;
            float cur_feedback = 0.0f;
            std::memcpy(&pos_feedback, &frame.data[0], sizeof(float));
            std::memcpy(&cur_feedback, &frame.data[4], sizeof(float));

            if (std::isfinite(pos_feedback)) {
                motor.recv.current_position_f.store(pos_feedback);
            }
            if (std::isfinite(cur_feedback)) {
                motor.recv.current_torque_f.store(cur_feedback);
                motor.recv.current_iq_f.store(cur_feedback);
            }
            motor.recv.current_speed_f.store(0.0f);
            motor.recv.motor_id = static_cast<uint8_t>(parsed_motor_id);
            motor.recv.mode = motor.send.mode;
            motor.recv.motor_state = 1; // full state frame (pos+cur)
            motor.recv.fault_message = 0;
        } else if (motor.info.api_type == 6 && !(frame.can_id & CAN_EFF_FLAG) && frame_len == 4) {
            // Some PFL28 firmwares return compact status/error frames.
            // Observed pattern: [0x07, 0x02, 0x00, err_code].
            motor.recv.motor_id = static_cast<uint8_t>(parsed_motor_id);
            motor.recv.mode = frame.data[0];
            motor.recv.motor_state = 2; // compact status/error frame
            motor.recv.fault_message = frame.data[3];
        } else if (motor.info.api_type == 5) {
            if (frame_len < 2) continue;
            const HQTypeAdapt adapt = get_hq_type_adapt(motor.info.type);

            const uint8_t cmd = frame.data[0];
            if (cmd == 0x24 && frame_len >= 14 &&
                frame.data[1] == 0x04 && frame.data[2] == 0x00 &&
                frame.data[11] == 0x21 && frame.data[12] == 0x0F) {
                const int16_t pos_raw = read_le_i16(&frame.data[5]);
                const int16_t vel_raw = read_le_i16(&frame.data[7]);
                const int16_t tq_raw = read_le_i16(&frame.data[9]);

                const float pos_turn = static_cast<float>(pos_raw) / HQ_POS_SCALE;
                const float vel_turn_s = static_cast<float>(vel_raw) / HQ_VEL_SCALE;
                const float tq_driver = static_cast<float>(tq_raw) / HQ_TORQUE_SCALE;
                const float tq_nm = hq_restore_torque_by_type(tq_driver, adapt);

                motor.recv.current_position_f.store(pos_turn * TWO_PI);
                motor.recv.current_speed_f.store(vel_turn_s * TWO_PI);
                motor.recv.current_iq_f.store(tq_driver);
                motor.recv.current_torque_f.store(tq_nm);
                motor.recv.motor_id = static_cast<uint8_t>(parsed_motor_id);
                motor.recv.mode = frame.data[3];
                motor.recv.fault_message = frame.data[13];
            } else if (cmd == 0x28 && frame_len >= 22 &&
                       frame.data[1] == 0x04 && frame.data[2] == 0x00 &&
                       frame.data[19] == 0x21 && frame.data[20] == 0x0F) {
                const int32_t pos_raw = read_le_i32(&frame.data[7]);
                const int32_t vel_raw = read_le_i32(&frame.data[11]);
                const int32_t tq_raw = read_le_i32(&frame.data[15]);

                const float pos_turn = static_cast<float>(pos_raw) / HQ_POS_SCALE_I32;
                const float vel_turn_s = static_cast<float>(vel_raw) / HQ_VEL_SCALE_I32;
                const float tq_driver = static_cast<float>(tq_raw) / HQ_TORQUE_SCALE_I32;
                const float tq_nm = hq_restore_torque_by_type(tq_driver, adapt);

                motor.recv.current_position_f.store(pos_turn * TWO_PI);
                motor.recv.current_speed_f.store(vel_turn_s * TWO_PI);
                motor.recv.current_iq_f.store(tq_driver);
                motor.recv.current_torque_f.store(tq_nm);
                motor.recv.motor_id = static_cast<uint8_t>(parsed_motor_id);
                motor.recv.mode = frame.data[3];
                motor.recv.fault_message = frame.data[21];
            } else if (cmd == 0x2C && frame_len >= 22 &&
                       frame.data[1] == 0x04 && frame.data[2] == 0x00 &&
                       frame.data[19] == 0x21 && frame.data[20] == 0x0F) {
                const float pos_turn = read_le_f32(&frame.data[7]);
                const float vel_turn_s = read_le_f32(&frame.data[11]);
                const float tq_driver = read_le_f32(&frame.data[15]);
                const float tq_nm = hq_restore_torque_by_type(tq_driver, adapt);

                motor.recv.current_position_f.store(pos_turn * TWO_PI);
                motor.recv.current_speed_f.store(vel_turn_s * TWO_PI);
                motor.recv.current_iq_f.store(tq_driver);
                motor.recv.current_torque_f.store(tq_nm);
                motor.recv.motor_id = static_cast<uint8_t>(parsed_motor_id);
                motor.recv.mode = frame.data[3];
                motor.recv.fault_message = frame.data[21];
            } else if (cmd == 0x27 && frame.data[1] == 0x01 && frame_len >= 8) {
                const int16_t pos_raw = read_le_i16(&frame.data[2]);
                const int16_t vel_raw = read_le_i16(&frame.data[4]);
                const int16_t tq_raw = read_le_i16(&frame.data[6]);
                const float pos_turn = static_cast<float>(pos_raw) / HQ_POS_SCALE;
                const float vel_turn_s = static_cast<float>(vel_raw) / HQ_VEL_SCALE;
                const float tq_driver = static_cast<float>(tq_raw) / HQ_TORQUE_SCALE;
                const float tq_nm = hq_restore_torque_by_type(tq_driver, adapt);

                motor.recv.current_position_f.store(pos_turn * TWO_PI);
                motor.recv.current_speed_f.store(vel_turn_s * TWO_PI);
                motor.recv.current_iq_f.store(tq_driver);
                motor.recv.current_torque_f.store(tq_nm);
                motor.recv.motor_id = static_cast<uint8_t>(parsed_motor_id);
                motor.recv.mode = motor.send.mode;
                motor.recv.fault_message = 0;
            }
        }
    }
}
