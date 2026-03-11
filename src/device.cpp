#include "device.hpp"
#include <algorithm>

namespace {

constexpr float TYPE5_TWO_PI = 6.28318530718f;

constexpr uint8_t TYPE5_CMD_WRITE = 0x01;
constexpr uint8_t TYPE5_CMD_WRITE_ACK = 0x02;
constexpr uint8_t TYPE5_CMD_READ = 0x03;
constexpr uint8_t TYPE5_CMD_READ_ACK = 0x04;
constexpr uint8_t TYPE5_CMD_FAST_WRITE = 0x05;

constexpr uint8_t TYPE5_ADDR_TORQUE_NOW = 0x05;
constexpr uint8_t TYPE5_ADDR_SPEED_NOW = 0x06;
constexpr uint8_t TYPE5_ADDR_POS_NOW = 0x07;
constexpr uint8_t TYPE5_ADDR_TARGET_TORQUE = 0x08;
constexpr uint8_t TYPE5_ADDR_TARGET_SPEED = 0x09;
constexpr uint8_t TYPE5_ADDR_TARGET_POS = 0x0A;
constexpr uint8_t TYPE5_ADDR_TARGET_ACC = 0x0B;
constexpr uint8_t TYPE5_ADDR_TARGET_DEC = 0x0C;
constexpr uint8_t TYPE5_ADDR_MODE = 0x0F;
constexpr uint8_t TYPE5_ADDR_ENABLE = 0x10;
constexpr uint8_t TYPE5_ADDR_STOP = 0x11;
constexpr uint8_t TYPE5_ADDR_FAULT = 0x15;
constexpr uint8_t TYPE5_ADDR_TEMP = 0x1D;
constexpr uint8_t TYPE5_ADDR_CURRENT_P = 0x23;
constexpr uint8_t TYPE5_ADDR_CURRENT_I = 0x24;
constexpr uint8_t TYPE5_ADDR_SPEED_P = 0x25;
constexpr uint8_t TYPE5_ADDR_SPEED_I = 0x26;
constexpr uint8_t TYPE5_ADDR_POSITION_P = 0x27;
constexpr uint8_t TYPE5_ADDR_POSITION_I = 0x28;
constexpr uint8_t TYPE5_ADDR_SAVE = 0x4D;

float type5EncoderCpr(const Motor_CAN_Info_Struct& info) {
    return (info.type5_encoder_cpr > 0.0f) ? info.type5_encoder_cpr : 65536.0f;
}

float type5DirSign(const Motor_CAN_Info_Struct& info) {
    return (std::fabs(info.type5_dir_sign) > 1e-6f) ? info.type5_dir_sign : 1.0f;
}

float type5RatedTorque(const Motor_CAN_Info_Struct& info) {
    if (info.type5_rated_torque_nm > 1e-6f) {
        return info.type5_rated_torque_nm;
    }

    const float fallback = std::max(std::fabs(info.t_min), std::fabs(info.t_max));
    return (fallback > 1e-6f) ? fallback : 1.0f;
}

int32_t type5ReadBeI32(const uint8_t* data) {
    const uint32_t raw = (static_cast<uint32_t>(data[0]) << 24)
        | (static_cast<uint32_t>(data[1]) << 16)
        | (static_cast<uint32_t>(data[2]) << 8)
        | static_cast<uint32_t>(data[3]);
    return static_cast<int32_t>(raw);
}

void type5WriteBeI32(uint8_t* data, int32_t value) {
    const uint32_t raw = static_cast<uint32_t>(value);
    data[0] = static_cast<uint8_t>((raw >> 24) & 0xFF);
    data[1] = static_cast<uint8_t>((raw >> 16) & 0xFF);
    data[2] = static_cast<uint8_t>((raw >> 8) & 0xFF);
    data[3] = static_cast<uint8_t>(raw & 0xFF);
}

int32_t type5PosRadToRaw(const Motor_CAN_Info_Struct& info, float pos_rad) {
    const float raw = ((pos_rad - info.type5_zero_offset_rad) / type5DirSign(info))
        * type5EncoderCpr(info) / TYPE5_TWO_PI;
    return static_cast<int32_t>(std::lround(raw));
}

float type5RawToPosRad(const Motor_CAN_Info_Struct& info, int32_t raw) {
    return type5DirSign(info) * (static_cast<float>(raw) * TYPE5_TWO_PI / type5EncoderCpr(info))
        + info.type5_zero_offset_rad;
}

int32_t type5VelRadToRaw(const Motor_CAN_Info_Struct& info, float vel_rad_s) {
    const float raw = (vel_rad_s / type5DirSign(info)) * type5EncoderCpr(info) / TYPE5_TWO_PI;
    return static_cast<int32_t>(std::lround(raw));
}

float type5RawToVelRad(const Motor_CAN_Info_Struct& info, int32_t raw) {
    return type5DirSign(info) * (static_cast<float>(raw) * TYPE5_TWO_PI / type5EncoderCpr(info));
}

int32_t type5TorqueNmToRaw(const Motor_CAN_Info_Struct& info, float torque_nm) {
    const float raw = torque_nm / type5RatedTorque(info) * 1000.0f;
    return static_cast<int32_t>(std::lround(raw));
}

float type5RawToTorqueNm(const Motor_CAN_Info_Struct& info, int32_t raw) {
    return static_cast<float>(raw) * type5RatedTorque(info) / 1000.0f;
}

float type5DefaultTorqueLimitNm(const Motor_CAN_Info_Struct& info, float requested_torque) {
    if (std::fabs(requested_torque) > 1e-6f) {
        return std::fabs(requested_torque);
    }
    return std::max(std::fabs(info.t_min), std::fabs(info.t_max));
}

float type5DefaultProfileVelocity(const Motor_CAN_Info_Struct& info, float requested_velocity) {
    if (std::fabs(requested_velocity) > 1e-6f) {
        return std::fabs(requested_velocity);
    }
    if (info.type5_profile_velocity > 1e-6f) {
        return info.type5_profile_velocity;
    }
    return std::max(std::fabs(info.v_min), std::fabs(info.v_max));
}

float type5DefaultProfileAccel(const Motor_CAN_Info_Struct& info, float fallback_velocity) {
    if (info.type5_profile_acc > 1e-6f) {
        return info.type5_profile_acc;
    }
    return (fallback_velocity > 1e-6f) ? fallback_velocity : 1.0f;
}

float type5DefaultProfileDecel(const Motor_CAN_Info_Struct& info, float fallback_velocity) {
    if (info.type5_profile_dec > 1e-6f) {
        return info.type5_profile_dec;
    }
    return (fallback_velocity > 1e-6f) ? fallback_velocity : 1.0f;
}

int type5MapMode(const Motor_CAN_Info_Struct& info, int unified_mode) {
    switch (unified_mode) {
    case 0:
    case 3:
        return info.type5_mode_current;
    case 2:
        return info.type5_mode_speed;
    case 1:
    default:
        return info.type5_mode_position;
    }
}

constexpr float TYPE6_TWO_PI = 6.28318530718f;
constexpr float TYPE6_RAD_TO_RPM = 60.0f / TYPE6_TWO_PI;
constexpr float TYPE6_RPM_TO_RAD = TYPE6_TWO_PI / 60.0f;

constexpr float TYPE6_DEFAULT_POS_MIN = -12.5f;
constexpr float TYPE6_DEFAULT_POS_MAX = 12.5f;
constexpr float TYPE6_DEFAULT_SPD_MIN = -18.0f;
constexpr float TYPE6_DEFAULT_SPD_MAX = 18.0f;
constexpr float TYPE6_DEFAULT_KP_MIN = 0.0f;
constexpr float TYPE6_DEFAULT_KP_MAX = 500.0f;
constexpr float TYPE6_DEFAULT_KD_MIN = 0.0f;
constexpr float TYPE6_DEFAULT_KD_MAX = 5.0f;
constexpr float TYPE6_DEFAULT_TOR_MIN = -30.0f;
constexpr float TYPE6_DEFAULT_TOR_MAX = 30.0f;
constexpr float TYPE6_DEFAULT_CURRENT_LIMIT_A = 10.0f;
constexpr float TYPE6_DEFAULT_CURRENT_FB_MIN_A = -90.0f;
constexpr float TYPE6_DEFAULT_CURRENT_FB_MAX_A = 90.0f;

constexpr uint8_t TYPE6_MODE_MIT = 0x00;
constexpr uint8_t TYPE6_MODE_POSITION = 0x01;
constexpr uint8_t TYPE6_MODE_SPEED = 0x02;
constexpr uint8_t TYPE6_MODE_CUR_TOR = 0x03;
constexpr uint8_t TYPE6_MODE_BRAKE = 0x04;
constexpr uint8_t TYPE6_MODE_QUERY = 0x07;

constexpr uint8_t TYPE6_ACK_NONE = 0;
constexpr uint8_t TYPE6_ACK_TYPE1 = 1;
constexpr uint8_t TYPE6_ACK_TYPE2 = 2;
constexpr uint8_t TYPE6_ACK_TYPE3 = 3;

constexpr uint8_t TYPE6_CUR_STATE_CURRENT = 0;
constexpr uint8_t TYPE6_CUR_STATE_TORQUE = 1;
constexpr uint8_t TYPE6_CUR_STATE_DAMPING = 2;

float type6Clip(float value, float low, float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

void type6ResolveRange(float cfg_min, float cfg_max, float def_min, float def_max, float& out_min, float& out_max) {
    if (cfg_max > cfg_min) {
        out_min = cfg_min;
        out_max = cfg_max;
    } else {
        out_min = def_min;
        out_max = def_max;
    }
}

uint16_t type6PosToRaw(const Motor_CAN_Info_Struct& info, float pos_rad) {
    float p_min = 0.0f;
    float p_max = 0.0f;
    type6ResolveRange(info.p_min, info.p_max, TYPE6_DEFAULT_POS_MIN, TYPE6_DEFAULT_POS_MAX, p_min, p_max);
    return static_cast<uint16_t>(float_to_uint(pos_rad, p_min, p_max, 16));
}

float type6RawToPos(const Motor_CAN_Info_Struct& info, uint16_t raw) {
    float p_min = 0.0f;
    float p_max = 0.0f;
    type6ResolveRange(info.p_min, info.p_max, TYPE6_DEFAULT_POS_MIN, TYPE6_DEFAULT_POS_MAX, p_min, p_max);
    return uint_to_float(static_cast<int>(raw), p_min, p_max, 16);
}

uint16_t type6SpdToRaw(const Motor_CAN_Info_Struct& info, float spd_rad_s) {
    float v_min = 0.0f;
    float v_max = 0.0f;
    type6ResolveRange(info.v_min, info.v_max, TYPE6_DEFAULT_SPD_MIN, TYPE6_DEFAULT_SPD_MAX, v_min, v_max);
    return static_cast<uint16_t>(float_to_uint(spd_rad_s, v_min, v_max, 12));
}

float type6RawToSpd(const Motor_CAN_Info_Struct& info, uint16_t raw) {
    float v_min = 0.0f;
    float v_max = 0.0f;
    type6ResolveRange(info.v_min, info.v_max, TYPE6_DEFAULT_SPD_MIN, TYPE6_DEFAULT_SPD_MAX, v_min, v_max);
    return uint_to_float(static_cast<int>(raw), v_min, v_max, 12);
}

uint16_t type6KpToRaw(const Motor_CAN_Info_Struct& info, float kp) {
    float kp_min = 0.0f;
    float kp_max = 0.0f;
    type6ResolveRange(info.kp_min, info.kp_max, TYPE6_DEFAULT_KP_MIN, TYPE6_DEFAULT_KP_MAX, kp_min, kp_max);
    return static_cast<uint16_t>(float_to_uint(kp, kp_min, kp_max, 12));
}

uint16_t type6KdToRaw(const Motor_CAN_Info_Struct& info, float kd) {
    float kd_min = 0.0f;
    float kd_max = 0.0f;
    type6ResolveRange(info.kd_min, info.kd_max, TYPE6_DEFAULT_KD_MIN, TYPE6_DEFAULT_KD_MAX, kd_min, kd_max);
    return static_cast<uint16_t>(float_to_uint(kd, kd_min, kd_max, 9));
}

uint16_t type6TorToRaw(const Motor_CAN_Info_Struct& info, float tor) {
    float t_min = 0.0f;
    float t_max = 0.0f;
    type6ResolveRange(info.t_min, info.t_max, TYPE6_DEFAULT_TOR_MIN, TYPE6_DEFAULT_TOR_MAX, t_min, t_max);
    return static_cast<uint16_t>(float_to_uint(tor, t_min, t_max, 12));
}

uint16_t type6CurrentLimitToRaw12(float current_limit_a) {
    const float clipped = type6Clip(current_limit_a, 0.0f, 409.5f);
    return static_cast<uint16_t>(std::lround(clipped * 10.0f));
}

uint16_t type6CurrentLimitToRaw16(float current_limit_a) {
    const float clipped = type6Clip(current_limit_a, 0.0f, 6553.5f);
    return static_cast<uint16_t>(std::lround(clipped * 10.0f));
}

uint16_t type6SpeedLimitToRaw15(float speed_rad_s) {
    const float speed_rpm = std::fabs(speed_rad_s) * TYPE6_RAD_TO_RPM;
    const float clipped = type6Clip(speed_rpm, 0.0f, 3276.7f);
    return static_cast<uint16_t>(std::lround(clipped * 10.0f));
}

int16_t type6SignedCmdToRaw(float value) {
    const float clipped = type6Clip(value, -327.68f, 327.67f);
    return static_cast<int16_t>(std::lround(clipped * 100.0f));
}

float type6RawToCurrentDefault(int raw_u12) {
    return uint_to_float(raw_u12, TYPE6_DEFAULT_CURRENT_FB_MIN_A, TYPE6_DEFAULT_CURRENT_FB_MAX_A, 12);
}

float type6TempRawToDeg(uint8_t raw_temp) {
    return (static_cast<float>(raw_temp) - 50.0f) * 0.5f;
}

void type6WriteBeU16(uint8_t* data, uint16_t value) {
    data[0] = static_cast<uint8_t>((value >> 8) & 0xFF);
    data[1] = static_cast<uint8_t>(value & 0xFF);
}

uint16_t type6ReadBeU16(const uint8_t* data) {
    return static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1]));
}

int16_t type6ReadBeI16(const uint8_t* data) {
    return static_cast<int16_t>(type6ReadBeU16(data));
}

void type6WriteBeF32(uint8_t* data, float value) {
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    data[0] = static_cast<uint8_t>((bits >> 24) & 0xFF);
    data[1] = static_cast<uint8_t>((bits >> 16) & 0xFF);
    data[2] = static_cast<uint8_t>((bits >> 8) & 0xFF);
    data[3] = static_cast<uint8_t>(bits & 0xFF);
}

float type6ReadBeF32(const uint8_t* data) {
    const uint32_t bits = (static_cast<uint32_t>(data[0]) << 24)
        | (static_cast<uint32_t>(data[1]) << 16)
        | (static_cast<uint32_t>(data[2]) << 8)
        | static_cast<uint32_t>(data[3]);
    float value = 0.0f;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

void type6PackMitCmd(uint8_t* data, uint16_t kp_raw, uint16_t kd_raw, uint16_t pos_raw, uint16_t spd_raw, uint16_t tor_raw) {
    data[0] = static_cast<uint8_t>((TYPE6_MODE_MIT << 5) | ((kp_raw >> 7) & 0x1F));
    data[1] = static_cast<uint8_t>(((kp_raw & 0x7F) << 1) | ((kd_raw >> 8) & 0x01));
    data[2] = static_cast<uint8_t>(kd_raw & 0xFF);
    data[3] = static_cast<uint8_t>((pos_raw >> 8) & 0xFF);
    data[4] = static_cast<uint8_t>(pos_raw & 0xFF);
    data[5] = static_cast<uint8_t>((spd_raw >> 4) & 0xFF);
    data[6] = static_cast<uint8_t>(((spd_raw & 0x0F) << 4) | ((tor_raw >> 8) & 0x0F));
    data[7] = static_cast<uint8_t>(tor_raw & 0xFF);
}

void type6PackPositionCmd(uint8_t* data, float pos_deg, uint16_t speed_limit_raw, uint16_t current_limit_raw, uint8_t ack_type) {
    uint8_t pos_data[4] = {0};
    type6WriteBeF32(pos_data, pos_deg);
    const uint32_t pos_bits = (static_cast<uint32_t>(pos_data[0]) << 24)
        | (static_cast<uint32_t>(pos_data[1]) << 16)
        | (static_cast<uint32_t>(pos_data[2]) << 8)
        | static_cast<uint32_t>(pos_data[3]);

    data[0] = static_cast<uint8_t>((TYPE6_MODE_POSITION << 5) | ((pos_bits >> 27) & 0x1F));
    data[1] = static_cast<uint8_t>((pos_bits >> 19) & 0xFF);
    data[2] = static_cast<uint8_t>((pos_bits >> 11) & 0xFF);
    data[3] = static_cast<uint8_t>((pos_bits >> 3) & 0xFF);
    data[4] = static_cast<uint8_t>(((pos_bits & 0x07) << 5) | ((speed_limit_raw >> 10) & 0x1F));
    data[5] = static_cast<uint8_t>((speed_limit_raw >> 2) & 0xFF);
    data[6] = static_cast<uint8_t>(((speed_limit_raw & 0x03) << 6) | ((current_limit_raw >> 6) & 0x3F));
    data[7] = static_cast<uint8_t>(((current_limit_raw & 0x3F) << 2) | (ack_type & 0x03));
}

uint8_t type6FeedbackType(uint8_t first_byte) {
    return static_cast<uint8_t>((first_byte >> 5) & 0x07);
}

uint8_t type6FeedbackError(uint8_t first_byte) {
    return static_cast<uint8_t>(first_byte & 0x1F);
}

uint8_t type6PackModeStateAck(uint8_t mode, uint8_t state, uint8_t ack) {
    return static_cast<uint8_t>(((mode & 0x07) << 5) | ((state & 0x07) << 2) | (ack & 0x03));
}

} // namespace

// =========================================================
//  物理常数与系数定义 (保持原样)
// =========================================================
static const float RAD_TO_DEG = 57.2957795f;
static const float DEG_TO_RAD = 0.017453293f;

// 接收系数
static const float P_RIOT = 6.28318530718f / 65536.0f;
static const float V_ROIT = 57.29578f; 
static const float IQ_ROIT = 0.008056640625f; 

// 发送系数
static const float LK_CURRENT_SEND_FACTOR = 100.0f;
static const float LK_SPEED_SEND_FACTOR = 57.29578f * 100.0f;
static const float LK_POS_SEND_FACTOR = 57.29578f * 100.0f;

// C620 反馈转换系数
static const float C620_POS_TO_RAD = 6.28318530718f / 8191.0f;
static const float C620_RPM_TO_RAD_S = 6.28318530718f / 60.0f;

// =========================================================
//  DeviceX 实现
// =========================================================
bool DeviceX::Init(int32_t handle, const std::string& dev_name, int dev_idx, 
                      std::vector<Motor_CAN_Struct>* data_ptr, TopoMapper* mapper_ptr) {
    
    this->device_handle = handle;
    this->device_global_index = dev_idx;
    this->p_motors_data = data_ptr;
    this->p_mapper = mapper_ptr;
    
    is_running = true;
    rx_thread = std::thread(&DeviceX::ReceiveLoop, this);
    return true;
}

// =========================================================
//  DeviceX 公共接口 (Dispatcher / 分发层)
// =========================================================

void DeviceX::EnableMotor(int& motor_index) {
    if (!p_motors_data || motor_index < 0 || motor_index >= p_motors_data->size()) return;
    
    int type = (*p_motors_data)[motor_index].info.api_type;
    
    if (type == 1) {
        EnableMotor_Type1(motor_index);
    } else if (type == 2) {
        EnableMotor_Type2(motor_index);
    } else if (type == 3) {
        EnableMotor_Type3(motor_index);
    } else if (type == 4) {
        EnableMotor_Type4(motor_index);
    } else if (type == 5) {
        EnableMotor_Type5(motor_index);
    } else if (type == 6) {
        EnableMotor_Type6(motor_index);
    }
}

void DeviceX::DisableMotor(int& motor_index) {
    if (!p_motors_data || motor_index < 0 || motor_index >= p_motors_data->size()) return;
    
    int type = (*p_motors_data)[motor_index].info.api_type;
    
    if (type == 1) {
        DisableMotor_Type1(motor_index);
    } else if (type == 2) {
        DisableMotor_Type2(motor_index);
    } else if (type == 3) {
        DisableMotor_Type3(motor_index);
    } else if (type == 4) {
        DisableMotor_Type4(motor_index);
    } else if (type == 5) {
        DisableMotor_Type5(motor_index);
    } else if (type == 6) {
        DisableMotor_Type6(motor_index);
    }
}

void DeviceX::ClearError(int& motor_index) {
    if (!p_motors_data || motor_index < 0 || motor_index >= p_motors_data->size()) return;
    
    int type = (*p_motors_data)[motor_index].info.api_type;
    
    if (type == 1) {
        ClearError_Type1(motor_index);
    } else if (type == 2) {
        ClearError_Type2(motor_index);
    } else if (type == 3) {
        ClearError_Type3(motor_index);
    } else if (type == 4) {
        ClearError_Type4(motor_index);
    } else if (type == 5) {
        ClearError_Type5(motor_index);
    } else if (type == 6) {
        ClearError_Type6(motor_index);
    }
}

void DeviceX::SetMode(int& motor_index, int mode) {
    if (!p_motors_data || motor_index < 0 || motor_index >= p_motors_data->size()) return;
    
    int type = (*p_motors_data)[motor_index].info.api_type;
    
    if (type == 1) {
        SetMode_Type1(motor_index, mode);
    } else if (type == 2) {
        SetMode_Type2(motor_index, mode);
    } else if (type == 3) {
        SetMode_Type3(motor_index, mode);
    } else if (type == 4) {
        SetMode_Type4(motor_index, mode);
    } else if (type == 5) {
        SetMode_Type5(motor_index, mode);
    } else if (type == 6) {
        SetMode_Type6(motor_index, mode);
    }
}

void DeviceX::SendCommand(int& motor_index) {
    if (!p_motors_data || motor_index < 0 || motor_index >= p_motors_data->size()) {
        std::cerr << "[ERROR] SendCommand Failed! Index Out of Range or Null Ptr." << std::endl;
        return;
    }

    int type = (*p_motors_data)[motor_index].info.api_type;

    if (type == 1) {
        SendCommand_Type1(motor_index);
    } else if (type == 2) {
        SendCommand_Type2(motor_index);
    } else if (type == 3) {
        SendCommand_Type3(motor_index);
    } else if (type == 4) {
        SendCommand_Type4(motor_index);
    } else if (type == 5) {
        SendCommand_Type5(motor_index);
    } else if (type == 6) {
        SendCommand_Type6(motor_index);
    }
}

void DeviceX::QueryPos(int& motor_index) {
    if (!p_motors_data || motor_index < 0 || motor_index >= p_motors_data->size()) return;

    int type = (*p_motors_data)[motor_index].info.api_type;

    if (type == 1) {
        QueryPos_Type1(motor_index);
    } else if (type == 2) {
        QueryPos_Type2(motor_index);
    } else if (type == 3) {
        QueryPos_Type3(motor_index);
    } else if (type == 4) {
        QueryPos_Type4(motor_index);
    } else if (type == 5) {
        QueryPos_Type5(motor_index);
    } else if (type == 6) {
        QueryPos_Type6(motor_index);
    }
}

int DeviceX::setType5PIRegs(int& motor_index, uint8_t p_addr, uint32_t kp, uint8_t i_addr, uint32_t ki, bool save) {
    if (!p_motors_data || motor_index < 0 || motor_index >= (int)p_motors_data->size()) {
        return -1;
    }

    const auto& motor = (*p_motors_data)[motor_index];
    if (motor.info.api_type != 5) {
        return -2;
    }

    const uint8_t chan = (uint8_t)motor.info.chan;
    const uint16_t canid = (uint16_t)motor.info.canid;

    sendType5Write(chan, canid, p_addr, static_cast<int32_t>(kp), false);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    sendType5Write(chan, canid, i_addr, static_cast<int32_t>(ki), false);

    if (save) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        sendType5Write(chan, canid, TYPE5_ADDR_SAVE, 1, false);
    }

    return 0;
}

int DeviceX::SetType5CurrentPI(int& motor_index, uint32_t kp, uint32_t ki, bool save) {
    return setType5PIRegs(motor_index, TYPE5_ADDR_CURRENT_P, kp, TYPE5_ADDR_CURRENT_I, ki, save);
}

int DeviceX::SetType5SpeedPI(int& motor_index, uint32_t kp, uint32_t ki, bool save) {
    return setType5PIRegs(motor_index, TYPE5_ADDR_SPEED_P, kp, TYPE5_ADDR_SPEED_I, ki, save);
}

int DeviceX::SetType5PositionPI(int& motor_index, uint32_t kp, uint32_t ki, bool save) {
    return setType5PIRegs(motor_index, TYPE5_ADDR_POSITION_P, kp, TYPE5_ADDR_POSITION_I, ki, save);
}

int DeviceX::SaveType5Params(int& motor_index) {
    if (!p_motors_data || motor_index < 0 || motor_index >= (int)p_motors_data->size()) {
        return -1;
    }

    const auto& motor = (*p_motors_data)[motor_index];
    if (motor.info.api_type != 5) {
        return -2;
    }

    sendType5Write((uint8_t)motor.info.chan, (uint16_t)motor.info.canid, TYPE5_ADDR_SAVE, 1, false);
    return 0;
}

// =========================================================
//  Type 1 (灵足/LimX) 私有逻辑实现
//  (完全复制自原 DeviceX)
// =========================================================

void DeviceX::EnableMotor_Type1(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    const Motor_CAN_Struct& motor = (*p_motors_data)[motor_index];

    int channel = motor.info.chan;
    int id = motor.info.canid; 

    std::cout << "========================================" << std::endl;
    std::cout << "[Motor Enable Type1] Sending Enable Command..." << std::endl;
    std::cout << " - Motor Name   : " << info.name << std::endl;
    std::cout << " - Motor Number : " << info.num << std::endl;
    std::cout << " - Motor Index : " << motor_index << std::endl;
    std::cout << " - Device Index  : " << info.device_index << std::endl;
    std::cout << " - Using Device Index : " << this->device_global_index << std::endl;
    std::cout << " - CAN Channel   : " << channel << std::endl;
    std::cout << " - CAN ID (Hex) : 0x" << std::hex << (int)id << std::dec << std::endl;
    std::cout << "========================================" << std::endl;
    
    uint8_t data[8] = {0};
    sendRawFrame(info.chan, 3, 0xFD, info.canid, data);
}

void DeviceX::DisableMotor_Type1(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    uint8_t data[8] = {0}; // data[0]=0 仅停止
    sendRawFrame(info.chan, 4, 0xFD, info.canid, data);
    std::cout << "[LK] Disabled Motor ID: " << motor_index << std::endl;

}

void DeviceX::ClearError_Type1(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    uint8_t data[8] = {0};
    data[0] = 1; // Byte[0]=1 表示清除故障
    sendRawFrame(info.chan, 4, 0xFD, info.canid, data);
    std::cout << "[Motor] Clear Error Sent to: " << info.name << std::endl;
}

void DeviceX::SetMode_Type1(int& motor_index, int mode) {
    const auto& info = (*p_motors_data)[motor_index].info;
    
    uint8_t data[8] = {0};
    uint16_t index = 0x7005; // run_mode 寄存器地址
    uint8_t mode_val = (uint8_t)mode;
    
    memcpy(&data[0], &index, 2);
    memcpy(&data[4], &mode_val, 1);
    
    sendRawFrame(info.chan, 18, 0xFD, info.canid, data);
    std::cout << "[Motor] Mode Change to " << mode << " for: " << info.name << std::endl;
}

void DeviceX::SendCommand_Type1(int& motor_index) {
    const Motor_CAN_Struct& motor = (*p_motors_data)[motor_index];
    const auto& cmd = motor.send;
    const auto& info = motor.info;
    uint8_t data[8] = {0};
    
    // 1. ID 中的 DataArea 承载扭矩
    uint16_t t_int = float_to_uint(cmd.torque, info.t_min, info.t_max, 16);
    // 2. Data 域承载 P, V, Kp, Kd
    int p_int  = float_to_uint(cmd.position, info.p_min, info.p_max, 16);
    int v_int  = float_to_uint(cmd.speed, info.v_min, info.v_max, 16);
    int kp_int = float_to_uint(cmd.kp, info.kp_min, info.kp_max, 16);
    int kd_int = float_to_uint(cmd.kd, info.kd_min, info.kd_max, 16);

    // 高字节在前
    data[0] = p_int >> 8;  data[1] = p_int & 0xFF;
    data[2] = v_int >> 8;  data[3] = v_int & 0xFF;
    data[4] = kp_int >> 8; data[5] = kp_int & 0xFF;
    data[6] = kd_int >> 8; data[7] = kd_int & 0xFF;
    
    sendRawFrame(info.chan, 1, t_int, info.canid, data);
}
void DeviceX::QueryPos_Type1(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    
    uint8_t data[8] = {0};
    
    // 参数 Index: 0x7019 (mechPos 负载端计圈机械角度)
    // 协议要求：低字节在前
    data[0] = 0x19;
    data[1] = 0x70;
    // data[2]~data[7] 保持为 0
    
    // 发送指令：
    // Type: 17 (0x11) 单个参数读取
    // Data Area: 0xFD (主机 ID)
    // Motor ID: info.canid
    sendRawFrame(info.chan, 17, 0xFD, info.canid, data);
    
    // std::cout << "[Motor] Query Pos Sent to: " << info.name << std::endl;
}
void DeviceX::sendRawFrame(uint8_t chan, uint32_t type, uint16_t data_area, uint8_t motor_id, uint8_t* data) {
    FrameInfo txInfo;
    txInfo.canID = ((type & 0x1F) << 24) | ((data_area & 0xFFFF) << 8) | (motor_id & 0xFF);
    txInfo.frameType = EXTENDED;
    txInfo.dataLength = 8;
    sendUSBCAN(this->device_handle, chan, &txInfo, data);
}

void DeviceX::sendType5Write(uint8_t chan, uint16_t canid, uint8_t addr, int32_t value, bool fast_write) {
    FrameInfo txMsg = {};
    txMsg.frameType = STANDARD;
    txMsg.dataLength = 8;
    txMsg.canID = canid;

    uint8_t data[8] = {0};
    data[0] = fast_write ? TYPE5_CMD_FAST_WRITE : TYPE5_CMD_WRITE;
    data[1] = addr;
    type5WriteBeI32(&data[2], value);

    sendUSBCAN(this->device_handle, chan, &txMsg, data);
}

void DeviceX::sendType5Read(uint8_t chan, uint16_t canid, uint8_t addr) {
    FrameInfo txMsg = {};
    txMsg.frameType = STANDARD;
    txMsg.dataLength = 8;
    txMsg.canID = canid;

    uint8_t data[8] = {0};
    data[0] = TYPE5_CMD_READ;
    data[1] = addr;

    sendUSBCAN(this->device_handle, chan, &txMsg, data);
}

// =========================================================
//  Type 2 (LK/宇树) 私有逻辑实现
//  (完全复制自原 Device_Type2, 仅改为私有成员函数)
// =========================================================

void DeviceX::EnableMotor_Type2(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    const Motor_CAN_Struct& motor = (*p_motors_data)[motor_index];
    int channel = motor.info.chan;
    int id = motor.info.canid; 


    std::cout << "========================================" << std::endl;
    std::cout << "[Motor Enable Type1] Sending Enable Command..." << std::endl;
    std::cout << " - Motor Name   : " << info.name << std::endl;
    std::cout << " - Motor Number : " << info.num << std::endl;
    std::cout << " - Motor Index : " << motor_index << std::endl;
    std::cout << " - Device Index  : " << info.device_index << std::endl;
    std::cout << " - Using Device Index : " << this->device_global_index << std::endl;
    std::cout << " - CAN Channel   : " << channel << std::endl;
    std::cout << " - CAN ID (Hex) : 0x" << std::hex << (int)id << std::dec << std::endl;
    std::cout << "========================================" << std::endl;
    

    FrameInfo txMsg = {0}; 
    txMsg.frameType = STANDARD;
    txMsg.dataLength = 8;
    txMsg.canID = 0x140 + motor.info.canid;

    uint8_t data[8] = {0};
    data[0] = 0x88; // LK 使能指令

    if (sendUSBCAN(this->device_handle, (uint8_t)motor.info.chan, &txMsg, data) == 1) {
        std::cout << "[LK] Enabled Motor ID: " << motor_index << std::endl;
    } else {
        std::cerr << "[LK] Enable Failed ID: " << motor_index << std::endl;
    }
}

void DeviceX::DisableMotor_Type2(int& motor_index) {
    const Motor_CAN_Struct& motor = (*p_motors_data)[motor_index];

    FrameInfo txMsg = {0};
    txMsg.frameType = STANDARD;
    txMsg.dataLength = 8;
    txMsg.canID = 0x140 + motor.info.canid;

    uint8_t data[8] = {0};
    data[0] = 0x80; 

    sendUSBCAN(this->device_handle, (uint8_t)motor.info.chan, &txMsg, data);
    std::cout << "[LK] Disabled Motor ID: " << motor_index << std::endl;
}

void DeviceX::ClearError_Type2(int& motor_index) {
    const Motor_CAN_Struct& motor = (*p_motors_data)[motor_index];
    std::cout << "[INFO][LK] ClearError (Reset) -> Motor: " << motor.info.name << std::endl;
    
    // 直接调用 Type2 的 Disable 逻辑
    DisableMotor_Type2(motor_index);
}

void DeviceX::SetMode_Type2(int& motor_index, int mode) {
    // Type 2 仅更新本地 struct，不发送指令
    (*p_motors_data)[motor_index].send.mode = (uint8_t)mode;
}

void DeviceX::SendCommand_Type2(int& motor_index) {
    const Motor_CAN_Struct &motor = (*p_motors_data)[motor_index];
    int id = motor.info.canid;
    uint8_t current_mode = motor.send.mode;

    FrameInfo txMsg = {};
    txMsg.frameType = STANDARD; 
    txMsg.dataLength = 8;
    txMsg.canID = 0x140 + id; 

    uint8_t data[8] = {0};

    // --- Mode 0: MIT / Torque Control ---
    if (current_mode == 0 || current_mode == 3) {
        float current_p = motor.recv.current_position_f.load();
        float current_v = motor.recv.current_speed_f.load();
        
        float kp = motor.send.kp;
        float kd = motor.send.kd;
        float target_p = motor.send.position;
        float target_v = motor.send.speed;
        float t_ff = motor.send.torque;

        float iq_f = kp * (target_p - current_p) + kd * (target_v - current_v) + t_ff;
        int16_t iqControl = (int16_t)(iq_f * LK_CURRENT_SEND_FACTOR);

        if (iqControl > 2048) iqControl = 2048;
        if (iqControl < -2048) iqControl = -2048;

        data[0] = 0xA1;
        memcpy(&data[4], &iqControl, 2); 
    }
    // --- Mode 2: Position Control ---
    else if (current_mode == 1) {
        float target_rad = motor.send.position;
        int32_t angleControl = (int32_t)(target_rad * LK_POS_SEND_FACTOR);

        data[0] = 0xA3;
        memcpy(&data[4], &angleControl, 4);
    }
    // --- Mode 3: Speed Control ---
    else if (current_mode == 2) {
        float target_vel_rad = motor.send.speed;
        int32_t speedControl = (int32_t)(target_vel_rad * LK_SPEED_SEND_FACTOR);
        int16_t iqLimit = 2000; 

        data[0] = 0xA2;
        memcpy(&data[2], &iqLimit, 2);
        memcpy(&data[4], &speedControl, 4);
    }
    // --- Default: Read Status ---
    else {
        data[0] = 0x9C;
    }

    sendUSBCAN(this->device_handle, (uint8_t)motor.info.chan, &txMsg, data);
}

// =========================================================
//  Type 2 (LK/宇树/灵康) 私有逻辑实现 - 补全
// =========================================================

void DeviceX::QueryPos_Type2(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;

    // 构造 FrameInfo
    FrameInfo txMsg = {0};
    txMsg.frameType = STANDARD; // 标准帧
    txMsg.dataLength = 8;
    // 根据文档：命令报文标识符：0x140 + ID(1~32)
    txMsg.canID = 0x140 + info.canid;

    uint8_t data[8] = {0};
    
    // 根据文档 Section 3: 读取电机状态 2 命令
    // Data[0] = 0x9C
    // Data[1]~Data[7] = 0x00
    data[0] = 0x9C; 

    // 发送指令
    // 注意：Type2 在您之前的代码中是直接调用 sendUSBCAN，未经过 sendRawFrame 封装
    sendUSBCAN(this->device_handle, (uint8_t)info.chan, &txMsg, data);

    // std::cout << "[LK] Query Pos (0x9C) Sent to ID: " << info.canid << std::endl;
}

// =========================================================
//  Type 3 (达妙/DM) 私有逻辑实现
//  协议参考: 调试助手使用说明书（达妙驱动控制协议）V1.4
// =========================================================

void DeviceX::EnableMotor_Type3(int& motor_index) {
    const auto& motor = (*p_motors_data)[motor_index];
    FrameInfo txMsg = {};
    txMsg.frameType = STANDARD;
    txMsg.dataLength = 8;

    uint16_t canid = (uint16_t)motor.info.canid;
    uint8_t mode = motor.send.mode;
    if (mode == 1) txMsg.canID = 0x100 + canid;       // PV
    else if (mode == 2) txMsg.canID = 0x200 + canid;  // Vel
    else txMsg.canID = canid;                          // MIT / Torque(MIT)

    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    sendUSBCAN(this->device_handle, (uint8_t)motor.info.chan, &txMsg, data);
}

void DeviceX::DisableMotor_Type3(int& motor_index) {
    const auto& motor = (*p_motors_data)[motor_index];
    FrameInfo txMsg = {};
    txMsg.frameType = STANDARD;
    txMsg.dataLength = 8;

    uint16_t canid = (uint16_t)motor.info.canid;
    uint8_t mode = motor.send.mode;
    if (mode == 1) txMsg.canID = 0x100 + canid;
    else if (mode == 2) txMsg.canID = 0x200 + canid;
    else txMsg.canID = canid;

    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    sendUSBCAN(this->device_handle, (uint8_t)motor.info.chan, &txMsg, data);
}

void DeviceX::ClearError_Type3(int& motor_index) {
    const auto& motor = (*p_motors_data)[motor_index];
    FrameInfo txMsg = {};
    txMsg.frameType = STANDARD;
    txMsg.dataLength = 8;

    uint16_t canid = (uint16_t)motor.info.canid;
    uint8_t mode = motor.send.mode;
    if (mode == 1) txMsg.canID = 0x100 + canid;
    else if (mode == 2) txMsg.canID = 0x200 + canid;
    else txMsg.canID = canid;

    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
    sendUSBCAN(this->device_handle, (uint8_t)motor.info.chan, &txMsg, data);
}

void DeviceX::SetMode_Type3(int& motor_index, int mode) {
    // 统一接口: 0-MIT, 1-PV, 2-Vel, 3-Torque(走MIT帧)
    if (mode < 0) mode = 0;
    if (mode > 3) mode = 3;
    (*p_motors_data)[motor_index].send.mode = (uint8_t)mode;
}

void DeviceX::SendCommand_Type3(int& motor_index) {
    const Motor_CAN_Struct& motor = (*p_motors_data)[motor_index];
    const auto& cmd = motor.send;
    const auto& info = motor.info;

    FrameInfo txMsg = {};
    txMsg.frameType = STANDARD;
    uint8_t data[8] = {0};

    const float kp_min = (info.kp_max > info.kp_min) ? info.kp_min : 0.0f;
    const float kp_max = (info.kp_max > info.kp_min) ? info.kp_max : 500.0f;
    const float kd_min = (info.kd_max > info.kd_min) ? info.kd_min : 0.0f;
    const float kd_max = (info.kd_max > info.kd_min) ? info.kd_max : 5.0f;

    // mode: 0-MIT, 1-PV, 2-Vel, 3-Torque(MIT)
    if (cmd.mode == 0 || cmd.mode == 3) {
        const bool torque_only = (cmd.mode == 3);

        uint16_t p_int = (uint16_t)float_to_uint(cmd.position, info.p_min, info.p_max, 16);
        uint16_t v_int = (uint16_t)float_to_uint(cmd.speed, info.v_min, info.v_max, 12);
        uint16_t kp_int = (uint16_t)float_to_uint(torque_only ? 0.0f : cmd.kp, kp_min, kp_max, 12);
        uint16_t kd_int = (uint16_t)float_to_uint(torque_only ? 0.0f : cmd.kd, kd_min, kd_max, 12);
        uint16_t t_int = (uint16_t)float_to_uint(cmd.torque, info.t_min, info.t_max, 12);

        txMsg.canID = (uint16_t)info.canid;
        txMsg.dataLength = 8;
        data[0] = (uint8_t)(p_int >> 8);
        data[1] = (uint8_t)(p_int & 0xFF);
        data[2] = (uint8_t)(v_int >> 4);
        data[3] = (uint8_t)(((v_int & 0x0F) << 4) | (kp_int >> 8));
        data[4] = (uint8_t)(kp_int & 0xFF);
        data[5] = (uint8_t)(kd_int >> 4);
        data[6] = (uint8_t)(((kd_int & 0x0F) << 4) | (t_int >> 8));
        data[7] = (uint8_t)(t_int & 0xFF);
    } else if (cmd.mode == 1) {
        // PV: ID = 0x100 + ID, Data = float pos + float vel (LE)
        float p = cmd.position;
        float v = cmd.speed;
        txMsg.canID = 0x100 + (uint16_t)info.canid;
        txMsg.dataLength = 8;
        memcpy(&data[0], &p, 4);
        memcpy(&data[4], &v, 4);
    } else if (cmd.mode == 2) {
        // Vel: ID = 0x200 + ID, Data = float vel (LE), DLC=4
        float v = cmd.speed;
        txMsg.canID = 0x200 + (uint16_t)info.canid;
        txMsg.dataLength = 4;
        memcpy(&data[0], &v, 4);
    } else {
        return;
    }

    sendUSBCAN(this->device_handle, (uint8_t)info.chan, &txMsg, data);
}

void DeviceX::QueryPos_Type3(int& motor_index) {
    (void)motor_index;
    // DM 协议未定义独立“读位置”命令；位置由反馈帧统一上报。
    // 为避免 QueryPos 触发实际控制，这里不主动发控制帧。
}

// =========================================================
//  Type 4 (RoboMaster C620) 私有逻辑实现
//  协议参考: RoboMaster C620无刷电机调速器使用说明 V1.01
// =========================================================

void DeviceX::EnableMotor_Type4(int& motor_index) {
    // C620 无独立使能帧，发送控制电流即可；这里保持接口一致，仅做占位。
    (*p_motors_data)[motor_index].send.mode = 0;
}

void DeviceX::DisableMotor_Type4(int& motor_index) {
    // 发送该电机所属分组的零电流帧，相当于失能输出。
    const auto& motor = (*p_motors_data)[motor_index];
    const int id = motor.info.canid;
    if (id < 1 || id > 8) return;

    FrameInfo txMsg = {};
    txMsg.frameType = STANDARD;
    txMsg.dataLength = 8;
    txMsg.canID = (id <= 4) ? 0x200 : 0x1FF;

    uint8_t data[8] = {0};
    sendUSBCAN(this->device_handle, (uint8_t)motor.info.chan, &txMsg, data);
}

void DeviceX::ClearError_Type4(int& motor_index) {
    (void)motor_index;
    // C620 协议无清错控制帧，保持为 no-op。
}

void DeviceX::SetMode_Type4(int& motor_index, int mode) {
    // C620 CAN 仅定义电流控制帧，保留 mode 供上层兼容。
    (*p_motors_data)[motor_index].send.mode = (uint8_t)mode;
}

void DeviceX::SendCommand_Type4(int& motor_index) {
    const auto& motor = (*p_motors_data)[motor_index];
    const int id = motor.info.canid;
    if (id < 1 || id > 8) return;

    const uint32_t base_id = (id <= 4) ? 0x200 : 0x1FF;

    FrameInfo txMsg = {};
    txMsg.frameType = STANDARD;
    txMsg.dataLength = 8;
    txMsg.canID = base_id;

    uint8_t data[8] = {0};

    // C620 一帧控制同组4个ID；这里从全局电机池收集同设备同通道同Type4同分组的指令
    // 避免同组其他电机被置零。
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

    sendUSBCAN(this->device_handle, (uint8_t)motor.info.chan, &txMsg, data);
}

void DeviceX::QueryPos_Type4(int& motor_index) {
    (void)motor_index;
    // C620 反馈帧默认周期上报（1KHz，可配置），无需主动查询帧。
}

// =========================================================
//  Type 5 (意优 PP11 / 行星 V3) 私有逻辑实现
//  协议参考: 伺服关节-CAN总线协议 V2 + 用户手册-行星V3
// =========================================================

void DeviceX::EnableMotor_Type5(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    sendType5Write((uint8_t)info.chan, (uint16_t)info.canid, TYPE5_ADDR_ENABLE, 1, false);
}

void DeviceX::DisableMotor_Type5(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    sendType5Write((uint8_t)info.chan, (uint16_t)info.canid, TYPE5_ADDR_ENABLE, 0, false);
}

void DeviceX::ClearError_Type5(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;

    sendType5Write((uint8_t)info.chan, (uint16_t)info.canid, TYPE5_ADDR_STOP, 1, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    sendType5Write((uint8_t)info.chan, (uint16_t)info.canid, TYPE5_ADDR_ENABLE, 0, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    sendType5Write((uint8_t)info.chan, (uint16_t)info.canid, TYPE5_ADDR_ENABLE, 1, false);
}

void DeviceX::SetMode_Type5(int& motor_index, int mode) {
    auto& motor = (*p_motors_data)[motor_index];
    motor.send.mode = (uint8_t)mode;

    const int raw_mode = type5MapMode(motor.info, mode);
    sendType5Write((uint8_t)motor.info.chan, (uint16_t)motor.info.canid, TYPE5_ADDR_MODE, raw_mode, false);
}

void DeviceX::SendCommand_Type5(int& motor_index) {
    const auto& motor = (*p_motors_data)[motor_index];
    const auto& info = motor.info;
    const uint8_t chan = (uint8_t)info.chan;
    const uint16_t canid = (uint16_t)info.canid;
    const bool fast_write = info.type5_fast_write;
    const uint8_t mode = motor.send.mode;

    if (mode == 2) {
        const float torque_limit_nm = type5DefaultTorqueLimitNm(info, motor.send.torque);
        sendType5Write(chan, canid, TYPE5_ADDR_TARGET_TORQUE,
                       type5TorqueNmToRaw(info, torque_limit_nm), fast_write);
        sendType5Write(chan, canid, TYPE5_ADDR_TARGET_SPEED,
                       type5VelRadToRaw(info, motor.send.speed), fast_write);
        return;
    }

    if (mode == 1) {
        const float profile_velocity = type5DefaultProfileVelocity(info, motor.send.speed);
        const float profile_acc = type5DefaultProfileAccel(info, profile_velocity);
        const float profile_dec = type5DefaultProfileDecel(info, profile_velocity);
        const float torque_limit_nm = type5DefaultTorqueLimitNm(info, motor.send.torque);

        sendType5Write(chan, canid, TYPE5_ADDR_TARGET_TORQUE,
                       type5TorqueNmToRaw(info, torque_limit_nm), fast_write);
        sendType5Write(chan, canid, TYPE5_ADDR_TARGET_SPEED,
                       type5VelRadToRaw(info, profile_velocity), fast_write);
        sendType5Write(chan, canid, TYPE5_ADDR_TARGET_ACC,
                       type5VelRadToRaw(info, profile_acc), fast_write);
        sendType5Write(chan, canid, TYPE5_ADDR_TARGET_DEC,
                       type5VelRadToRaw(info, profile_dec), fast_write);
        sendType5Write(chan, canid, TYPE5_ADDR_TARGET_POS,
                       type5PosRadToRaw(info, motor.send.position), fast_write);
        return;
    }

    sendType5Write(chan, canid, TYPE5_ADDR_TARGET_TORQUE,
                   type5TorqueNmToRaw(info, motor.send.torque), fast_write);
}

void DeviceX::QueryPos_Type5(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    sendType5Read((uint8_t)info.chan, (uint16_t)info.canid, TYPE5_ADDR_POS_NOW);
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    sendType5Read((uint8_t)info.chan, (uint16_t)info.canid, TYPE5_ADDR_SPEED_NOW);
}

// =========================================================
//  Type 6 (ENCOS) 私有逻辑实现
//  协议参考: ENCOS 调试技术手册 V1.15 + 电机数据手册 V3.11
// =========================================================

void DeviceX::EnableMotor_Type6(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;

    FrameInfo txMsg = {};
    txMsg.frameType = STANDARD;
    txMsg.dataLength = 2;
    txMsg.canID = static_cast<uint16_t>(info.canid);

    uint8_t data[8] = {0};
    data[0] = static_cast<uint8_t>(TYPE6_MODE_BRAKE << 5);
    data[1] = 1; // 释放抱闸
    sendUSBCAN(this->device_handle, static_cast<uint8_t>(info.chan), &txMsg, data);

    std::this_thread::sleep_for(std::chrono::milliseconds(2));

    txMsg.dataLength = 3;
    data[0] = type6PackModeStateAck(TYPE6_MODE_CUR_TOR, TYPE6_CUR_STATE_CURRENT, TYPE6_ACK_NONE);
    data[1] = 0;
    data[2] = 0;
    sendUSBCAN(this->device_handle, static_cast<uint8_t>(info.chan), &txMsg, data);
}

void DeviceX::DisableMotor_Type6(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;

    FrameInfo txMsg = {};
    txMsg.frameType = STANDARD;
    txMsg.dataLength = 3;
    txMsg.canID = static_cast<uint16_t>(info.canid);

    uint8_t data[8] = {0};
    data[0] = type6PackModeStateAck(TYPE6_MODE_CUR_TOR, TYPE6_CUR_STATE_DAMPING, TYPE6_ACK_NONE);
    data[1] = 0;
    data[2] = 0;
    sendUSBCAN(this->device_handle, static_cast<uint8_t>(info.chan), &txMsg, data);
}

void DeviceX::ClearError_Type6(int& motor_index) {
    // ENCOS 协议无显式“清错”控制字，发送零电流并查询状态。
    DisableMotor_Type6(motor_index);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    QueryPos_Type6(motor_index);
}

void DeviceX::SetMode_Type6(int& motor_index, int mode) {
    if (mode < 0) mode = 0;
    if (mode > 3) mode = 3;
    (*p_motors_data)[motor_index].send.mode = static_cast<uint8_t>(mode);
}

void DeviceX::SendCommand_Type6(int& motor_index) {
    const auto& motor = (*p_motors_data)[motor_index];
    const auto& info = motor.info;
    const auto& cmd = motor.send;

    FrameInfo txMsg = {};
    txMsg.frameType = STANDARD;
    txMsg.canID = static_cast<uint16_t>(info.canid);

    uint8_t data[8] = {0};

    // mode: 0-力位混控, 1-位置, 2-速度, 3-力矩(走模式3的“力矩控制子模式”)
    if (cmd.mode == 0) {
        const uint16_t kp_raw = type6KpToRaw(info, cmd.kp);
        const uint16_t kd_raw = type6KdToRaw(info, cmd.kd);
        const uint16_t pos_raw = type6PosToRaw(info, cmd.position);
        const uint16_t spd_raw = type6SpdToRaw(info, cmd.speed);
        const uint16_t tor_raw = type6TorToRaw(info, cmd.torque);

        txMsg.dataLength = 8;
        type6PackMitCmd(data, kp_raw, kd_raw, pos_raw, spd_raw, tor_raw);
        sendUSBCAN(this->device_handle, static_cast<uint8_t>(info.chan), &txMsg, data);
        return;
    }

    if (cmd.mode == 1) {
        const float pos_deg = cmd.position * RAD_TO_DEG;
        const float current_limit_a = (std::fabs(cmd.torque) > 1e-6f)
            ? std::fabs(cmd.torque)
            : TYPE6_DEFAULT_CURRENT_LIMIT_A;
        const uint16_t speed_limit_raw = type6SpeedLimitToRaw15(cmd.speed);
        const uint16_t current_limit_raw = type6CurrentLimitToRaw12(current_limit_a);

        txMsg.dataLength = 8;
        type6PackPositionCmd(data, pos_deg, speed_limit_raw, current_limit_raw, TYPE6_ACK_TYPE2);
        sendUSBCAN(this->device_handle, static_cast<uint8_t>(info.chan), &txMsg, data);
        return;
    }

    if (cmd.mode == 2) {
        const float current_limit_a = (std::fabs(cmd.torque) > 1e-6f)
            ? std::fabs(cmd.torque)
            : TYPE6_DEFAULT_CURRENT_LIMIT_A;
        const float target_speed_rpm = cmd.speed * TYPE6_RAD_TO_RPM;

        txMsg.dataLength = 7;
        data[0] = type6PackModeStateAck(TYPE6_MODE_SPEED, 0, TYPE6_ACK_TYPE3);
        type6WriteBeF32(&data[1], target_speed_rpm);
        type6WriteBeU16(&data[5], type6CurrentLimitToRaw16(current_limit_a));
        sendUSBCAN(this->device_handle, static_cast<uint8_t>(info.chan), &txMsg, data);
        return;
    }

    float torque_cmd = cmd.torque;
    if (info.t_max > info.t_min) {
        torque_cmd = type6Clip(torque_cmd, info.t_min, info.t_max);
    }

    txMsg.dataLength = 3;
    data[0] = type6PackModeStateAck(TYPE6_MODE_CUR_TOR, TYPE6_CUR_STATE_TORQUE, TYPE6_ACK_TYPE2);
    type6WriteBeU16(&data[1], static_cast<uint16_t>(type6SignedCmdToRaw(torque_cmd)));
    sendUSBCAN(this->device_handle, static_cast<uint8_t>(info.chan), &txMsg, data);
}

void DeviceX::QueryPos_Type6(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;

    FrameInfo txMsg = {};
    txMsg.frameType = STANDARD;
    txMsg.dataLength = 2;
    txMsg.canID = static_cast<uint16_t>(info.canid);

    uint8_t data[8] = {0};
    data[0] = type6PackModeStateAck(TYPE6_MODE_QUERY, 0, TYPE6_ACK_TYPE1);
    data[1] = 1; // 查询当前位置(角度制)
    sendUSBCAN(this->device_handle, static_cast<uint8_t>(info.chan), &txMsg, data);

    std::this_thread::sleep_for(std::chrono::microseconds(100));

    data[1] = 2; // 查询当前速度(RPM)
    sendUSBCAN(this->device_handle, static_cast<uint8_t>(info.chan), &txMsg, data);
}

// =========================================================
//  Unified Receive Loop (融合版)
// =========================================================

void DeviceX::ReceiveLoop() {
    FrameInfo rxInfo;
    uint8_t rxData[8];
    uint8_t rxChannel = 0;

    while(is_running) {
        // 读取 CAN 帧
        int32_t num = readUSBCAN(this->device_handle, &rxChannel, &rxInfo, rxData, 5000);
        if (num < 0) continue;

        int parsed_motor_id = -1;
        int g_idx = -1;

        // --- Step 1: 根据帧类型解析 ID ---
        if (rxInfo.frameType == EXTENDED) { // Extended Frame -> Type 1
            uint32_t canID = rxInfo.canID;
            parsed_motor_id = (canID >> 8) & 0xFF; 
        }
        else if (rxInfo.frameType == STANDARD) { // Standard Frame -> Type 2 / Type 3
            uint32_t canID = rxInfo.canID;
            if (canID <= 0x7FF) {
                const int direct_idx = p_mapper->get_id({(uint)this->device_global_index, (uint)rxChannel, (uint)canID});
                if (direct_idx != -1 && direct_idx < (int)p_motors_data->size()) {
                    const auto& mapped_motor = (*p_motors_data)[direct_idx];
                    if (mapped_motor.info.api_type == 5 || mapped_motor.info.api_type == 6) {
                        parsed_motor_id = (int)canID;
                        g_idx = direct_idx;
                    }
                }
            }

            if (g_idx == -1 && canID >= 0x140 && canID < 0x160) {
                parsed_motor_id = canID - 0x140;
            } else if (g_idx == -1 && canID >= 0x180 && canID < 0x1A0) {
                parsed_motor_id = canID - 0x180;
            } else if (g_idx == -1 && canID >= 0x201 && canID <= 0x208) {
                // Type 4(C620) 反馈帧: 0x200 + ID
                parsed_motor_id = canID - 0x200;
            } else if (g_idx == -1) {
                // Type 3(DM) 反馈帧中，ID 在 Data[0] 低4位：ID|ERR<<4
                parsed_motor_id = rxData[0] & 0x0F;
            }
        }

        // --- Step 2: 查找 Mapper ---
        if (parsed_motor_id == -1) continue;
        if (g_idx == -1) {
            g_idx = p_mapper->get_id({(uint)this->device_global_index, (uint)rxChannel, (uint)parsed_motor_id});
        }

        // --- Step 3: 根据 API Type 分发解析 ---
        if (g_idx != -1 && g_idx < (int)p_motors_data->size()) {
            Motor_CAN_Struct& motor = (*p_motors_data)[g_idx];

            // >>> TYPE 1 Logic (LimX) >>>
            if (motor.info.api_type == 1) { 
                uint32_t canID = rxInfo.canID;
                uint8_t type_field = (canID >> 24) & 0x1F;

                if (type_field == 2) { 
                    uint16_t p_int = (rxData[0] << 8) | rxData[1];
                    uint16_t v_int = (rxData[2] << 8) | rxData[3];
                    uint16_t t_int = (rxData[4] << 8) | rxData[5];
                    uint16_t temp_int = (rxData[6] << 8) | rxData[7];

                    motor.recv.current_position_f.store(uint_to_float(p_int, motor.info.p_min, motor.info.p_max, 16));
                    motor.recv.current_speed_f.store(uint_to_float(v_int, motor.info.v_min, motor.info.v_max, 16));
                    motor.recv.current_torque_f.store(uint_to_float(t_int, motor.info.t_min, motor.info.t_max, 16));
                    motor.recv.current_temp_f.store((float)temp_int / 10.0f);

                    motor.recv.fault_message = (canID >> 16) & 0x3F;
                    motor.recv.mode = (canID >> 22) & 0x03;
                }
                else if (type_field == 17) {
                    // 校验 Index 是否为我们查询的 0x7019 (低字节在前: Data[0]=0x19, Data[1]=0x70)
                    if (rxData[0] == 0x19 && rxData[1] == 0x70) {
                        float temp_val;
                        // 数据在 Byte4~7，低字节在前 (Little Endian)
                        memcpy(&temp_val, &rxData[4], 4);
                        
                        // 更新位置 (0x7019 是 mechPos，单位 rad)
                        motor.recv.current_position_f.store(temp_val);
                        
                        // 由于 Type 17 不包含速度和力矩，这里只更新位置
                    }
                }
                else if (type_field == 21) { 
                    uint32_t fault_word;
                    memcpy(&fault_word, &rxData[0], 4);
                    motor.recv.fault_message = (uint8_t)fault_word;
                }
            }
            // >>> TYPE 2 Logic (LK) >>>
            else if (motor.info.api_type == 2) {
                uint8_t status = rxData[0];
                if (status == 0x9C || status == 0xA1 || status == 0xA2 || status == 0xA3) {
                    int8_t temp = (int8_t)rxData[1];
                    int16_t iq_raw = (int16_t)(rxData[2] | (rxData[3] << 8));
                    int16_t spd_raw = (int16_t)(rxData[4] | (rxData[5] << 8));
                    uint16_t pos_raw = (uint16_t)(rxData[6] | (rxData[7] << 8));

                    motor.recv.current_temp_f.store((float)temp);
                    motor.recv.current_iq_f.store((float)iq_raw * IQ_ROIT); 
                    motor.recv.current_speed_f.store((float)spd_raw / V_ROIT); 
                    motor.recv.current_position_f.store((float)pos_raw * P_RIOT);

                    motor.recv.motor_id = parsed_motor_id;
                    motor.recv.mode = status; 
                    motor.recv.fault_message = 0; 
                }
            }
            // >>> TYPE 3 Logic (DM) >>>
            else if (motor.info.api_type == 3) {
                // 反馈帧格式:
                // D0: ID|ERR<<4
                // D1..D2: Pos(16b), D3..D4[7:4]: Vel(12b), D4[3:0]..D5: Tor(12b)
                // D6: MOS温度, D7: 转子温度
                uint8_t id_and_err = rxData[0];
                uint8_t err = (id_and_err >> 4) & 0x0F;
                uint8_t motor_id = id_and_err & 0x0F;

                uint16_t p_int = (uint16_t)((rxData[1] << 8) | rxData[2]);
                uint16_t v_int = (uint16_t)((rxData[3] << 4) | (rxData[4] >> 4));
                uint16_t t_int = (uint16_t)(((rxData[4] & 0x0F) << 8) | rxData[5]);

                motor.recv.current_position_f.store(uint_to_float((int)p_int, motor.info.p_min, motor.info.p_max, 16));
                motor.recv.current_speed_f.store(uint_to_float((int)v_int, motor.info.v_min, motor.info.v_max, 12));
                motor.recv.current_torque_f.store(uint_to_float((int)t_int, motor.info.t_min, motor.info.t_max, 12));
                motor.recv.current_temp_f.store((float)rxData[6]); // MOS 温度

                motor.recv.motor_id = motor_id;
                motor.recv.fault_message = err;
            }
            // >>> TYPE 4 Logic (RoboMaster C620) >>>
            else if (motor.info.api_type == 4) {
                // 反馈帧: ID=0x200+id, D0-1角度(0~8191), D2-3转速(rpm), D4-5实际转矩电流, D6温度
                uint16_t pos_raw = (uint16_t)((rxData[0] << 8) | rxData[1]);
                int16_t spd_rpm = (int16_t)((rxData[2] << 8) | rxData[3]);
                int16_t iq_raw = (int16_t)((rxData[4] << 8) | rxData[5]);
                uint8_t temp = rxData[6];

                motor.recv.current_position_f.store((float)pos_raw * C620_POS_TO_RAD);
                motor.recv.current_speed_f.store((float)spd_rpm * C620_RPM_TO_RAD_S);
                motor.recv.current_iq_f.store((float)iq_raw * (20.0f / 16384.0f)); // 约等效相电流(A)

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
            // >>> TYPE 5 Logic (意优 PP11 / 行星 V3) >>>
            else if (motor.info.api_type == 5) {
                const uint8_t cmd = rxData[0];
                const uint8_t addr = rxData[1];

                if (cmd == TYPE5_CMD_WRITE_ACK) {
                    motor.recv.motor_id = (uint8_t)parsed_motor_id;
                    motor.recv.motor_state = rxData[2];
                }
                else if (cmd == TYPE5_CMD_READ_ACK) {
                    const int32_t raw_value = type5ReadBeI32(&rxData[2]);
                    motor.recv.motor_id = (uint8_t)parsed_motor_id;

                    switch (addr) {
                    case TYPE5_ADDR_POS_NOW:
                        motor.recv.current_position_f.store(type5RawToPosRad(motor.info, raw_value));
                        break;
                    case TYPE5_ADDR_SPEED_NOW:
                        motor.recv.current_speed_f.store(type5RawToVelRad(motor.info, raw_value));
                        break;
                    case TYPE5_ADDR_TORQUE_NOW: {
                        const float torque_nm = type5RawToTorqueNm(motor.info, raw_value);
                        motor.recv.current_torque_f.store(torque_nm);
                        if (motor.info.type5_torque_constant > 1e-6f) {
                            motor.recv.current_iq_f.store(torque_nm / motor.info.type5_torque_constant);
                        }
                        break;
                    }
                    case TYPE5_ADDR_TEMP:
                        motor.recv.current_temp_f.store((float)raw_value);
                        break;
                    case TYPE5_ADDR_FAULT:
                        motor.recv.fault_message = (uint32_t)raw_value;
                        break;
                    case TYPE5_ADDR_MODE:
                        motor.recv.mode = (uint8_t)raw_value;
                        break;
                    case TYPE5_ADDR_ENABLE:
                        motor.recv.motor_state = (uint8_t)raw_value;
                        break;
                    default:
                        break;
                    }
                }
            }
            // >>> TYPE 6 Logic (ENCOS) >>>
            else if (motor.info.api_type == 6) {
                if (rxInfo.dataLength < 2) {
                    continue;
                }

                const uint8_t fb_type = type6FeedbackType(rxData[0]);
                const uint8_t fb_error = type6FeedbackError(rxData[0]);
                motor.recv.motor_id = (uint8_t)parsed_motor_id;
                motor.recv.fault_message = fb_error;

                if (fb_type == 1) {
                    // 类型1: 位置16 + 速度12 + 电流12 + 电机温度 + MOS温度
                    if (rxInfo.dataLength >= 8) {
                        const uint16_t pos_raw = static_cast<uint16_t>((static_cast<uint16_t>(rxData[1]) << 8) | rxData[2]);
                        const uint16_t spd_raw = static_cast<uint16_t>((static_cast<uint16_t>(rxData[3]) << 4) | (rxData[4] >> 4));
                        const uint16_t cur_raw = static_cast<uint16_t>(((rxData[4] & 0x0F) << 8) | rxData[5]);

                        motor.recv.current_position_f.store(type6RawToPos(motor.info, pos_raw));
                        motor.recv.current_speed_f.store(type6RawToSpd(motor.info, spd_raw));
                        motor.recv.current_iq_f.store(type6RawToCurrentDefault((int)cur_raw));
                        motor.recv.current_temp_f.store(type6TempRawToDeg(rxData[6]));
                    }
                    motor.recv.mode = 1;
                }
                else if (fb_type == 2) {
                    // 类型2: 位置(度,float32) + 电流(int16,0.01A) + 温度
                    if (rxInfo.dataLength >= 8) {
                        const float pos_deg = type6ReadBeF32(&rxData[1]);
                        const int16_t current_raw = type6ReadBeI16(&rxData[5]);

                        motor.recv.current_position_f.store(pos_deg * DEG_TO_RAD);
                        motor.recv.current_iq_f.store(static_cast<float>(current_raw) * 0.01f);
                        motor.recv.current_temp_f.store(type6TempRawToDeg(rxData[7]));
                    }
                    motor.recv.mode = 2;
                }
                else if (fb_type == 3) {
                    // 类型3: 速度(rpm,float32) + 电流(int16,0.01A) + 温度
                    if (rxInfo.dataLength >= 8) {
                        const float speed_rpm = type6ReadBeF32(&rxData[1]);
                        const int16_t current_raw = type6ReadBeI16(&rxData[5]);

                        motor.recv.current_speed_f.store(speed_rpm * TYPE6_RPM_TO_RAD);
                        motor.recv.current_iq_f.store(static_cast<float>(current_raw) * 0.01f);
                        motor.recv.current_temp_f.store(type6TempRawToDeg(rxData[7]));
                    }
                    motor.recv.mode = 3;
                }
                else if (fb_type == 4) {
                    // 类型4: 参数配置返回
                    if (rxInfo.dataLength >= 3) {
                        motor.recv.motor_state = rxData[2];
                    }
                }
                else if (fb_type == 5) {
                    // 类型5: 查询返回（查询码 + 数据）
                    const uint8_t query_code = rxData[1];
                    if (query_code == 1 && rxInfo.dataLength >= 6) {
                        const float pos_deg = type6ReadBeF32(&rxData[2]);
                        motor.recv.current_position_f.store(pos_deg * DEG_TO_RAD);
                    } else if (query_code == 2 && rxInfo.dataLength >= 6) {
                        const float speed_rpm = type6ReadBeF32(&rxData[2]);
                        motor.recv.current_speed_f.store(speed_rpm * TYPE6_RPM_TO_RAD);
                    } else if (query_code == 3 && rxInfo.dataLength >= 6) {
                        const float current_a = type6ReadBeF32(&rxData[2]);
                        motor.recv.current_iq_f.store(current_a);
                    } else if (query_code == 37 && rxInfo.dataLength >= 3) {
                        motor.recv.motor_state = rxData[2];
                    }
                }
                else if (fb_type == 6) {
                    // 类型6: 抱闸状态返回
                    if (rxInfo.dataLength >= 2) {
                        motor.recv.motor_state = rxData[1];
                    }
                }
            }
        }
    }
}
