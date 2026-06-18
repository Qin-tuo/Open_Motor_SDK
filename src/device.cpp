#include "device.hpp"
#include <algorithm>
#include <array>
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
#include <limits>
#include <sstream>
#include <thread>

#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>

namespace encos {

struct Limits {
    float p_min = -12.5f;
    float p_max = 12.5f;
    float v_min = -18.0f;
    float v_max = 18.0f;
    float kp_min = 0.0f;
    float kp_max = 500.0f;
    float kd_min = 0.0f;
    float kd_max = 5.0f;
    float t_min = -30.0f;
    float t_max = 30.0f;
};

struct CommandFrame {
    uint32_t can_id = 0;
    uint8_t dlc = 0;
    std::array<uint8_t, 8> data {};
};

struct Feedback {
    bool valid = false;
    uint8_t type = 0;
    uint8_t error = 0;
    bool has_position = false;
    bool has_speed = false;
    bool has_current = false;
    bool has_temperature = false;
    bool has_state = false;
    float position_rad = 0.0f;
    float speed_rad_s = 0.0f;
    float current_a = 0.0f;
    float temperature_c = 0.0f;
    uint8_t state = 0;
    uint8_t mode = 0;
};

namespace {

constexpr float TWO_PI_LOCAL = 6.28318530718f;
constexpr float RAD_TO_DEG_LOCAL = 57.2957795f;
constexpr float DEG_TO_RAD_LOCAL = 0.017453293f;
constexpr float RAD_TO_RPM = 60.0f / TWO_PI_LOCAL;
constexpr float RPM_TO_RAD = TWO_PI_LOCAL / 60.0f;
constexpr float DEFAULT_CURRENT_LIMIT_A = 10.0f;
constexpr float DEFAULT_CURRENT_FB_MIN_A = -90.0f;
constexpr float DEFAULT_CURRENT_FB_MAX_A = 90.0f;

constexpr uint8_t MODE_MIT = 0x00;
constexpr uint8_t MODE_POSITION = 0x01;
constexpr uint8_t MODE_SPEED = 0x02;
constexpr uint8_t MODE_CUR_TOR = 0x03;
constexpr uint8_t MODE_BRAKE = 0x04;
constexpr uint8_t MODE_QUERY = 0x07;

constexpr uint8_t ACK_NONE = 0;
constexpr uint8_t ACK_TYPE1 = 1;
constexpr uint8_t ACK_TYPE2 = 2;
constexpr uint8_t ACK_TYPE3 = 3;

constexpr uint8_t CUR_STATE_CURRENT = 0;
constexpr uint8_t CUR_STATE_TORQUE = 1;
constexpr uint8_t CUR_STATE_DAMPING = 2;

float clip(float value, float low, float high) {
    return std::max(low, std::min(value, high));
}

void resolve_range(float cfg_min, float cfg_max, float def_min, float def_max,
                   float& out_min, float& out_max) {
    if (cfg_max > cfg_min) {
        out_min = cfg_min;
        out_max = cfg_max;
    } else {
        out_min = def_min;
        out_max = def_max;
    }
}

uint16_t pos_to_raw(const Limits& limits, float pos_rad) {
    float p_min = 0.0f;
    float p_max = 0.0f;
    resolve_range(limits.p_min, limits.p_max, -12.5f, 12.5f, p_min, p_max);
    return static_cast<uint16_t>(float_to_uint(pos_rad, p_min, p_max, 16));
}

float raw_to_pos(const Limits& limits, uint16_t raw) {
    float p_min = 0.0f;
    float p_max = 0.0f;
    resolve_range(limits.p_min, limits.p_max, -12.5f, 12.5f, p_min, p_max);
    return uint_to_float(static_cast<int>(raw), p_min, p_max, 16);
}

uint16_t spd_to_raw(const Limits& limits, float spd_rad_s) {
    float v_min = 0.0f;
    float v_max = 0.0f;
    resolve_range(limits.v_min, limits.v_max, -18.0f, 18.0f, v_min, v_max);
    return static_cast<uint16_t>(float_to_uint(spd_rad_s, v_min, v_max, 12));
}

float raw_to_spd(const Limits& limits, uint16_t raw) {
    float v_min = 0.0f;
    float v_max = 0.0f;
    resolve_range(limits.v_min, limits.v_max, -18.0f, 18.0f, v_min, v_max);
    return uint_to_float(static_cast<int>(raw), v_min, v_max, 12);
}

uint16_t kp_to_raw(const Limits& limits, float kp) {
    float kp_min = 0.0f;
    float kp_max = 0.0f;
    resolve_range(limits.kp_min, limits.kp_max, 0.0f, 500.0f, kp_min, kp_max);
    return static_cast<uint16_t>(float_to_uint(kp, kp_min, kp_max, 12));
}

uint16_t kd_to_raw(const Limits& limits, float kd) {
    float kd_min = 0.0f;
    float kd_max = 0.0f;
    resolve_range(limits.kd_min, limits.kd_max, 0.0f, 5.0f, kd_min, kd_max);
    return static_cast<uint16_t>(float_to_uint(kd, kd_min, kd_max, 9));
}

uint16_t tor_to_raw(const Limits& limits, float tor) {
    float t_min = 0.0f;
    float t_max = 0.0f;
    resolve_range(limits.t_min, limits.t_max, -30.0f, 30.0f, t_min, t_max);
    return static_cast<uint16_t>(float_to_uint(tor, t_min, t_max, 12));
}

uint16_t current_limit_to_raw12(float current_limit_a) {
    const float clipped = clip(current_limit_a, 0.0f, 409.5f);
    return static_cast<uint16_t>(std::lround(clipped * 10.0f));
}

uint16_t current_limit_to_raw16(float current_limit_a) {
    const float clipped = clip(current_limit_a, 0.0f, 6553.5f);
    return static_cast<uint16_t>(std::lround(clipped * 10.0f));
}

uint16_t speed_limit_to_raw15(float speed_rad_s) {
    const float speed_rpm = std::fabs(speed_rad_s) * RAD_TO_RPM;
    const float clipped = clip(speed_rpm, 0.0f, 3276.7f);
    return static_cast<uint16_t>(std::lround(clipped * 10.0f));
}

int16_t signed_cmd_to_raw(float value) {
    const float clipped = clip(value, -327.68f, 327.67f);
    return static_cast<int16_t>(std::lround(clipped * 100.0f));
}

float raw_to_current_default(int raw_u12) {
    return uint_to_float(raw_u12, DEFAULT_CURRENT_FB_MIN_A, DEFAULT_CURRENT_FB_MAX_A, 12);
}

float temp_raw_to_deg(uint8_t raw_temp) {
    return (static_cast<float>(raw_temp) - 50.0f) * 0.5f;
}

void write_be_u16(uint8_t* data, uint16_t value) {
    data[0] = static_cast<uint8_t>((value >> 8) & 0xFF);
    data[1] = static_cast<uint8_t>(value & 0xFF);
}

uint16_t read_be_u16(const uint8_t* data) {
    return static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) |
                                 static_cast<uint16_t>(data[1]));
}

int16_t read_be_i16(const uint8_t* data) {
    return static_cast<int16_t>(read_be_u16(data));
}

void write_be_f32(uint8_t* data, float value) {
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    data[0] = static_cast<uint8_t>((bits >> 24) & 0xFF);
    data[1] = static_cast<uint8_t>((bits >> 16) & 0xFF);
    data[2] = static_cast<uint8_t>((bits >> 8) & 0xFF);
    data[3] = static_cast<uint8_t>(bits & 0xFF);
}

float read_be_f32(const uint8_t* data) {
    const uint32_t bits = (static_cast<uint32_t>(data[0]) << 24) |
                          (static_cast<uint32_t>(data[1]) << 16) |
                          (static_cast<uint32_t>(data[2]) << 8) |
                          static_cast<uint32_t>(data[3]);
    float value = 0.0f;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

uint8_t pack_mode_state_ack(uint8_t mode, uint8_t state, uint8_t ack) {
    return static_cast<uint8_t>(((mode & 0x07) << 5) |
                                ((state & 0x07) << 2) |
                                (ack & 0x03));
}

CommandFrame make_frame(uint32_t can_id, uint8_t dlc) {
    CommandFrame frame {};
    frame.can_id = can_id & 0x7FFU;
    frame.dlc = dlc;
    return frame;
}

}  // namespace

Limits limits_from_info(const Motor_CAN_Info_Struct& info) {
    Limits limits {};
    limits.p_min = info.p_min;
    limits.p_max = info.p_max;
    limits.v_min = info.v_min;
    limits.v_max = info.v_max;
    limits.kp_min = info.kp_min;
    limits.kp_max = info.kp_max;
    limits.kd_min = info.kd_min;
    limits.kd_max = info.kd_max;
    limits.t_min = info.t_min;
    limits.t_max = info.t_max;
    return limits;
}

uint8_t feedback_type(uint8_t first_byte) {
    return static_cast<uint8_t>((first_byte >> 5) & 0x07);
}

uint8_t feedback_error(uint8_t first_byte) {
    return static_cast<uint8_t>(first_byte & 0x1F);
}

CommandFrame make_brake_release_command(uint32_t can_id) {
    CommandFrame frame = make_frame(can_id, 2);
    frame.data[0] = static_cast<uint8_t>(MODE_BRAKE << 5);
    frame.data[1] = 1;
    return frame;
}

CommandFrame make_current_zero_command(uint32_t can_id) {
    CommandFrame frame = make_frame(can_id, 3);
    frame.data[0] = pack_mode_state_ack(MODE_CUR_TOR, CUR_STATE_CURRENT, ACK_NONE);
    return frame;
}

CommandFrame make_set_zero_command(uint32_t motor_id) {
    CommandFrame frame = make_frame(0x7FF, 4);
    frame.data[0] = static_cast<uint8_t>((motor_id >> 8) & 0xFF);
    frame.data[1] = static_cast<uint8_t>(motor_id & 0xFF);
    frame.data[2] = 0x00;
    frame.data[3] = 0x03;
    return frame;
}

CommandFrame make_damping_command(uint32_t can_id) {
    CommandFrame frame = make_frame(can_id, 3);
    frame.data[0] = pack_mode_state_ack(MODE_CUR_TOR, CUR_STATE_DAMPING, ACK_NONE);
    return frame;
}

CommandFrame make_mit_command(uint32_t can_id, const Limits& limits,
                              float position_rad, float speed_rad_s,
                              float kp, float kd, float torque) {
    const uint16_t kp_raw = kp_to_raw(limits, kp);
    const uint16_t kd_raw = kd_to_raw(limits, kd);
    const uint16_t pos_raw = pos_to_raw(limits, position_rad);
    const uint16_t spd_raw = spd_to_raw(limits, speed_rad_s);
    const uint16_t tor_raw = tor_to_raw(limits, torque);

    CommandFrame frame = make_frame(can_id, 8);
    frame.data[0] = static_cast<uint8_t>((MODE_MIT << 5) | ((kp_raw >> 7) & 0x1F));
    frame.data[1] = static_cast<uint8_t>(((kp_raw & 0x7F) << 1) | ((kd_raw >> 8) & 0x01));
    frame.data[2] = static_cast<uint8_t>(kd_raw & 0xFF);
    frame.data[3] = static_cast<uint8_t>((pos_raw >> 8) & 0xFF);
    frame.data[4] = static_cast<uint8_t>(pos_raw & 0xFF);
    frame.data[5] = static_cast<uint8_t>((spd_raw >> 4) & 0xFF);
    frame.data[6] = static_cast<uint8_t>(((spd_raw & 0x0F) << 4) | ((tor_raw >> 8) & 0x0F));
    frame.data[7] = static_cast<uint8_t>(tor_raw & 0xFF);
    return frame;
}

CommandFrame make_position_command(uint32_t can_id, float position_rad,
                                   float speed_limit_rad_s,
                                   float current_limit_a) {
    CommandFrame frame = make_frame(can_id, 8);
    uint8_t pos_data[4] = {0};
    write_be_f32(pos_data, position_rad * RAD_TO_DEG_LOCAL);
    const uint32_t pos_bits = (static_cast<uint32_t>(pos_data[0]) << 24) |
                              (static_cast<uint32_t>(pos_data[1]) << 16) |
                              (static_cast<uint32_t>(pos_data[2]) << 8) |
                              static_cast<uint32_t>(pos_data[3]);
    const uint16_t speed_limit_raw = speed_limit_to_raw15(speed_limit_rad_s);
    const uint16_t current_limit_raw = current_limit_to_raw12(
        (std::fabs(current_limit_a) > 1e-6f) ? std::fabs(current_limit_a) : DEFAULT_CURRENT_LIMIT_A);

    frame.data[0] = static_cast<uint8_t>((MODE_POSITION << 5) | ((pos_bits >> 27) & 0x1F));
    frame.data[1] = static_cast<uint8_t>((pos_bits >> 19) & 0xFF);
    frame.data[2] = static_cast<uint8_t>((pos_bits >> 11) & 0xFF);
    frame.data[3] = static_cast<uint8_t>((pos_bits >> 3) & 0xFF);
    frame.data[4] = static_cast<uint8_t>(((pos_bits & 0x07) << 5) | ((speed_limit_raw >> 10) & 0x1F));
    frame.data[5] = static_cast<uint8_t>((speed_limit_raw >> 2) & 0xFF);
    frame.data[6] = static_cast<uint8_t>(((speed_limit_raw & 0x03) << 6) | ((current_limit_raw >> 6) & 0x3F));
    frame.data[7] = static_cast<uint8_t>(((current_limit_raw & 0x3F) << 2) | (ACK_TYPE2 & 0x03));
    return frame;
}

CommandFrame make_speed_command(uint32_t can_id, float speed_rad_s,
                                float current_limit_a) {
    CommandFrame frame = make_frame(can_id, 7);
    const float target_speed_rpm = speed_rad_s * RAD_TO_RPM;
    const float limit = (std::fabs(current_limit_a) > 1e-6f) ? std::fabs(current_limit_a) : DEFAULT_CURRENT_LIMIT_A;

    frame.data[0] = pack_mode_state_ack(MODE_SPEED, 0, ACK_TYPE3);
    write_be_f32(&frame.data[1], target_speed_rpm);
    write_be_u16(&frame.data[5], current_limit_to_raw16(limit));
    return frame;
}

CommandFrame make_torque_command(uint32_t can_id, const Limits& limits,
                                 float torque) {
    float torque_cmd = torque;
    if (limits.t_max > limits.t_min) {
        torque_cmd = clip(torque_cmd, limits.t_min, limits.t_max);
    }

    CommandFrame frame = make_frame(can_id, 3);
    frame.data[0] = pack_mode_state_ack(MODE_CUR_TOR, CUR_STATE_TORQUE, ACK_TYPE2);
    write_be_u16(&frame.data[1], static_cast<uint16_t>(signed_cmd_to_raw(torque_cmd)));
    return frame;
}

CommandFrame make_query_command(uint32_t can_id, uint8_t query_code) {
    CommandFrame frame = make_frame(can_id, 2);
    frame.data[0] = pack_mode_state_ack(MODE_QUERY, 0, ACK_TYPE1);
    frame.data[1] = query_code;
    return frame;
}

Feedback parse_feedback(const uint8_t* data, uint8_t len, const Limits& limits) {
    Feedback feedback {};
    if (!data || len < 2) {
        return feedback;
    }

    feedback.valid = true;
    feedback.type = feedback_type(data[0]);
    feedback.error = feedback_error(data[0]);

    if (feedback.type == 1) {
        if (len >= 8) {
            const uint16_t pos_raw = static_cast<uint16_t>((static_cast<uint16_t>(data[1]) << 8) | data[2]);
            const uint16_t spd_raw = static_cast<uint16_t>((static_cast<uint16_t>(data[3]) << 4) | (data[4] >> 4));
            const uint16_t cur_raw = static_cast<uint16_t>(((data[4] & 0x0F) << 8) | data[5]);

            feedback.has_position = true;
            feedback.has_speed = true;
            feedback.has_current = true;
            feedback.has_temperature = true;
            feedback.position_rad = raw_to_pos(limits, pos_raw);
            feedback.speed_rad_s = raw_to_spd(limits, spd_raw);
            feedback.current_a = raw_to_current_default(static_cast<int>(cur_raw));
            feedback.temperature_c = temp_raw_to_deg(data[6]);
        }
        feedback.mode = 1;
    } else if (feedback.type == 2) {
        if (len >= 8) {
            feedback.has_position = true;
            feedback.has_current = true;
            feedback.has_temperature = true;
            feedback.position_rad = read_be_f32(&data[1]) * DEG_TO_RAD_LOCAL;
            feedback.current_a = static_cast<float>(read_be_i16(&data[5])) * 0.01f;
            feedback.temperature_c = temp_raw_to_deg(data[7]);
        }
        feedback.mode = 2;
    } else if (feedback.type == 3) {
        if (len >= 8) {
            feedback.has_speed = true;
            feedback.has_current = true;
            feedback.has_temperature = true;
            feedback.speed_rad_s = read_be_f32(&data[1]) * RPM_TO_RAD;
            feedback.current_a = static_cast<float>(read_be_i16(&data[5])) * 0.01f;
            feedback.temperature_c = temp_raw_to_deg(data[7]);
        }
        feedback.mode = 3;
    } else if (feedback.type == 4) {
        if (len >= 3) {
            feedback.has_state = true;
            feedback.state = data[2];
        }
    } else if (feedback.type == 5) {
        const uint8_t query_code = data[1];
        if (query_code == 1 && len >= 6) {
            feedback.has_position = true;
            feedback.position_rad = read_be_f32(&data[2]) * DEG_TO_RAD_LOCAL;
        } else if (query_code == 2 && len >= 6) {
            feedback.has_speed = true;
            feedback.speed_rad_s = read_be_f32(&data[2]) * RPM_TO_RAD;
        } else if (query_code == 3 && len >= 6) {
            feedback.has_current = true;
            feedback.current_a = read_be_f32(&data[2]);
        } else if (query_code == 37 && len >= 3) {
            feedback.has_state = true;
            feedback.state = data[2];
        }
    } else if (feedback.type == 6) {
        if (len >= 2) {
            feedback.has_state = true;
            feedback.state = data[1];
        }
    }

    return feedback;
}

}  // namespace encos

namespace {

constexpr float kHaitaiTwoPi = 6.28318530718f;
constexpr float kHaitaiPi = 3.14159265359f;
constexpr float kHaitaiCountPerTurn = 16384.0f;
constexpr float kHaitaiSpeedScale = 100.0f;    // 0.01 rpm
constexpr float kHaitaiCurrentScale = 1000.0f; // 0.001 A
constexpr float kHaitaiMitKpMax = 500.0f;
constexpr float kHaitaiMitKdMax = 5.0f;

int32_t haitai_clamp_i32(float value) {
    const float hi = static_cast<float>(std::numeric_limits<int32_t>::max());
    const float lo = static_cast<float>(std::numeric_limits<int32_t>::min());
    if (value > hi) value = hi;
    if (value < lo) value = lo;
    return static_cast<int32_t>(std::lround(value));
}

uint16_t haitai_clamp_u16(float value) {
    if (value < 0.0f) value = 0.0f;
    if (value > static_cast<float>(std::numeric_limits<uint16_t>::max())) {
        value = static_cast<float>(std::numeric_limits<uint16_t>::max());
    }
    return static_cast<uint16_t>(std::lround(value));
}

void haitai_write_le_i32(uint8_t* dst, int32_t value) {
    const uint32_t raw = static_cast<uint32_t>(value);
    dst[0] = static_cast<uint8_t>(raw & 0xFF);
    dst[1] = static_cast<uint8_t>((raw >> 8) & 0xFF);
    dst[2] = static_cast<uint8_t>((raw >> 16) & 0xFF);
    dst[3] = static_cast<uint8_t>((raw >> 24) & 0xFF);
}

void haitai_write_le_u16(uint8_t* dst, uint16_t value) {
    dst[0] = static_cast<uint8_t>(value & 0xFF);
    dst[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

int32_t haitai_read_le_i32(const uint8_t* src) {
    const uint32_t raw = static_cast<uint32_t>(src[0]) |
                         (static_cast<uint32_t>(src[1]) << 8) |
                         (static_cast<uint32_t>(src[2]) << 16) |
                         (static_cast<uint32_t>(src[3]) << 24);
    return static_cast<int32_t>(raw);
}

uint16_t haitai_read_le_u16(const uint8_t* src) {
    return static_cast<uint16_t>(src[0]) |
           (static_cast<uint16_t>(src[1]) << 8);
}

int16_t haitai_read_le_i16(const uint8_t* src) {
    return static_cast<int16_t>(haitai_read_le_u16(src));
}

int32_t haitai_rad_to_count(float rad) {
    return haitai_clamp_i32(rad * (kHaitaiCountPerTurn / kHaitaiTwoPi));
}

int32_t haitai_rad_s_to_rpm_x100(float rad_s) {
    return haitai_clamp_i32(rad_s * (60.0f / kHaitaiTwoPi) * kHaitaiSpeedScale);
}

int32_t haitai_amp_to_milliamp(float amp) {
    return haitai_clamp_i32(amp * kHaitaiCurrentScale);
}

float haitai_count_to_rad(int32_t count) {
    return static_cast<float>(count) * (kHaitaiTwoPi / kHaitaiCountPerTurn);
}

float haitai_wrap_to_signed_pi(float rad) {
    float wrapped = std::fmod(rad + kHaitaiPi, kHaitaiTwoPi);
    if (wrapped < 0.0f) wrapped += kHaitaiTwoPi;
    return wrapped - kHaitaiPi;
}

float haitai_rpm_x100_to_rad_s(int32_t raw) {
    const float rpm = static_cast<float>(raw) / kHaitaiSpeedScale;
    return rpm * (kHaitaiTwoPi / 60.0f);
}

float haitai_milliamp_to_amp(int32_t raw) {
    return static_cast<float>(raw) / kHaitaiCurrentScale;
}

uint32_t haitai_float_to_uint(float value, float min_value, float max_value, uint8_t bits) {
    if (max_value <= min_value) {
        return 0;
    }
    if (value < min_value) value = min_value;
    if (value > max_value) value = max_value;

    const uint32_t max_raw = (1U << bits) - 1U;
    const float span = max_value - min_value;
    const float scaled = (value - min_value) * static_cast<float>(max_raw) / span;
    return static_cast<uint32_t>(std::lround(scaled));
}

float haitai_uint_to_float(uint32_t raw, float min_value, float max_value, uint8_t bits) {
    const uint32_t max_raw = (1U << bits) - 1U;
    const float span = max_value - min_value;
    return static_cast<float>(raw) * span / static_cast<float>(max_raw) + min_value;
}

constexpr int kFeetechSCSPositionMax = 1023;
constexpr float kFeetechSCS0037RangeRad = 4.712389f; // 270 deg
constexpr float kFeetechRpmToRadS = 6.28318530718f / 60.0f;
constexpr int kFeetechReadTimeoutMs = 100;

constexpr uint8_t kFeetechInstRead = 0x02;
constexpr uint8_t kFeetechInstWrite = 0x03;

constexpr uint8_t kFeetechServoIdAddr = 5;
constexpr uint8_t kFeetechTorqueEnableAddr = 40;
constexpr uint8_t kFeetechGoalPositionAddr = 42;
constexpr uint8_t kFeetechEepromLockAddr = 48;
constexpr uint8_t kFeetechPresentPositionAddr = 56;
constexpr uint8_t kFeetechPresentSpeedAddr = 58;
constexpr uint8_t kFeetechPresentLoadAddr = 60;
constexpr uint8_t kFeetechPresentTemperatureAddr = 63;

bool feetech_valid_motor_index(const std::vector<Motor_CAN_Struct>* motors, int idx) {
    return motors && idx >= 0 && static_cast<std::size_t>(idx) < motors->size();
}

speed_t feetech_baud_to_termios(int baud) {
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

uint8_t feetech_checksum(const uint8_t* data, std::size_t length) {
    uint8_t sum = 0;
    for (std::size_t i = 0; i < length; ++i) {
        sum = static_cast<uint8_t>(sum + data[i]);
    }
    return static_cast<uint8_t>(~sum);
}

void feetech_write_be_u16(uint8_t* out, int value) {
    const uint16_t v = static_cast<uint16_t>(std::max(0, std::min(0xFFFF, value)));
    out[0] = static_cast<uint8_t>((v >> 8) & 0xFF);
    out[1] = static_cast<uint8_t>(v & 0xFF);
}

int feetech_read_be_u16(const uint8_t* data) {
    return (static_cast<int>(data[0]) << 8) | static_cast<int>(data[1]);
}

float jc_rad_to_deg_x100(float rad) {
    constexpr float kRadToDeg = 57.2957795f;
    return rad * kRadToDeg * 100.0f;
}

float jc_deg_x100_to_rad(int32_t raw) {
    constexpr float kDegToRad = 0.017453293f;
    return static_cast<float>(raw) * 0.01f * kDegToRad;
}

void jc_write_be_i32(uint8_t* dst, int32_t value) {
    const uint32_t raw = static_cast<uint32_t>(value);
    dst[0] = static_cast<uint8_t>((raw >> 24) & 0xFF);
    dst[1] = static_cast<uint8_t>((raw >> 16) & 0xFF);
    dst[2] = static_cast<uint8_t>((raw >> 8) & 0xFF);
    dst[3] = static_cast<uint8_t>(raw & 0xFF);
}

int32_t jc_read_be_i32(const uint8_t* src) {
    const uint32_t raw = (static_cast<uint32_t>(src[0]) << 24) |
                         (static_cast<uint32_t>(src[1]) << 16) |
                         (static_cast<uint32_t>(src[2]) << 8) |
                         static_cast<uint32_t>(src[3]);
    return static_cast<int32_t>(raw);
}

void make_jc_write16(uint8_t* data, uint16_t reg, uint16_t value) {
    data[0] = 0x2B;
    data[1] = static_cast<uint8_t>((reg >> 8) & 0xFF);
    data[2] = static_cast<uint8_t>(reg & 0xFF);
    data[3] = 0x00;
    data[4] = static_cast<uint8_t>((value >> 8) & 0xFF);
    data[5] = static_cast<uint8_t>(value & 0xFF);
    data[6] = 0x00;
    data[7] = 0x00;
}

void make_jc_write32(uint8_t* data, uint16_t reg, int32_t value) {
    data[0] = 0x23;
    data[1] = static_cast<uint8_t>((reg >> 8) & 0xFF);
    data[2] = static_cast<uint8_t>(reg & 0xFF);
    data[3] = 0x00;
    jc_write_be_i32(&data[4], value);
}

void make_jc_read32(uint8_t* data, uint16_t reg) {
    data[0] = 0x43;
    data[1] = static_cast<uint8_t>((reg >> 8) & 0xFF);
    data[2] = static_cast<uint8_t>(reg & 0xFF);
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
}

}  // namespace

HaitaiCommandFrame build_haitai_command(uint8_t mode, float position_rad,
                                        float speed_rad_s, float current_a,
                                        uint32_t can_id) {
    HaitaiCommandFrame frame {};
    frame.can_id = can_id & 0x7FFU;
    frame.dlc = 5;

    uint8_t cmd = 0xC2;
    int32_t value = haitai_rad_to_count(position_rad);

    switch (mode) {
        case 1:
            cmd = 0xC0;
            value = haitai_amp_to_milliamp(current_a);
            break;
        case 2:
            cmd = 0xC1;
            value = haitai_rad_s_to_rpm_x100(speed_rad_s);
            break;
        case 3:
            cmd = 0xC3;
            value = haitai_rad_to_count(position_rad);
            break;
        case 0:
        default:
            cmd = 0xC2;
            value = haitai_rad_to_count(position_rad);
            break;
    }

    frame.data[0] = cmd;
    haitai_write_le_i32(&frame.data[1], value);
    return frame;
}

HaitaiCommandFrame build_haitai_mit_command(float position_rad, float speed_rad_s,
                                            float kp, float kd, float torque_nm,
                                            float pos_max_rad,
                                            float vel_max_rad_s,
                                            float torque_max_nm,
                                            uint32_t can_id) {
    HaitaiCommandFrame frame {};
    const uint32_t node_id = can_id & 0x7FFU;
    frame.can_id = 0x400U | node_id;
    frame.dlc = 8;

    const uint32_t pos_raw = haitai_float_to_uint(position_rad, -pos_max_rad, pos_max_rad, 16);
    const uint32_t vel_raw = haitai_float_to_uint(speed_rad_s, -vel_max_rad_s, vel_max_rad_s, 12);
    const uint32_t kp_raw = haitai_float_to_uint(kp, 0.0f, kHaitaiMitKpMax, 12);
    const uint32_t kd_raw = haitai_float_to_uint(kd, 0.0f, kHaitaiMitKdMax, 12);
    const uint32_t torque_raw = haitai_float_to_uint(torque_nm, -torque_max_nm, torque_max_nm, 12);

    frame.data[0] = static_cast<uint8_t>((pos_raw >> 8) & 0xFF);
    frame.data[1] = static_cast<uint8_t>(pos_raw & 0xFF);
    frame.data[2] = static_cast<uint8_t>((vel_raw >> 4) & 0xFF);
    frame.data[3] = static_cast<uint8_t>(((vel_raw & 0x0F) << 4) | ((kp_raw >> 8) & 0x0F));
    frame.data[4] = static_cast<uint8_t>(kp_raw & 0xFF);
    frame.data[5] = static_cast<uint8_t>((kd_raw >> 4) & 0xFF);
    frame.data[6] = static_cast<uint8_t>(((kd_raw & 0x0F) << 4) | ((torque_raw >> 8) & 0x0F));
    frame.data[7] = static_cast<uint8_t>(torque_raw & 0xFF);

    return frame;
}

HaitaiCommandFrame build_haitai_mit_limits_config(float pos_max_rad,
                                                  float vel_max_rad_s,
                                                  float torque_max_nm,
                                                  uint32_t can_id) {
    HaitaiCommandFrame frame {};
    frame.can_id = can_id & 0x7FFU;
    frame.dlc = 7;
    frame.data[0] = 0xF0;
    haitai_write_le_u16(&frame.data[1], haitai_clamp_u16(pos_max_rad * 10.0f));
    haitai_write_le_u16(&frame.data[3], haitai_clamp_u16(vel_max_rad_s * 100.0f));
    haitai_write_le_u16(&frame.data[5], haitai_clamp_u16(torque_max_nm * 100.0f));
    return frame;
}

HaitaiCommandFrame make_haitai_simple_query(uint8_t cmd, uint32_t can_id) {
    HaitaiCommandFrame frame {};
    frame.can_id = can_id & 0x7FFU;
    frame.dlc = 1;
    frame.data[0] = cmd;
    return frame;
}

bool parse_haitai_feedback(uint32_t can_id, const std::array<uint8_t, 8>& data,
                           uint8_t dlc, HaitaiFeedback& out,
                           float mit_pos_max_rad,
                           float mit_vel_max_rad_s,
                           float mit_torque_max_nm) {
    (void)can_id;
    if (dlc == 0 || dlc > data.size()) {
        return false;
    }

    out = HaitaiFeedback {};
    out.command = data[0];

    switch (data[0]) {
        case 0xA0:
            if (dlc != 8) return false;
            out.has_version = true;
            out.boot_version = haitai_read_le_u16(&data[1]);
            out.app_version = haitai_read_le_u16(&data[3]);
            out.hw_version = haitai_read_le_u16(&data[5]);
            out.can_proto_version = data[7];
            return true;
        case 0xA1:
        case 0xC0:
            if (dlc != 5) return false;
            out.has_current = true;
            out.current_a = haitai_milliamp_to_amp(haitai_read_le_i32(&data[1]));
            return true;
        case 0xA2:
        case 0xC1:
            if (dlc != 5) return false;
            out.has_speed = true;
            out.speed_rad_s = haitai_rpm_x100_to_rad_s(haitai_read_le_i32(&data[1]));
            return true;
        case 0xA3:
        case 0xC2:
        case 0xC3:
        case 0xC4:
            if (dlc != 7) return false;
            out.has_position = true;
            out.position_rad = haitai_wrap_to_signed_pi(
                haitai_count_to_rad(haitai_read_le_u16(&data[1])));
            return true;
        case 0xA4:
            if (dlc != 8) return false;
            out.has_temperature = true;
            out.has_current = true;
            out.has_speed = true;
            out.has_position = true;
            out.temperature_c = static_cast<float>(data[1]);
            out.current_a = haitai_milliamp_to_amp(haitai_read_le_i16(&data[2]));
            out.speed_rad_s = haitai_rpm_x100_to_rad_s(haitai_read_le_i16(&data[4]));
            out.position_rad = haitai_wrap_to_signed_pi(
                haitai_count_to_rad(haitai_read_le_u16(&data[6])));
            return true;
        case 0xAE:
        case 0xCF:
            if (dlc != 8) return false;
            out.has_status = true;
            out.has_temperature = true;
            out.bus_voltage_v = static_cast<float>(haitai_read_le_u16(&data[1])) / 100.0f;
            out.bus_current_a = static_cast<float>(haitai_read_le_u16(&data[3])) / 100.0f;
            out.temperature_c = static_cast<float>(data[5]);
            out.mode = data[6];
            out.fault = data[7];
            return true;
        case 0xAF:
            if (dlc != 2) return false;
            out.has_status = true;
            out.fault = data[1];
            return true;
        case 0xB1:
            if (dlc != 3) return false;
            out.has_position = true;
            out.position_rad = haitai_wrap_to_signed_pi(
                haitai_count_to_rad(haitai_read_le_u16(&data[1])));
            return true;
        case 0xF0:
            if (dlc != 7) return false;
            out.has_mit_limits = true;
            out.mit_pos_max_rad = static_cast<float>(haitai_read_le_u16(&data[1])) * 0.1f;
            out.mit_vel_max_rad_s = static_cast<float>(haitai_read_le_u16(&data[3])) * 0.01f;
            out.mit_torque_max_nm = static_cast<float>(haitai_read_le_u16(&data[5])) * 0.01f;
            return true;
        case 0xF1: {
            if (dlc != 7) return false;
            const uint32_t pos_raw = (static_cast<uint32_t>(data[1]) << 8) |
                                     static_cast<uint32_t>(data[2]);
            const uint32_t vel_raw = (static_cast<uint32_t>(data[3]) << 4) |
                                     (static_cast<uint32_t>(data[4]) >> 4);
            const uint32_t torque_raw = ((static_cast<uint32_t>(data[4]) & 0x0F) << 8) |
                                         static_cast<uint32_t>(data[5]);
            out.has_mit_state = true;
            out.has_position = true;
            out.has_speed = true;
            out.has_torque = true;
            out.position_rad = haitai_uint_to_float(pos_raw, -mit_pos_max_rad, mit_pos_max_rad, 16);
            out.speed_rad_s = haitai_uint_to_float(vel_raw, -mit_vel_max_rad_s, mit_vel_max_rad_s, 12);
            out.torque_nm = haitai_uint_to_float(torque_raw, -mit_torque_max_nm, mit_torque_max_nm, 12);
            out.mit_status = data[6];
            out.mit_in_mode = (out.mit_status & 0x01U) != 0;
            out.mit_fault = (out.mit_status & 0x02U) != 0;
            out.mode = out.mit_in_mode ? 4 : 0;
            out.fault = out.mit_fault ? 1 : 0;
            return true;
        }
        default:
            return false;
    }
}

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
    const speed_t termios_baud = feetech_baud_to_termios(baud_);
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
    const float pos_max = (info.p_max > info.p_min) ? info.p_max : kFeetechSCS0037RangeRad;
    position_rad = std::max(pos_min, std::min(pos_max, position_rad));
    const float ratio = (position_rad - pos_min) / (pos_max - pos_min);
    return static_cast<int>(std::lround(ratio * static_cast<float>(kFeetechSCSPositionMax)));
}

float FeetechServoDevice::countToPosition(const Motor_CAN_Info_Struct& info, int count) const {
    const float pos_min = (info.p_max > info.p_min) ? info.p_min : 0.0f;
    const float pos_max = (info.p_max > info.p_min) ? info.p_max : kFeetechSCS0037RangeRad;
    count = std::max(0, std::min(kFeetechSCSPositionMax, count));
    const float ratio = static_cast<float>(count) / static_cast<float>(kFeetechSCSPositionMax);
    return pos_min + ratio * (pos_max - pos_min);
}

int FeetechServoDevice::speedToCount(float speed_rad_s) const {
    if (!std::isfinite(speed_rad_s) || speed_rad_s <= 0.0f) {
        return 0;
    }
    const float rpm = speed_rad_s / kFeetechRpmToRadS;
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
    packet[5 + length] = feetech_checksum(packet.data() + 2, length + 3);

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
        timeout.tv_usec = kFeetechReadTimeoutMs * 1000;

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

    if (feetech_checksum(packet.data() + 2, length + 3) != packet[expected_len - 1]) {
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

    if (!writePacket(static_cast<uint8_t>(id), kFeetechInstWrite, params.data(), length + 1)) {
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
    if (!writePacket(static_cast<uint8_t>(id), kFeetechInstRead, params, sizeof(params))) {
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

    int value = feetech_read_be_u16(data);
    if (signed_value && (value & (1 << sign_bit))) {
        value = -(value & ~(1 << sign_bit));
    }
    return value;
}

bool FeetechServoDevice::writePosition(int id, int position, int speed) {
    uint8_t data[6] = {0, 0, 0, 0, 0, 0};
    feetech_write_be_u16(data + 0, position);
    feetech_write_be_u16(data + 2, 0);
    feetech_write_be_u16(data + 4, speed);
    return writeRegister(id, kFeetechGoalPositionAddr, data, sizeof(data));
}

bool FeetechServoDevice::enableTorque(int id, bool enable) {
    return writeRegisterByte(id, kFeetechTorqueEnableAddr, enable ? 1 : 0);
}

bool FeetechServoDevice::SetServoId(int old_id, int new_id) {
    if (!is_open_ || old_id < 0 || old_id > 253 || new_id < 0 || new_id > 253 ||
        old_id == 254 || new_id == 254) {
        return false;
    }

    if (!writeRegisterByte(old_id, kFeetechEepromLockAddr, 0)) {
        std::cerr << "[Error] Failed to unlock Feetech EEPROM: id=" << old_id << std::endl;
        return false;
    }

    if (!writeRegisterByte(old_id, kFeetechServoIdAddr, static_cast<uint8_t>(new_id))) {
        std::cerr << "[Error] Failed to write Feetech ID: old_id=" << old_id
                  << ", new_id=" << new_id << std::endl;
        return false;
    }

    if (!writeRegisterByte(new_id, kFeetechEepromLockAddr, 1)) {
        std::cerr << "[Warn] Feetech ID was written, but EEPROM re-lock did not confirm: new_id="
                  << new_id << std::endl;
    }
    return true;
}

void FeetechServoDevice::EnableMotor(int& motor_index) {
    if (!is_open_ || !feetech_valid_motor_index(p_motors_data_, motor_index)) return;
    const auto& info = (*p_motors_data_)[motor_index].info;
    if (enableTorque(info.canid, true)) {
        (*p_motors_data_)[motor_index].recv.motor_state = 1;
    } else {
        (*p_motors_data_)[motor_index].recv.fault_message = 1;
    }
}

void FeetechServoDevice::DisableMotor(int& motor_index) {
    if (!is_open_ || !feetech_valid_motor_index(p_motors_data_, motor_index)) return;
    const auto& info = (*p_motors_data_)[motor_index].info;
    if (enableTorque(info.canid, false)) {
        (*p_motors_data_)[motor_index].recv.motor_state = 0;
    } else {
        (*p_motors_data_)[motor_index].recv.fault_message = 1;
    }
}

void FeetechServoDevice::ClearError(int& motor_index) {
    if (!feetech_valid_motor_index(p_motors_data_, motor_index)) return;
    (*p_motors_data_)[motor_index].recv.fault_message = 0;
}

void FeetechServoDevice::SetZero(int& motor_index) {
    if (!feetech_valid_motor_index(p_motors_data_, motor_index)) return;
    std::cout << "[Info] Feetech SetZero is software-neutral; keep TOML pos_min/pos_max as travel limits."
              << std::endl;
}

void FeetechServoDevice::SetMode(int& motor_index, int mode) {
    if (!feetech_valid_motor_index(p_motors_data_, motor_index)) return;
    (*p_motors_data_)[motor_index].send.mode = static_cast<uint8_t>(std::max(0, mode));
}

void FeetechServoDevice::SendCommand(int& motor_index) {
    if (!is_open_ || !feetech_valid_motor_index(p_motors_data_, motor_index)) return;
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
    if (!is_open_ || !feetech_valid_motor_index(p_motors_data_, motor_index)) return;
    auto& motor = (*p_motors_data_)[motor_index];
    const int id = motor.info.canid;

    const int pos = readRegisterWord(id, kFeetechPresentPositionAddr, false, 0);
    if (pos >= 0) {
        motor.recv.current_position_f.store(countToPosition(motor.info, pos));
        motor.recv.motor_id = static_cast<uint8_t>(id);
    }

    const int speed = readRegisterWord(id, kFeetechPresentSpeedAddr, true, 15);
    if (speed != -1) {
        motor.recv.current_speed_f.store(static_cast<float>(speed) * kFeetechRpmToRadS);
    }

    const int load = readRegisterWord(id, kFeetechPresentLoadAddr, true, 10);
    if (load != -1) {
        motor.recv.current_torque_f.store(static_cast<float>(load) / 1000.0f);
    }

    const int temp = readRegisterByte(id, kFeetechPresentTemperatureAddr);
    if (temp != -1) {
        motor.recv.current_temp_f.store(static_cast<float>(temp));
    }
}

void FeetechServoDevice::QueryVersion(int& motor_index) {
    (void)motor_index;
}

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

static const float TWO_PI = 6.28318530718f;
static const float HAITAI_COUNT_TO_RAD = TWO_PI / 16384.0f;
static const float HAITAI_MIT_DEFAULT_POS_MAX_RAD = 95.5f;
static const float HAITAI_MIT_DEFAULT_VEL_MAX_RAD_S = 45.0f;
static const float HAITAI_MIT_DEFAULT_TORQUE_MAX_NM = 18.0f;
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

static bool env_bool(const char* key, bool default_val) {
    const char* env = std::getenv(key);
    if (!env) return default_val;
    const std::string v(env);
    return (v == "1" || v == "true" || v == "TRUE" || v == "on" || v == "ON");
}

static bool pfl28_use_xyber_mode() {
    // AgiBot X1 drives L28 through DCU CAN-FD broadcast slots at 1 kHz.
    return env_bool("PFL28_XYBER_MODE", true);
}

static bool pfl28_use_canfd_brs() {
    const char* env = std::getenv("PFL28_CANFD_BRS");
    if (!env) return pfl28_use_xyber_mode();
    return env_bool("PFL28_CANFD_BRS", false);
}

static bool pfl28_use_canfd_frame() {
    // Official manual specifies CAN FD transport for PowerFlow L28.
    // Keep CAN FD as default; allow temporary fallback via env for field debugging.
    return env_bool("PFL28_USE_CANFD", true);
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
        cmd += " sample-point 0.8 dbitrate " + std::to_string(dbitrate) + " dsample-point 0.75 fd on";
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

unsigned long long DeviceX::EnobufsDropCount() const {
    return enobufs_drop_count.load(std::memory_order_relaxed);
}

bool DeviceX::SocketReady() const {
    return socket_fd >= 0;
}

const std::string& DeviceX::InterfaceName() const {
    return iface_name;
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

bool DeviceX::sendExtendedFrame(uint32_t type, uint16_t data_area, uint8_t motor_id, const uint8_t* data) {
    if (socket_fd < 0 || data == nullptr) return false;

    struct can_frame frame {};
    frame.can_id = ((type & 0x1F) << 24) | ((data_area & 0xFFFF) << 8) | (motor_id & 0xFF);
    frame.can_id |= CAN_EFF_FLAG;
    frame.can_dlc = 8;
    std::memcpy(frame.data, data, 8);
    return sendFrameWithRetry(&frame, sizeof(frame), "sendExtendedFrame");
}

bool DeviceX::writeType1Param(uint8_t motor_id, uint16_t index, float value) {
    uint8_t data[8] = {0};
    std::memcpy(&data[0], &index, sizeof(index));
    std::memcpy(&data[4], &value, sizeof(value));
    return sendExtendedFrame(18, 0xFD, motor_id, data);
}

bool DeviceX::sendExtendedIdFrame(uint32_t can_id, const uint8_t* data, uint8_t dlc) {
    if (socket_fd < 0 || data == nullptr || dlc > 8) return false;

    struct can_frame frame {};
    frame.can_id = (can_id & CAN_EFF_MASK) | CAN_EFF_FLAG;
    frame.can_dlc = dlc;
    std::memcpy(frame.data, data, dlc);
    return sendFrameWithRetry(&frame, sizeof(frame), "sendExtendedIdFrame");
}

bool DeviceX::sendExtendedIdFdFrame(uint32_t can_id, const uint8_t* data, uint8_t len) {
    if (socket_fd < 0 || data == nullptr || len > 64) return false;

    struct canfd_frame frame {};
    frame.can_id = (can_id & CAN_EFF_MASK) | CAN_EFF_FLAG;
    frame.len = len;
    frame.flags = use_canfd_brs() ? CANFD_BRS : 0;
    std::memcpy(frame.data, data, len);
    return sendFrameWithRetry(&frame, sizeof(frame), "sendExtendedIdFdFrame");
}

bool DeviceX::sendStandardFrame(uint32_t can_id, const uint8_t* data, uint8_t dlc) {
    if (socket_fd < 0 || data == nullptr || dlc > 8) return false;

    struct can_frame frame {};
    frame.can_id = (can_id & CAN_SFF_MASK);
    frame.can_dlc = dlc;
    std::memcpy(frame.data, data, dlc);
    return sendFrameWithRetry(&frame, sizeof(frame), "sendStandardFrame");
}

bool DeviceX::sendStandardFdFrame(uint32_t can_id, const uint8_t* data, uint8_t len, bool brs) {
    if (socket_fd < 0 || data == nullptr || len > CANFD_MAX_DLEN) return false;

    struct canfd_frame frame {};
    frame.can_id = (can_id & CAN_SFF_MASK);
    frame.len = len;
    frame.flags = brs ? CANFD_BRS : 0;
    std::memcpy(frame.data, data, len);
    return sendFrameWithRetry(&frame, sizeof(frame), "sendStandardFdFrame");
}

bool DeviceX::sendRawFrame(uint8_t chan, uint32_t type, uint16_t data_area, uint8_t motor_id, uint8_t* data) {
    (void)chan;
    return sendExtendedFrame(type, data_area, motor_id, data);
}

// =========================================================
//  DeviceX public dispatch
// =========================================================

bool DeviceX::EnableMotor(int& motor_index) {
    if (!p_motors_data || motor_index < 0 || motor_index >= (int)p_motors_data->size()) return false;

    int type = (*p_motors_data)[motor_index].info.api_type;

    if (type == 1) return EnableMotor_Type1(motor_index);
    else if (type == 2) return EnableMotor_Type2(motor_index);
    else if (type == 3) return EnableMotor_Type3(motor_index);
    else if (type == 5) return EnableMotor_Type5(motor_index);
    else if (type == 6) return EnableMotor_Type6(motor_index);
    else if (type == 7) return EnableMotor_Type7(motor_index);
    else if (type == 8) return EnableMotor_Type8(motor_index);
    else if (type == 9) return EnableMotor_Type9(motor_index);
    return false;
}

bool DeviceX::DisableMotor(int& motor_index) {
    if (!p_motors_data || motor_index < 0 || motor_index >= (int)p_motors_data->size()) return false;

    int type = (*p_motors_data)[motor_index].info.api_type;

    if (type == 1) return DisableMotor_Type1(motor_index);
    else if (type == 2) return DisableMotor_Type2(motor_index);
    else if (type == 3) return DisableMotor_Type3(motor_index);
    else if (type == 5) return DisableMotor_Type5(motor_index);
    else if (type == 6) return DisableMotor_Type6(motor_index);
    else if (type == 7) return DisableMotor_Type7(motor_index);
    else if (type == 8) return DisableMotor_Type8(motor_index);
    else if (type == 9) return DisableMotor_Type9(motor_index);
    return false;
}

void DeviceX::ClearError(int& motor_index) {
    if (!p_motors_data || motor_index < 0 || motor_index >= (int)p_motors_data->size()) return;

    int type = (*p_motors_data)[motor_index].info.api_type;

    if (type == 1) ClearError_Type1(motor_index);
    else if (type == 2) ClearError_Type2(motor_index);
    else if (type == 3) ClearError_Type3(motor_index);
    else if (type == 5) ClearError_Type5(motor_index);
    else if (type == 6) ClearError_Type6(motor_index);
    else if (type == 7) ClearError_Type7(motor_index);
    else if (type == 8) ClearError_Type8(motor_index);
    else if (type == 9) ClearError_Type9(motor_index);
}

void DeviceX::SetZero(int& motor_index) {
    if (!p_motors_data || motor_index < 0 || motor_index >= (int)p_motors_data->size()) return;

    int type = (*p_motors_data)[motor_index].info.api_type;

    if (type == 1) SetZero_Type1(motor_index);
    else if (type == 2) SetZero_Type2(motor_index);
    else if (type == 3) SetZero_Type3(motor_index);
    else if (type == 5) SetZero_Type5(motor_index);
    else if (type == 6) SetZero_Type6(motor_index);
    else if (type == 7) SetZero_Type7(motor_index);
    else if (type == 8) SetZero_Type8(motor_index);
    else if (type == 9) SetZero_Type9(motor_index);
}

bool DeviceX::SetMode(int& motor_index, int mode) {
    if (!p_motors_data || motor_index < 0 || motor_index >= (int)p_motors_data->size()) return false;

    int type = (*p_motors_data)[motor_index].info.api_type;

    if (type == 1) return SetMode_Type1(motor_index, mode);
    else if (type == 2) return SetMode_Type2(motor_index, mode);
    else if (type == 3) return SetMode_Type3(motor_index, mode);
    else if (type == 5) return SetMode_Type5(motor_index, mode);
    else if (type == 6) return SetMode_Type6(motor_index, mode);
    else if (type == 7) return SetMode_Type7(motor_index, mode);
    else if (type == 8) return SetMode_Type8(motor_index, mode);
    else if (type == 9) return SetMode_Type9(motor_index, mode);
    return false;
}

bool DeviceX::SendCommand(int& motor_index) {
    if (!p_motors_data || motor_index < 0 || motor_index >= (int)p_motors_data->size()) {
        std::cerr << "[ERROR] SendCommand Failed! Index Out of Range or Null Ptr." << std::endl;
        return false;
    }

    int type = (*p_motors_data)[motor_index].info.api_type;

    if (type == 1) return SendCommand_Type1(motor_index);
    else if (type == 2) return SendCommand_Type2(motor_index);
    else if (type == 3) return SendCommand_Type3(motor_index);
    else if (type == 5) return SendCommand_Type5(motor_index);
    else if (type == 6) return SendCommand_Type6(motor_index);
    else if (type == 7) return SendCommand_Type7(motor_index);
    else if (type == 8) return SendCommand_Type8(motor_index);
    else if (type == 9) return SendCommand_Type9(motor_index);
    return false;
}

bool DeviceX::QueryPos(int& motor_index) {
    if (!p_motors_data || motor_index < 0 || motor_index >= (int)p_motors_data->size()) return false;

    int type = (*p_motors_data)[motor_index].info.api_type;

    if (type == 1) return QueryPos_Type1(motor_index);
    else if (type == 2) return QueryPos_Type2(motor_index);
    else if (type == 3) return QueryPos_Type3(motor_index);
    else if (type == 5) return QueryPos_Type5(motor_index);
    else if (type == 6) return QueryPos_Type6(motor_index);
    else if (type == 7) return QueryPos_Type7(motor_index);
    else if (type == 8) return QueryPos_Type8(motor_index);
    else if (type == 9) return QueryPos_Type9(motor_index);
    return false;
}

void DeviceX::QueryVersion(int& motor_index) {
    if (!p_motors_data || motor_index < 0 || motor_index >= (int)p_motors_data->size()) return;

    int type = (*p_motors_data)[motor_index].info.api_type;
    if (type == 7) QueryVersion_Type7(motor_index);
}

// =========================================================
//  Type 1 (LimX)
// =========================================================

bool DeviceX::EnableMotor_Type1(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    uint8_t data[8] = {0};
    return sendExtendedFrame(3, 0xFD, info.canid, data);
}

bool DeviceX::DisableMotor_Type1(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    uint8_t data[8] = {0};
    return sendExtendedFrame(4, 0xFD, info.canid, data);
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

bool DeviceX::SetMode_Type1(int& motor_index, int mode) {
    auto& motor = (*p_motors_data)[motor_index];
    const auto& info = motor.info;
    if (mode < 0) mode = 0;
    if (mode > 3) mode = 3;
    motor.send.mode = static_cast<uint8_t>(mode);

    uint8_t data[8] = {0};
    uint16_t index = 0x7005;
    uint8_t mode_val = motor.send.mode;

    std::memcpy(&data[0], &index, 2);
    std::memcpy(&data[4], &mode_val, 1);

    return sendExtendedFrame(18, 0xFD, info.canid, data);
}

bool DeviceX::SendCommand_Type1(int& motor_index) {
    const Motor_CAN_Struct& motor = (*p_motors_data)[motor_index];
    const auto& cmd = motor.send;
    const auto& info = motor.info;

    uint8_t data[8] = {0};

    if (cmd.mode == 2) {
        float speed = cmd.speed;
        if (!std::isfinite(speed)) speed = 0.0f;
        if (info.v_max > info.v_min) {
            speed = std::max(info.v_min, std::min(info.v_max, speed));
        }

        float limit_cur = std::fabs(cmd.torque);
        if (!std::isfinite(limit_cur) || limit_cur <= 1e-6f) {
            limit_cur = (info.t_max > 0.0f) ? info.t_max : 43.0f;
        }
        limit_cur = std::max(0.0f, std::min(43.0f, limit_cur));

        const bool limit_ok = writeType1Param(static_cast<uint8_t>(info.canid), 0x7018, limit_cur);
        const bool speed_ok = writeType1Param(static_cast<uint8_t>(info.canid), 0x700A, speed);
        return limit_ok && speed_ok;
    }

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

    return sendExtendedFrame(1, t_int, info.canid, data);
}

bool DeviceX::QueryPos_Type1(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;

    uint8_t data[8] = {0};
    uint16_t index = 0x7019;
    std::memcpy(&data[0], &index, 2);

    return sendExtendedFrame(17, 0xFD, info.canid, data);
}

// =========================================================
//  Type 2 (LK)
// =========================================================

bool DeviceX::EnableMotor_Type2(int& motor_index) {
    const Motor_CAN_Struct& motor = (*p_motors_data)[motor_index];

    uint8_t data[8] = {0};
    data[0] = 0x88;
    return sendStandardFrame(0x140 + motor.info.canid, data);
}

bool DeviceX::DisableMotor_Type2(int& motor_index) {
    const Motor_CAN_Struct& motor = (*p_motors_data)[motor_index];

    uint8_t data[8] = {0};
    data[0] = 0x80;
    return sendStandardFrame(0x140 + motor.info.canid, data);
}

void DeviceX::ClearError_Type2(int& motor_index) {
    DisableMotor_Type2(motor_index);
}

void DeviceX::SetZero_Type2(int& motor_index) {
    (void)motor_index;
}

bool DeviceX::SetMode_Type2(int& motor_index, int mode) {
    (*p_motors_data)[motor_index].send.mode = static_cast<uint8_t>(mode);
    return true;
}

bool DeviceX::SendCommand_Type2(int& motor_index) {
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

    return sendStandardFrame(0x140 + id, data);
}

bool DeviceX::QueryPos_Type2(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    uint8_t data[8] = {0};
    data[0] = 0x9C;
    return sendStandardFrame(0x140 + info.canid, data);
}

// =========================================================
//  Type 3 (DM)
// =========================================================

bool DeviceX::EnableMotor_Type3(int& motor_index) {
    const auto& motor = (*p_motors_data)[motor_index];

    uint16_t canid = static_cast<uint16_t>(motor.info.canid);
    uint8_t mode = motor.send.mode;
    uint32_t can_id = (mode == 1) ? (0x100 + canid) : ((mode == 2) ? (0x200 + canid) : canid);

    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    return sendStandardFrame(can_id, data);
}

bool DeviceX::DisableMotor_Type3(int& motor_index) {
    const auto& motor = (*p_motors_data)[motor_index];

    uint16_t canid = static_cast<uint16_t>(motor.info.canid);
    uint8_t mode = motor.send.mode;
    uint32_t can_id = (mode == 1) ? (0x100 + canid) : ((mode == 2) ? (0x200 + canid) : canid);

    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    return sendStandardFrame(can_id, data);
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

bool DeviceX::SetMode_Type3(int& motor_index, int mode) {
    if (mode < 0) mode = 0;
    if (mode > 3) mode = 3;
    (*p_motors_data)[motor_index].send.mode = static_cast<uint8_t>(mode);
    return true;
}

bool DeviceX::SendCommand_Type3(int& motor_index) {
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
        return sendStandardFrame(can_id, data);
    } else if (cmd.mode == 1) {
        float p = cmd.position;
        float v = cmd.speed;
        uint32_t can_id = 0x100 + (uint16_t)info.canid;
        std::memcpy(&data[0], &p, 4);
        std::memcpy(&data[4], &v, 4);
        return sendStandardFrame(can_id, data);
    } else if (cmd.mode == 2) {
        float v = cmd.speed;
        uint32_t can_id = 0x200 + (uint16_t)info.canid;
        std::memcpy(&data[0], &v, 4);
        return sendStandardFrame(can_id, data, 4);
    }
    return false;
}

bool DeviceX::QueryPos_Type3(int& motor_index) {
    const auto& motor = (*p_motors_data)[motor_index];

    uint8_t data[8] = {0};
    const uint16_t canid = static_cast<uint16_t>(motor.info.canid);
    data[0] = static_cast<uint8_t>(canid & 0xFF);
    data[1] = static_cast<uint8_t>((canid >> 8) & 0xFF);
    data[2] = 0xCC;

    return sendStandardFrame(0x7FF, data);
}

// =========================================================
//  Type 5 (HighTorque/高擎)
// =========================================================

bool DeviceX::EnableMotor_Type5(int& motor_index) {
    // HighTorque protocol does not expose a dedicated "enable" command in the
    // provided CAN document. We perform a state query to confirm communication.
    return QueryPos_Type5(motor_index);
}

bool DeviceX::DisableMotor_Type5(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    const uint32_t can_id = build_hq_can_id(static_cast<uint8_t>(info.canid), HQ_REPLY_REQUIRED);
    // ref/livelybot_fdcan.c: set_motor_stop_int16()
    const uint8_t tdata[] = {0x01, 0x00, 0x00, 0x14, 0x04, 0x00, 0x11, 0x0F};
    return sendExtendedIdFrame(can_id, tdata, sizeof(tdata));
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

bool DeviceX::SetMode_Type5(int& motor_index, int mode) {
    if (mode < 0) mode = 0;
    if (mode > 3) mode = 3;
    (*p_motors_data)[motor_index].send.mode = static_cast<uint8_t>(mode);
    return true;
}

bool DeviceX::SendCommand_Type5(int& motor_index) {
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
            return sendExtendedIdFrame(can_id, tdata, 8);
        }
        case 2: { // velocity + torque, position unlimited
            tdata[0] = 0x07;
            tdata[1] = 0x07;
            write_le_u16(&tdata[2], unlimited);
            write_le_i16(&tdata[4], vel_raw);
            write_le_i16(&tdata[6], tq_ff_raw);
            return sendExtendedIdFrame(can_id, tdata, 8);
        }
        case 3: { // torque only
            const int16_t tq_raw = encode_torque_raw(cmd.torque);
            tdata[0] = 0x05;
            tdata[1] = 0x13;
            write_le_i16(&tdata[2], tq_raw);
            return sendExtendedIdFrame(can_id, tdata, 4);
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
            return sendExtendedIdFdFrame(can_id, fd_data, sizeof(fd_data));
        }
    }
    return false;
}

bool DeviceX::QueryPos_Type5(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    const uint32_t can_id = build_hq_can_id(static_cast<uint8_t>(info.canid), HQ_REPLY_REQUIRED);
    // ref/livelybot_fdcan.c: read_motor_state_int16()
    const uint8_t tdata[] = {0x14, 0x04, 0x00, 0x11, 0x0F};
    return sendExtendedIdFrame(can_id, tdata, sizeof(tdata));
}

// =========================================================
//  Type 6 (AgiBot PowerFlow L28/PFL28)
// =========================================================

bool DeviceX::EnableMotor_Type6(int& motor_index) {
    // PFL28 powers up in enabled state.
    (void)motor_index;
    return true;
}

bool DeviceX::DisableMotor_Type6(int& motor_index) {
    // No dedicated disable command in PFL28 public protocol.
    // Send a zero-current hold at the latest known position as a safe fallback.
    if (!p_motors_data) return false;
    auto& motor = (*p_motors_data)[motor_index];
    motor.send.position = motor.recv.current_position_f.load();
    motor.send.torque = 0.0f;
    return SendCommand_Type6(motor_index);
}

void DeviceX::ClearError_Type6(int& motor_index) {
    // No clear-error frame is documented for PFL28.
    (void)motor_index;
}

void DeviceX::SetZero_Type6(int& motor_index) {
    // No set-zero frame is documented for PFL28.
    (void)motor_index;
}

bool DeviceX::SetMode_Type6(int& motor_index, int mode) {
    // PFL28 uses position/current command frame; keep mode for compatibility.
    (*p_motors_data)[motor_index].send.mode = static_cast<uint8_t>(mode);
    return true;
}

bool DeviceX::SendCommand_Type6(int& motor_index) {
    const auto& motor = (*p_motors_data)[motor_index];
    const auto& info = motor.info;

    if (pfl28_use_canfd_frame() && pfl28_use_xyber_mode() && info.canid >= 1 && info.canid <= 8) {
        return SendCommand_Type6_XyberBroadcast(motor_index);
    }

    const float pos_min = (info.p_max > info.p_min) ? info.p_min : 0.0f;
    const float pos_max = (info.p_max > info.p_min) ? info.p_max : 9.5f;
    const float cur_max = (info.t_max > info.t_min) ? info.t_max : 2.5f;
    float cur_min = (info.t_max > info.t_min) ? info.t_min : 0.0f;
    if (pfl28_allow_neg_current() && cur_min >= 0.0f) {
        cur_min = -cur_max;
    }

    float pos_cmd = motor.send.position;
    float cur_cmd = motor.send.torque;

    if (!std::isfinite(pos_cmd)) pos_cmd = 0.0f;
    if (!std::isfinite(cur_cmd)) cur_cmd = 0.0f;

    pos_cmd = std::max(pos_min, std::min(pos_max, pos_cmd));
    cur_cmd = std::max(cur_min, std::min(cur_max, cur_cmd));

    if (!pfl28_use_canfd_frame()) {
        std::cerr << "[Error] PFL28 point-to-point protocol requires CAN FD." << std::endl;
        return false;
    }

    uint8_t data[8] = {0};
    std::memcpy(&data[0], &pos_cmd, sizeof(float));
    std::memcpy(&data[4], &cur_cmd, sizeof(float));
    return sendStandardFdFrame(static_cast<uint32_t>(info.canid), data, sizeof(data), pfl28_use_canfd_brs());
}

namespace {

void appendPfl28XyberSlot(
    const Motor_CAN_Struct& motor,
    uint8_t* data,
    bool allow_neg_i) {
    const auto& info = motor.info;
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

    const std::size_t offset = static_cast<std::size_t>(info.canid - 1) * 8U;
    std::memcpy(&data[offset], &pos_cmd, sizeof(float));
    std::memcpy(&data[offset + 4], &cur_cmd, sizeof(float));
}

}  // namespace

bool DeviceX::SendCommand_Type6_XyberBroadcast(int& motor_index) {
    if (!p_motors_data || motor_index < 0 ||
        motor_index >= static_cast<int>(p_motors_data->size())) {
        return false;
    }

    const auto& motor = (*p_motors_data)[motor_index];
    const auto& info = motor.info;
    uint8_t data[64] = {0};
    const bool allow_neg_i = pfl28_allow_neg_current();
    bool has_slot = false;

    for (const auto& other : *p_motors_data) {
        if (other.info.api_type == 6 &&
            other.info.device_index == info.device_index &&
            other.info.canid >= 1 &&
            other.info.canid <= 8) {
            appendPfl28XyberSlot(other, data, allow_neg_i);
            has_slot = true;
        }
    }

    if (!has_slot) {
        return false;
    }
    return sendStandardFdFrame(0, data, sizeof(data), pfl28_use_canfd_brs());
}

bool DeviceX::QueryPos_Type6(int& motor_index) {
    // PFL28 returns state after each control frame; no standalone query frame documented.
    // We keep this as no-op to avoid accidental motion during status polling loops.
    (void)motor_index;
    return true;
}

// =========================================================
//  Type 7 (Haitai/海泰)
// =========================================================

bool DeviceX::EnableMotor_Type7(int& motor_index) {
    // Haitai MIT/status policy is owned by the caller/node. Keep enable passive
    // so construction or mode switches never write 0xF0 implicitly.
    (void)motor_index;
    return true;
}

bool DeviceX::DisableMotor_Type7(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    const HaitaiCommandFrame frame = make_haitai_simple_query(0xCF, static_cast<uint32_t>(info.canid));
    return sendStandardFrame(frame.can_id, frame.data.data(), frame.dlc);
}

void DeviceX::ClearError_Type7(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    const HaitaiCommandFrame frame = make_haitai_simple_query(0xAF, static_cast<uint32_t>(info.canid));
    sendStandardFrame(frame.can_id, frame.data.data(), frame.dlc);
}

void DeviceX::SetZero_Type7(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    const HaitaiCommandFrame frame = make_haitai_simple_query(0xB1, static_cast<uint32_t>(info.canid));
    sendStandardFrame(frame.can_id, frame.data.data(), frame.dlc);
}

bool DeviceX::SetMode_Type7(int& motor_index, int mode) {
    if (mode < 0) mode = 0;
    if (mode > 4) mode = 4;
    auto& motor = (*p_motors_data)[motor_index];
    motor.send.mode = static_cast<uint8_t>(mode);
    return true;
}

bool DeviceX::ConfigureHaitaiMitLimits(int& motor_index) {
    if (!p_motors_data || motor_index < 0 ||
        static_cast<std::size_t>(motor_index) >= p_motors_data->size()) {
        return false;
    }

    auto& motor = (*p_motors_data)[motor_index];
    const auto& info = motor.info;
    if (info.api_type != 7) {
        return false;
    }

    const float pos_max = (info.p_max > 0.0f) ? info.p_max : HAITAI_MIT_DEFAULT_POS_MAX_RAD;
    const float vel_max = (info.v_max > 0.0f) ? info.v_max : HAITAI_MIT_DEFAULT_VEL_MAX_RAD_S;
    const float torque_max = (info.t_max > 0.0f) ? info.t_max : HAITAI_MIT_DEFAULT_TORQUE_MAX_NM;

    const HaitaiCommandFrame limits_frame = build_haitai_mit_limits_config(
        pos_max,
        vel_max,
        torque_max,
        static_cast<uint32_t>(info.canid));
    return sendStandardFrame(limits_frame.can_id, limits_frame.data.data(), limits_frame.dlc);
}

bool DeviceX::SendCommand_Type7(int& motor_index) {
    const auto& motor = (*p_motors_data)[motor_index];
    const auto& info = motor.info;
    const auto& cmd = motor.send;

    HaitaiCommandFrame frame {};
    if (cmd.mode == 4) {
        const auto& recv = motor.recv;
        const float pos_max = recv.haitai_mit_limits_valid ? recv.haitai_mit_pos_max_rad :
            ((info.p_max > 0.0f) ? info.p_max : HAITAI_MIT_DEFAULT_POS_MAX_RAD);
        const float vel_max = recv.haitai_mit_limits_valid ? recv.haitai_mit_vel_max_rad_s :
            ((info.v_max > 0.0f) ? info.v_max : HAITAI_MIT_DEFAULT_VEL_MAX_RAD_S);
        const float torque_max = recv.haitai_mit_limits_valid ? recv.haitai_mit_torque_max_nm :
            ((info.t_max > 0.0f) ? info.t_max : HAITAI_MIT_DEFAULT_TORQUE_MAX_NM);
        frame = build_haitai_mit_command(
            cmd.position,
            cmd.speed,
            cmd.kp,
            cmd.kd,
            cmd.torque,
            pos_max,
            vel_max,
            torque_max,
            static_cast<uint32_t>(info.canid));
    } else {
        frame = build_haitai_command(
            cmd.mode,
            cmd.position,
            cmd.speed,
            cmd.torque,
            static_cast<uint32_t>(info.canid));
    }
    return sendStandardFrame(frame.can_id, frame.data.data(), frame.dlc);
}

bool DeviceX::QueryPos_Type7(int& motor_index) {
    auto& motor = (*p_motors_data)[motor_index];
    const auto& info = motor.info;
    const uint32_t can_id = static_cast<uint32_t>(info.canid);

    auto send_query = [&](uint8_t cmd) {
        const HaitaiCommandFrame frame = make_haitai_simple_query(cmd, can_id);
        return sendStandardFrame(frame.can_id, frame.data.data(), frame.dlc);
    };

    if (!motor.recv.version_valid) {
        bool ok = send_query(0xA0);
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        ok = send_query(0xA4) && ok;
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        ok = send_query(0xA3) && ok;
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        ok = send_query(0xA2) && ok;
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        ok = send_query(0xA1) && ok;
        return ok;
    }

    if (motor.recv.can_proto_version < 7) {
        bool ok = send_query(0xA3);
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        ok = send_query(0xA2) && ok;
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        ok = send_query(0xA1) && ok;
        return ok;
    }

    return send_query(0xA4);
}

void DeviceX::QueryVersion_Type7(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    const HaitaiCommandFrame frame = make_haitai_simple_query(0xA0, static_cast<uint32_t>(info.canid));
    sendStandardFrame(frame.can_id, frame.data.data(), frame.dlc);
}

// =========================================================
//  Type 8 (ENCOS EC-A series)
// =========================================================

bool DeviceX::EnableMotor_Type8(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    const encos::CommandFrame release =
        encos::make_brake_release_command(static_cast<uint32_t>(info.canid));
    const bool release_ok = sendStandardFrame(release.can_id, release.data.data(), release.dlc);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    const encos::CommandFrame current_zero =
        encos::make_current_zero_command(static_cast<uint32_t>(info.canid));
    const bool zero_ok = sendStandardFrame(current_zero.can_id, current_zero.data.data(), current_zero.dlc);
    return release_ok && zero_ok;
}

bool DeviceX::DisableMotor_Type8(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    const encos::CommandFrame frame =
        encos::make_damping_command(static_cast<uint32_t>(info.canid));
    return sendStandardFrame(frame.can_id, frame.data.data(), frame.dlc);
}

void DeviceX::ClearError_Type8(int& motor_index) {
    DisableMotor_Type8(motor_index);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    QueryPos_Type8(motor_index);
}

void DeviceX::SetZero_Type8(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    const encos::CommandFrame frame =
        encos::make_set_zero_command(static_cast<uint32_t>(info.canid));
    sendStandardFrame(frame.can_id, frame.data.data(), frame.dlc);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

bool DeviceX::SetMode_Type8(int& motor_index, int mode) {
    if (mode < 0) mode = 0;
    if (mode > 3) mode = 3;
    (*p_motors_data)[motor_index].send.mode = static_cast<uint8_t>(mode);
    return true;
}

bool DeviceX::SendCommand_Type8(int& motor_index) {
    const auto& motor = (*p_motors_data)[motor_index];
    const auto& info = motor.info;
    const auto& cmd = motor.send;
    const encos::Limits limits = encos::limits_from_info(info);
    const uint32_t can_id = static_cast<uint32_t>(info.canid);
    encos::CommandFrame frame {};

    if (cmd.mode == 0) {
        frame = encos::make_mit_command(
            can_id, limits, cmd.position, cmd.speed, cmd.kp, cmd.kd, cmd.torque);
    } else if (cmd.mode == 1) {
        frame = encos::make_position_command(
            can_id, cmd.position, cmd.speed, cmd.torque);
    } else if (cmd.mode == 2) {
        frame = encos::make_speed_command(
            can_id, cmd.speed, cmd.torque);
    } else {
        frame = encos::make_torque_command(
            can_id, limits, cmd.torque);
    }
    return sendStandardFrame(frame.can_id, frame.data.data(), frame.dlc);
}

bool DeviceX::QueryPos_Type8(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    const uint32_t can_id = static_cast<uint32_t>(info.canid);
    const encos::CommandFrame pos_query = encos::make_query_command(can_id, 1);
    const bool pos_ok = sendStandardFrame(pos_query.can_id, pos_query.data.data(), pos_query.dlc);
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    const encos::CommandFrame speed_query = encos::make_query_command(can_id, 2);
    const bool speed_ok = sendStandardFrame(speed_query.can_id, speed_query.data.data(), speed_query.dlc);
    return pos_ok && speed_ok;
}

// =========================================================
//  Type 9 (JC CAN servo)
// =========================================================

bool DeviceX::EnableMotor_Type9(int& motor_index) {
    auto& motor = (*p_motors_data)[motor_index];
    const auto& info = motor.info;
    uint8_t data[8] = {0};

    const uint8_t mode = (motor.send.mode <= 5) ? motor.send.mode : 4;
    make_jc_write16(data, 0x0060, mode == 0 ? 4 : mode);
    const bool mode_sent = sendStandardFrame(0x600U + static_cast<uint32_t>(info.canid), data, sizeof(data));
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    make_jc_write16(data, 0x00A2, 1);
    const bool enable_sent = sendStandardFrame(0x600U + static_cast<uint32_t>(info.canid), data, sizeof(data));
    if (mode_sent && enable_sent) {
        motor.recv.motor_state = 1;
    }
    return mode_sent && enable_sent;
}

bool DeviceX::DisableMotor_Type9(int& motor_index) {
    auto& motor = (*p_motors_data)[motor_index];
    const auto& info = motor.info;
    uint8_t data[8] = {0};

    make_jc_write16(data, 0x00A0, 1);
    const bool sent = sendStandardFrame(0x600U + static_cast<uint32_t>(info.canid), data, sizeof(data));
    if (sent) {
        motor.recv.motor_state = 0;
    }
    return sent;
}

void DeviceX::ClearError_Type9(int& motor_index) {
    auto& motor = (*p_motors_data)[motor_index];
    const auto& info = motor.info;
    uint8_t data[8] = {0};

    make_jc_read32(data, 0x000C);
    sendStandardFrame(0x600U + static_cast<uint32_t>(info.canid), data, sizeof(data));
    motor.recv.fault_message = 0;
}

void DeviceX::SetZero_Type9(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    uint8_t data[8] = {0};

    make_jc_write16(data, 0x00A7, 1);
    sendStandardFrame(0x600U + static_cast<uint32_t>(info.canid), data, sizeof(data));
}

bool DeviceX::SetMode_Type9(int& motor_index, int mode) {
    if (mode < 0) mode = 4;
    if (mode > 5) mode = 5;
    auto& motor = (*p_motors_data)[motor_index];
    motor.send.mode = static_cast<uint8_t>(mode);

    const auto& info = motor.info;
    uint8_t data[8] = {0};
    make_jc_write16(data, 0x0060, static_cast<uint16_t>(mode));
    return sendStandardFrame(0x600U + static_cast<uint32_t>(info.canid), data, sizeof(data));
}

bool DeviceX::SendCommand_Type9(int& motor_index) {
    const auto& motor = (*p_motors_data)[motor_index];
    const auto& info = motor.info;
    const auto& cmd = motor.send;
    uint8_t data[8] = {0};

    if (cmd.mode == 0 || cmd.mode == 2 || cmd.mode == 3 || cmd.mode == 4) {
        const int32_t pos_raw = static_cast<int32_t>(std::lround(jc_rad_to_deg_x100(cmd.position)));
        make_jc_write32(data, 0x0023, pos_raw);
    } else if (cmd.mode == 1) {
        const float rpm = cmd.speed * 60.0f / TWO_PI;
        const int32_t speed_raw = static_cast<int32_t>(std::lround(rpm * 100.0f));
        make_jc_write32(data, 0x0021, speed_raw);
    } else {
        const int16_t torque_raw = static_cast<int16_t>(std::lround(cmd.torque * 100.0f));
        make_jc_write16(data, 0x0020, static_cast<uint16_t>(torque_raw));
    }

    return sendStandardFrame(0x600U + static_cast<uint32_t>(info.canid), data, sizeof(data));
}

bool DeviceX::QueryPos_Type9(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    uint8_t data[8] = {0};

    make_jc_read32(data, 0x0008);
    return sendStandardFrame(0x600U + static_cast<uint32_t>(info.canid), data, sizeof(data));
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

        if (!is_eff && canID == 0 && frame_len >= 64 && p_motors_data) {
            for (int idx = 0; idx < static_cast<int>(p_motors_data->size()); ++idx) {
                Motor_CAN_Struct& motor = (*p_motors_data)[idx];
                if (motor.info.device_index != device_global_index ||
                    motor.info.api_type != 6 ||
                    motor.info.canid < 1 ||
                    motor.info.canid > 8) {
                    continue;
                }

                const std::size_t offset = static_cast<std::size_t>(motor.info.canid - 1) * 8U;
                const float pos_feedback = read_le_f32(&frame.data[offset]);
                const float cur_feedback = read_le_f32(&frame.data[offset + 4]);
                if (std::isfinite(pos_feedback)) {
                    motor.recv.current_position_f.store(pos_feedback);
                }
                if (std::isfinite(cur_feedback)) {
                    motor.recv.current_torque_f.store(cur_feedback);
                    motor.recv.current_iq_f.store(cur_feedback);
                }
                motor.recv.current_speed_f.store(0.0f);
                motor.recv.motor_id = static_cast<uint8_t>(motor.info.canid);
                motor.recv.mode = motor.send.mode;
                motor.recv.motor_state = 1;
                motor.recv.fault_message = 0;
            }
            continue;
        }

        if (is_eff) {
            parsed_motor_id = (canID >> 8) & 0xFF;
        } else {
            if (canID >= 0x140 && canID < 0x160) {
                parsed_motor_id = canID - 0x140;
            } else if (canID >= 0x180 && canID < 0x1A0) {
                parsed_motor_id = canID - 0x180;
            } else if (canID == 0x7FF && frame_len >= 4 && frame.data[2] == 0x01) {
                // ENCOS setting replies use broadcast ID 0x7FF and carry the motor ID in bytes 0-1.
                parsed_motor_id = (static_cast<int>(frame.data[0]) << 8) |
                                  static_cast<int>(frame.data[1]);
            } else if (canID >= 0x201 && canID <= 0x208) {
                parsed_motor_id = canID - 0x200;
            } else if (canID >= 0x581 && canID <= 0x5FF) {
                parsed_motor_id = canID - 0x580;
            } else if (canID >= 0x401 && canID <= 0x4FF) {
                // Haitai MIT frames set standard ID bit 10: 0x400 | device address.
                parsed_motor_id = static_cast<int>(canID & 0xFFU);
            } else if (canID > 0 && canID <= 0xFF) {
                // PFL28/L28, Haitai, and ENCOS standard frames: CAN ID is node id/device address.
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

        if (motor.info.api_type == 8 && !is_eff && canID == 0x7FF &&
            frame_len >= 4 && frame.data[2] == 0x01) {
            motor.recv.motor_id = static_cast<uint8_t>(parsed_motor_id);
            if (frame.data[3] == 0x03) {
                motor.recv.current_position_f.store(0.0f);
                motor.recv.fault_message = 0;
                motor.recv.motor_state = 1;
            } else {
                motor.recv.fault_message = 1;
                motor.recv.motor_state = 0;
            }
        } else if (motor.info.api_type == 1 && is_eff) {
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
        } else if (motor.info.api_type == 7 && !(frame.can_id & CAN_EFF_FLAG)) {
            std::array<uint8_t, 8> raw {};
            std::memcpy(raw.data(), frame.data, std::min<std::size_t>(frame_len, raw.size()));

            HaitaiFeedback feedback {};
            const float mit_pos_max = motor.recv.haitai_mit_limits_valid ?
                motor.recv.haitai_mit_pos_max_rad : HAITAI_MIT_DEFAULT_POS_MAX_RAD;
            const float mit_vel_max = motor.recv.haitai_mit_limits_valid ?
                motor.recv.haitai_mit_vel_max_rad_s : HAITAI_MIT_DEFAULT_VEL_MAX_RAD_S;
            const float mit_torque_max = motor.recv.haitai_mit_limits_valid ?
                motor.recv.haitai_mit_torque_max_nm : HAITAI_MIT_DEFAULT_TORQUE_MAX_NM;
            if (!parse_haitai_feedback(canID, raw, frame_len, feedback,
                                       mit_pos_max, mit_vel_max, mit_torque_max)) {
                continue;
            }

            if (feedback.has_position) {
                motor.recv.current_position_f.store(feedback.position_rad);
            }
            if (feedback.has_speed) {
                motor.recv.current_speed_f.store(feedback.speed_rad_s);
            }
            if (feedback.has_current) {
                motor.recv.current_iq_f.store(feedback.current_a);
                motor.recv.current_torque_f.store(feedback.current_a);
            }
            if (feedback.has_torque) {
                motor.recv.current_torque_f.store(feedback.torque_nm);
            }
            if (feedback.has_temperature) {
                motor.recv.current_temp_f.store(feedback.temperature_c);
            }
            if (feedback.has_version) {
                motor.recv.version_valid = true;
                motor.recv.boot_version = feedback.boot_version;
                motor.recv.app_version = feedback.app_version;
                motor.recv.hw_version = feedback.hw_version;
                motor.recv.can_proto_version = feedback.can_proto_version;
            }
            if (feedback.has_mit_limits) {
                motor.recv.haitai_mit_limits_valid = true;
                motor.recv.haitai_mit_pos_max_rad = feedback.mit_pos_max_rad;
                motor.recv.haitai_mit_vel_max_rad_s = feedback.mit_vel_max_rad_s;
                motor.recv.haitai_mit_torque_max_nm = feedback.mit_torque_max_nm;
            }
            if (feedback.has_mit_state) {
                motor.recv.haitai_mit_status = feedback.mit_status;
                motor.recv.haitai_mit_in_mode = feedback.mit_in_mode;
                motor.recv.haitai_mit_fault = feedback.mit_fault;
                motor.recv.mode = 4;
                motor.recv.fault_message = feedback.fault;
                motor.recv.haitai_fault_source_command = feedback.command;
                motor.recv.motor_state = 1;
            }
            if (feedback.has_status) {
                motor.recv.mode = feedback.mode;
                motor.recv.fault_message = feedback.fault;
                motor.recv.haitai_fault_source_command = feedback.command;
                motor.recv.motor_state = 1;
            }

            if ((feedback.command == 0xA3 || feedback.command == 0xC2 ||
                 feedback.command == 0xC3 || feedback.command == 0xC4) &&
                frame_len >= 7) {
                const uint16_t angle_single = static_cast<uint16_t>(frame.data[1]) |
                                              (static_cast<uint16_t>(frame.data[2]) << 8);
                motor.recv.motor_state = 1;
                motor.recv.mode = motor.send.mode;
                if (std::fabs(motor.recv.current_position_f.load()) < 1e-6f) {
                    const float single_turn_rad = static_cast<float>(angle_single) * HAITAI_COUNT_TO_RAD;
                    motor.recv.current_position_f.store(haitai_wrap_to_signed_pi(single_turn_rad));
                }
            } else if ((feedback.command == 0xA1 || feedback.command == 0xC0 ||
                        feedback.command == 0xA2 || feedback.command == 0xC1)) {
                motor.recv.mode = motor.send.mode;
                motor.recv.motor_state = 1;
            } else if (feedback.command == 0xA4) {
                motor.recv.mode = motor.send.mode;
                motor.recv.motor_state = 1;
            } else if (feedback.command == 0xA0) {
                motor.recv.motor_state = 1;
            } else if (feedback.command == 0xAF) {
                motor.recv.fault_message = feedback.fault;
                motor.recv.motor_state = 1;
            } else if (feedback.command == 0xF0) {
                motor.recv.motor_state = 1;
            }

            motor.recv.motor_id = static_cast<uint8_t>(parsed_motor_id);
            motor.recv.feedback_sequence.fetch_add(1, std::memory_order_relaxed);
        } else if (motor.info.api_type == 8 && !(frame.can_id & CAN_EFF_FLAG)) {
            const encos::Feedback feedback = encos::parse_feedback(
                frame.data, frame_len, encos::limits_from_info(motor.info));
            if (!feedback.valid) {
                continue;
            }

            motor.recv.motor_id = static_cast<uint8_t>(parsed_motor_id);
            motor.recv.fault_message = feedback.error;
            if (feedback.has_position) {
                motor.recv.current_position_f.store(feedback.position_rad);
            }
            if (feedback.has_speed) {
                motor.recv.current_speed_f.store(feedback.speed_rad_s);
            }
            if (feedback.has_current) {
                motor.recv.current_iq_f.store(feedback.current_a);
                motor.recv.current_torque_f.store(feedback.current_a);
            }
            if (feedback.has_temperature) {
                motor.recv.current_temp_f.store(feedback.temperature_c);
            }
            if (feedback.mode != 0) {
                motor.recv.mode = feedback.mode;
            }
            if (feedback.has_state) {
                motor.recv.motor_state = feedback.state;
            }
        } else if (motor.info.api_type == 9 && !(frame.can_id & CAN_EFF_FLAG) && frame_len >= 8) {
            const uint16_t reg = static_cast<uint16_t>(
                (static_cast<uint16_t>(frame.data[1]) << 8) | frame.data[2]);

            motor.recv.motor_id = static_cast<uint8_t>(parsed_motor_id);
            motor.recv.motor_state = 1;
            if (reg == 0x0008 || reg == 0x0023) {
                const int32_t pos_raw = jc_read_be_i32(&frame.data[4]);
                motor.recv.current_position_f.store(jc_deg_x100_to_rad(pos_raw));
                motor.recv.mode = motor.send.mode;
                motor.recv.fault_message = 0;
            } else if (reg == 0x0021) {
                const int32_t speed_raw = jc_read_be_i32(&frame.data[4]);
                const float rpm = static_cast<float>(speed_raw) * 0.01f;
                motor.recv.current_speed_f.store(rpm * TWO_PI / 60.0f);
            } else if (reg == 0x0020) {
                const int16_t tq_raw = static_cast<int16_t>(
                    (static_cast<uint16_t>(frame.data[4]) << 8) | frame.data[5]);
                motor.recv.current_torque_f.store(static_cast<float>(tq_raw) * 0.01f);
            } else if (reg == 0x000C) {
                motor.recv.fault_message = frame.data[7];
            } else if (reg == 0x0060) {
                motor.recv.mode = frame.data[5];
            }
            motor.recv.feedback_sequence.fetch_add(1, std::memory_order_relaxed);
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
