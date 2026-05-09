#include "haitai_protocol.hpp"

#include <cmath>
#include <cstring>
#include <limits>

namespace {

constexpr float kTwoPi = 6.28318530718f;
constexpr float kCountPerTurn = 16384.0f;
constexpr float kSpeedScale = 100.0f;   // 0.01 rpm
constexpr float kCurrentScale = 1000.0f; // 0.001 A
constexpr float kMitKpMax = 500.0f;
constexpr float kMitKdMax = 5.0f;

int32_t clamp_i32(float value) {
    const float hi = static_cast<float>(std::numeric_limits<int32_t>::max());
    const float lo = static_cast<float>(std::numeric_limits<int32_t>::min());
    if (value > hi) value = hi;
    if (value < lo) value = lo;
    return static_cast<int32_t>(std::lround(value));
}

uint16_t clamp_u16(float value) {
    if (value < 0.0f) value = 0.0f;
    if (value > static_cast<float>(std::numeric_limits<uint16_t>::max())) {
        value = static_cast<float>(std::numeric_limits<uint16_t>::max());
    }
    return static_cast<uint16_t>(std::lround(value));
}

void write_le_i32(uint8_t* dst, int32_t value) {
    const uint32_t raw = static_cast<uint32_t>(value);
    dst[0] = static_cast<uint8_t>(raw & 0xFF);
    dst[1] = static_cast<uint8_t>((raw >> 8) & 0xFF);
    dst[2] = static_cast<uint8_t>((raw >> 16) & 0xFF);
    dst[3] = static_cast<uint8_t>((raw >> 24) & 0xFF);
}

void write_le_u16(uint8_t* dst, uint16_t value) {
    dst[0] = static_cast<uint8_t>(value & 0xFF);
    dst[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

int32_t read_le_i32(const uint8_t* src) {
    const uint32_t raw = static_cast<uint32_t>(src[0]) |
                         (static_cast<uint32_t>(src[1]) << 8) |
                         (static_cast<uint32_t>(src[2]) << 16) |
                         (static_cast<uint32_t>(src[3]) << 24);
    return static_cast<int32_t>(raw);
}

uint16_t read_le_u16(const uint8_t* src) {
    return static_cast<uint16_t>(src[0]) |
           (static_cast<uint16_t>(src[1]) << 8);
}

int16_t read_le_i16(const uint8_t* src) {
    return static_cast<int16_t>(read_le_u16(src));
}

int32_t rad_to_count(float rad) {
    return clamp_i32(rad * (kCountPerTurn / kTwoPi));
}

int32_t rad_s_to_rpm_x100(float rad_s) {
    return clamp_i32(rad_s * (60.0f / kTwoPi) * kSpeedScale);
}

int32_t amp_to_milliamp(float amp) {
    return clamp_i32(amp * kCurrentScale);
}

float count_to_rad(int32_t count) {
    return static_cast<float>(count) * (kTwoPi / kCountPerTurn);
}

float rpm_x100_to_rad_s(int32_t raw) {
    const float rpm = static_cast<float>(raw) / kSpeedScale;
    return rpm * (kTwoPi / 60.0f);
}

float milliamp_to_amp(int32_t raw) {
    return static_cast<float>(raw) / kCurrentScale;
}

uint32_t float_to_uint(float value, float min_value, float max_value, uint8_t bits) {
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

float uint_to_float(uint32_t raw, float min_value, float max_value, uint8_t bits) {
    const uint32_t max_raw = (1U << bits) - 1U;
    const float span = max_value - min_value;
    return static_cast<float>(raw) * span / static_cast<float>(max_raw) + min_value;
}

}  // namespace

HaitaiCommandFrame build_haitai_command(uint8_t mode, float position_rad,
                                        float speed_rad_s, float current_a,
                                        uint32_t can_id) {
    HaitaiCommandFrame frame {};
    frame.can_id = can_id & 0x7FFU;
    frame.dlc = 5;

    uint8_t cmd = 0xC2;
    int32_t value = rad_to_count(position_rad);

    switch (mode) {
        case 1:
            cmd = 0xC0;
            value = amp_to_milliamp(current_a);
            break;
        case 2:
            cmd = 0xC1;
            value = rad_s_to_rpm_x100(speed_rad_s);
            break;
        case 3:
            cmd = 0xC3;
            value = rad_to_count(position_rad);
            break;
        case 0:
        default:
            cmd = 0xC2;
            value = rad_to_count(position_rad);
            break;
    }

    frame.data[0] = cmd;
    write_le_i32(&frame.data[1], value);
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

    const uint32_t pos_raw = float_to_uint(position_rad, -pos_max_rad, pos_max_rad, 16);
    const uint32_t vel_raw = float_to_uint(speed_rad_s, -vel_max_rad_s, vel_max_rad_s, 12);
    const uint32_t kp_raw = float_to_uint(kp, 0.0f, kMitKpMax, 12);
    const uint32_t kd_raw = float_to_uint(kd, 0.0f, kMitKdMax, 12);
    const uint32_t torque_raw = float_to_uint(torque_nm, -torque_max_nm, torque_max_nm, 12);

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
    write_le_u16(&frame.data[1], clamp_u16(pos_max_rad * 10.0f));
    write_le_u16(&frame.data[3], clamp_u16(vel_max_rad_s * 100.0f));
    write_le_u16(&frame.data[5], clamp_u16(torque_max_nm * 100.0f));
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
            out.boot_version = read_le_u16(&data[1]);
            out.app_version = read_le_u16(&data[3]);
            out.hw_version = read_le_u16(&data[5]);
            out.can_proto_version = data[7];
            return true;
        case 0xA1:
        case 0xC0:
            if (dlc != 5) return false;
            out.has_current = true;
            out.current_a = milliamp_to_amp(read_le_i32(&data[1]));
            return true;
        case 0xA2:
        case 0xC1:
            if (dlc != 5) return false;
            out.has_speed = true;
            out.speed_rad_s = rpm_x100_to_rad_s(read_le_i32(&data[1]));
            return true;
        case 0xA3:
        case 0xC2:
        case 0xC3:
        case 0xC4:
            if (dlc != 7) return false;
            out.has_position = true;
            out.position_rad = count_to_rad(read_le_i32(&data[3]));
            return true;
        case 0xA4:
            if (dlc != 8) return false;
            out.has_temperature = true;
            out.has_current = true;
            out.has_speed = true;
            out.has_position = true;
            out.temperature_c = static_cast<float>(data[1]);
            out.current_a = milliamp_to_amp(read_le_i16(&data[2]));
            out.speed_rad_s = rpm_x100_to_rad_s(read_le_i16(&data[4]));
            out.position_rad = count_to_rad(read_le_u16(&data[6]));
            return true;
        case 0xAE:
        case 0xCF:
            if (dlc != 8) return false;
            out.has_status = true;
            out.has_temperature = true;
            out.bus_voltage_v = static_cast<float>(read_le_u16(&data[1])) / 100.0f;
            out.bus_current_a = static_cast<float>(read_le_u16(&data[3])) / 100.0f;
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
            return dlc == 3;
        case 0xF0:
            if (dlc != 7) return false;
            out.has_mit_limits = true;
            out.mit_pos_max_rad = static_cast<float>(read_le_u16(&data[1])) * 0.1f;
            out.mit_vel_max_rad_s = static_cast<float>(read_le_u16(&data[3])) * 0.01f;
            out.mit_torque_max_nm = static_cast<float>(read_le_u16(&data[5])) * 0.01f;
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
            out.position_rad = uint_to_float(pos_raw, -mit_pos_max_rad, mit_pos_max_rad, 16);
            out.speed_rad_s = uint_to_float(vel_raw, -mit_vel_max_rad_s, mit_vel_max_rad_s, 12);
            out.torque_nm = uint_to_float(torque_raw, -mit_torque_max_nm, mit_torque_max_nm, 12);
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
