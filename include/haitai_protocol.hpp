#pragma once

#include <array>
#include <cstdint>

struct HaitaiCommandFrame {
    uint32_t can_id = 0;
    uint8_t dlc = 0;
    std::array<uint8_t, 8> data{};
};

struct HaitaiFeedback {
    bool has_position = false;
    bool has_speed = false;
    bool has_current = false;
    bool has_torque = false;
    bool has_temperature = false;
    bool has_status = false;
    bool has_version = false;
    bool has_mit_limits = false;
    bool has_mit_state = false;

    float position_rad = 0.0f;
    float speed_rad_s = 0.0f;
    float current_a = 0.0f;
    float torque_nm = 0.0f;
    float temperature_c = 0.0f;
    float bus_voltage_v = 0.0f;
    float bus_current_a = 0.0f;
    float mit_pos_max_rad = 0.0f;
    float mit_vel_max_rad_s = 0.0f;
    float mit_torque_max_nm = 0.0f;

    uint16_t boot_version = 0;
    uint16_t app_version = 0;
    uint16_t hw_version = 0;
    uint8_t can_proto_version = 0;
    uint8_t mode = 0;
    uint8_t fault = 0;
    uint8_t mit_status = 0;
    bool mit_in_mode = false;
    bool mit_fault = false;
    uint8_t command = 0;
};

HaitaiCommandFrame build_haitai_command(uint8_t mode, float position_rad,
                                        float speed_rad_s, float current_a,
                                        uint32_t can_id = 0x01);

HaitaiCommandFrame build_haitai_mit_command(float position_rad, float speed_rad_s,
                                            float kp, float kd, float torque_nm,
                                            float pos_max_rad,
                                            float vel_max_rad_s,
                                            float torque_max_nm,
                                            uint32_t can_id = 0x01);

HaitaiCommandFrame build_haitai_mit_limits_config(float pos_max_rad,
                                                  float vel_max_rad_s,
                                                  float torque_max_nm,
                                                  uint32_t can_id = 0x01);

HaitaiCommandFrame make_haitai_simple_query(uint8_t cmd, uint32_t can_id);

bool parse_haitai_feedback(uint32_t can_id, const std::array<uint8_t, 8>& data,
                           uint8_t dlc, HaitaiFeedback& out,
                           float mit_pos_max_rad = 95.5f,
                           float mit_vel_max_rad_s = 45.0f,
                           float mit_torque_max_nm = 18.0f);
