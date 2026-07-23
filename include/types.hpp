#pragma once
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>

// 工具函数：设为 inline 防止重定义错误
inline int float_to_uint(float x, float x_min, float x_max, int bits)
{
    if (!std::isfinite(x) || !std::isfinite(x_min) || !std::isfinite(x_max) ||
        x_max <= x_min || bits <= 0 || bits >= 31) {
        return 0;
    }
    x = std::clamp(x, x_min, x_max);
    return static_cast<int>((x - x_min) * static_cast<float>((1U << bits) - 1U) /
                            (x_max - x_min));
}

inline float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    if (!std::isfinite(x_min) || !std::isfinite(x_max) ||
        x_max <= x_min || bits <= 0 || bits >= 31) {
        return x_min;
    }
    return static_cast<float>(x_int) * (x_max - x_min) /
           static_cast<float>((1U << bits) - 1U) + x_min;
}

// 结构体定义
struct Motor_CAN_Send_Struct
{
    uint8_t mode = 0;
    float position = 0.0f;
    float speed = 0.0f;
    float kp = 0.0f;
    float kd = 0.0f;
    float torque = 0.0f;
};

struct Motor_CAN_Receive_Struct
{
    uint8_t master_id;
    uint8_t motor_id;
    uint8_t motor_state;
    uint8_t mode;
    std::atomic<uint8_t> fault_message;
    std::atomic<uint8_t> haitai_fault_source_command;

    std::atomic<float> current_position_f;
    std::atomic<float> current_speed_f;
    std::atomic<float> current_torque_f;
    std::atomic<float> current_temp_f;
    std::atomic<float> current_iq_f;
    bool version_valid;
    uint16_t boot_version;
    uint16_t app_version;
    uint16_t hw_version;
    uint8_t can_proto_version;
    bool haitai_mit_limits_valid;
    float haitai_mit_pos_max_rad;
    float haitai_mit_vel_max_rad_s;
    float haitai_mit_torque_max_nm;
    std::atomic<uint8_t> haitai_mit_status;
    std::atomic<bool> haitai_mit_in_mode;
    std::atomic<bool> haitai_mit_fault;
    std::atomic<unsigned long long> feedback_sequence;
    uint64_t last_feedback_ns;
    uint64_t last_version_query_ns;
    uint64_t last_mit_limits_query_ns;
    uint64_t poll_sequence;

    Motor_CAN_Receive_Struct()
    {
        master_id = 0;
        motor_id = 0;
        motor_state = 0;
        mode = 0;
        fault_message = 0;
        haitai_fault_source_command = 0;
        current_position_f = 0.0f;
        current_speed_f = 0.0f;
        current_torque_f = 0.0f;
        current_temp_f = 0.0f;
        current_iq_f = 0.0f;
        version_valid = false;
        boot_version = 0;
        app_version = 0;
        hw_version = 0;
        can_proto_version = 0;
        haitai_mit_limits_valid = false;
        haitai_mit_pos_max_rad = 95.5f;
        haitai_mit_vel_max_rad_s = 45.0f;
        haitai_mit_torque_max_nm = 18.0f;
        haitai_mit_status = 0;
        haitai_mit_in_mode = false;
        haitai_mit_fault = false;
        feedback_sequence = 0;
        last_feedback_ns = 0;
        last_version_query_ns = 0;
        last_mit_limits_query_ns = 0;
        poll_sequence = 0;
    }

    Motor_CAN_Receive_Struct(const Motor_CAN_Receive_Struct &other)
    {
        master_id = other.master_id;
        motor_id = other.motor_id;
        fault_message.store(other.fault_message.load());
        haitai_fault_source_command.store(other.haitai_fault_source_command.load());
        motor_state = other.motor_state;
        mode = other.mode;
        current_position_f.store(other.current_position_f.load());
        current_speed_f.store(other.current_speed_f.load());
        current_torque_f.store(other.current_torque_f.load());
        current_temp_f.store(other.current_temp_f.load());
        current_iq_f.store(other.current_iq_f.load());
        version_valid = other.version_valid;
        boot_version = other.boot_version;
        app_version = other.app_version;
        hw_version = other.hw_version;
        can_proto_version = other.can_proto_version;
        haitai_mit_limits_valid = other.haitai_mit_limits_valid;
        haitai_mit_pos_max_rad = other.haitai_mit_pos_max_rad;
        haitai_mit_vel_max_rad_s = other.haitai_mit_vel_max_rad_s;
        haitai_mit_torque_max_nm = other.haitai_mit_torque_max_nm;
        haitai_mit_status.store(other.haitai_mit_status.load());
        haitai_mit_in_mode.store(other.haitai_mit_in_mode.load());
        haitai_mit_fault.store(other.haitai_mit_fault.load());
        feedback_sequence.store(other.feedback_sequence.load());
        last_feedback_ns = other.last_feedback_ns;
        last_version_query_ns = other.last_version_query_ns;
        last_mit_limits_query_ns = other.last_mit_limits_query_ns;
        poll_sequence = other.poll_sequence;
    }

    Motor_CAN_Receive_Struct &operator=(const Motor_CAN_Receive_Struct &other)
    {
        if (this != &other)
        {
            master_id = other.master_id;
            motor_id = other.motor_id;
            fault_message.store(other.fault_message.load());
            haitai_fault_source_command.store(other.haitai_fault_source_command.load());
            motor_state = other.motor_state;
            mode = other.mode;
            current_position_f.store(other.current_position_f.load());
            current_speed_f.store(other.current_speed_f.load());
            current_torque_f.store(other.current_torque_f.load());
            current_temp_f.store(other.current_temp_f.load());
            current_iq_f.store(other.current_iq_f.load());
            version_valid = other.version_valid;
            boot_version = other.boot_version;
            app_version = other.app_version;
            hw_version = other.hw_version;
            can_proto_version = other.can_proto_version;
            haitai_mit_limits_valid = other.haitai_mit_limits_valid;
            haitai_mit_pos_max_rad = other.haitai_mit_pos_max_rad;
            haitai_mit_vel_max_rad_s = other.haitai_mit_vel_max_rad_s;
            haitai_mit_torque_max_nm = other.haitai_mit_torque_max_nm;
            haitai_mit_status.store(other.haitai_mit_status.load());
            haitai_mit_in_mode.store(other.haitai_mit_in_mode.load());
            haitai_mit_fault.store(other.haitai_mit_fault.load());
            feedback_sequence.store(other.feedback_sequence.load());
            last_feedback_ns = other.last_feedback_ns;
            last_version_query_ns = other.last_version_query_ns;
            last_mit_limits_query_ns = other.last_mit_limits_query_ns;
            poll_sequence = other.poll_sequence;
        }
        return *this;
    }
};

struct Motor_CAN_Info_Struct
{
    int num;
    std::string name;
    std::string type;
    int api_type;
    std::string device_name;
    int device_index;
    int chan;
    int canid;

    float p_min, p_max;
    float v_min, v_max;
    float kp_min, kp_max;
    float kd_min, kd_max;
    float t_min, t_max;
    float current_min, current_max;
    float torque_constant;
    float kp_in_use, kd_in_use;
    float pos_min, pos_max;
    
};

struct Motor_CAN_Struct
{
    Motor_CAN_Send_Struct send;
    Motor_CAN_Receive_Struct recv;
    Motor_CAN_Info_Struct info;
};

inline bool motor_uses_extended_frame(int api_type)
{
    return api_type == 1 || api_type == 5;
}

inline bool motor_mode_supported(int api_type, int mode)
{
    switch (api_type) {
        case 1: return mode == 0 || mode == 2;
        case 2: return mode >= 1 && mode <= 3;
        case 3: return mode >= 0 && mode <= 3;
        case 5: return mode >= 0 && mode <= 3;
        case 7: return mode >= 0 && mode <= 4;
        case 8: return mode >= 0 && mode <= 3;
        case 9: return mode >= 1 && mode <= 5;
        default: return false;
    }
}

inline uint64_t steady_time_ns()
{
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

inline bool sanitize_motor_command(const Motor_CAN_Info_Struct& info,
                                   Motor_CAN_Send_Struct& command)
{
    if (!std::isfinite(command.position) || !std::isfinite(command.speed) ||
        !std::isfinite(command.torque) || !std::isfinite(command.kp) ||
        !std::isfinite(command.kd)) {
        return false;
    }

    const float position_min = info.pos_min < info.pos_max ? info.pos_min : info.p_min;
    const float position_max = info.pos_min < info.pos_max ? info.pos_max : info.p_max;
    const bool current_limit_command = info.current_min < info.current_max && (
        (info.api_type == 1 && command.mode == 2) ||
        (info.api_type == 7 && command.mode == 1) ||
        (info.api_type == 8 && (command.mode == 1 || command.mode == 2)));
    if (info.api_type == 1 && command.mode == 2 &&
        std::fabs(command.torque) <= 1e-6f) {
        return false;
    }
    const float effort_min = current_limit_command ? info.current_min : info.t_min;
    const float effort_max = current_limit_command ? info.current_max : info.t_max;
    if (!(position_min < position_max) || !(info.v_min < info.v_max) ||
        !(effort_min < effort_max)) {
        return false;
    }
    command.position = std::clamp(command.position, position_min, position_max);
    command.speed = std::clamp(command.speed, info.v_min, info.v_max);
    command.torque = std::clamp(command.torque, effort_min, effort_max);
    if (info.kp_min < info.kp_max) command.kp = std::clamp(command.kp, info.kp_min, info.kp_max);
    if (info.kd_min < info.kd_max) command.kd = std::clamp(command.kd, info.kd_min, info.kd_max);
    return true;
}
