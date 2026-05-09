#pragma once
#include <vector>
#include <string>
#include <atomic>
#include <cstdint>
#include <cmath>

using uint = unsigned int;

// 工具函数：设为 inline 防止重定义错误
inline int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max)
        x = x_max;
    else if (x < x_min)
        x = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

inline float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

// 结构体定义
typedef struct
{
    uint8_t mode = 0;
    float position = 0.0f;
    float speed = 0.0f;
    float kp = 0.0f;
    float kd = 0.0f;
    float torque = 0.0f;
} Motor_CAN_Send_Struct;

struct Motor_CAN_Receive_Struct
{
    uint8_t master_id;
    uint8_t motor_id;
    uint8_t fault_message;
    uint8_t motor_state;
    uint8_t mode;

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
    uint8_t haitai_mit_status;
    bool haitai_mit_in_mode;
    bool haitai_mit_fault;

    Motor_CAN_Receive_Struct()
    {
        master_id = 0;
        motor_id = 0;
        fault_message = 0;
        motor_state = 0;
        mode = 0;
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
    }

    Motor_CAN_Receive_Struct(const Motor_CAN_Receive_Struct &other)
    {
        master_id = other.master_id;
        motor_id = other.motor_id;
        fault_message = other.fault_message;
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
        haitai_mit_status = other.haitai_mit_status;
        haitai_mit_in_mode = other.haitai_mit_in_mode;
        haitai_mit_fault = other.haitai_mit_fault;
    }

    Motor_CAN_Receive_Struct &operator=(const Motor_CAN_Receive_Struct &other)
    {
        if (this != &other)
        {
            master_id = other.master_id;
            motor_id = other.motor_id;
            fault_message = other.fault_message;
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
            haitai_mit_status = other.haitai_mit_status;
            haitai_mit_in_mode = other.haitai_mit_in_mode;
            haitai_mit_fault = other.haitai_mit_fault;
        }
        return *this;
    }
};

typedef struct
{
    int num;
    std::string name;
    std::string type;
    int api_type;
    std::string device_name;
    int device_index;
    int chan;
    int canid;
    std::string port;
    int baud;

    float p_min, p_max;
    float v_min, v_max;
    float kp_min, kp_max;
    float kd_min, kd_max;
    float t_min, t_max;
    float kp_in_use, kd_in_use;
    float pos_min, pos_max;
    
} Motor_CAN_Info_Struct;

typedef struct
{
    Motor_CAN_Send_Struct send;
    Motor_CAN_Receive_Struct recv;
    Motor_CAN_Info_Struct info;
} Motor_CAN_Struct;
