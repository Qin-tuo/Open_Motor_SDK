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
    uint8_t mode;
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
    uint32_t fault_message;
    uint8_t motor_state;
    uint8_t mode;

    std::atomic<float> current_position_f;
    std::atomic<float> current_speed_f;
    std::atomic<float> current_torque_f;
    std::atomic<float> current_temp_f;
    std::atomic<float> current_iq_f;

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
        }
        return *this;
    }
};

typedef struct
{
    int num = 0;
    std::string name;
    std::string type;
    int api_type = 0;
    std::string device_name;
    int device_index = -1;
    int chan = 0;
    int canid = 0;

    float p_min = 0.0f, p_max = 0.0f;
    float v_min = 0.0f, v_max = 0.0f;
    float kp_min = 0.0f, kp_max = 0.0f;
    float kd_min = 0.0f, kd_max = 0.0f;
    float t_min = 0.0f, t_max = 0.0f;
    float kp_in_use = 0.0f, kd_in_use = 0.0f;
    float pos_min = 0.0f, pos_max = 0.0f;

    float type5_rated_torque_nm = 0.0f;
    float type5_peak_torque_nm = 0.0f;
    float type5_torque_constant = 0.0f;
    float type5_encoder_cpr = 65536.0f;
    float type5_dir_sign = 1.0f;
    float type5_zero_offset_rad = 0.0f;
    int type5_mode_position = 1;
    int type5_mode_speed = 3;
    int type5_mode_current = 4;
    bool type5_fast_write = true;
    float type5_profile_velocity = 0.0f;
    float type5_profile_acc = 0.0f;
    float type5_profile_dec = 0.0f;
    
} Motor_CAN_Info_Struct;

typedef struct
{
    Motor_CAN_Send_Struct send;
    Motor_CAN_Receive_Struct recv;
    Motor_CAN_Info_Struct info;
} Motor_CAN_Struct;
