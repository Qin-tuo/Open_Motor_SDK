#include "motor_many.h"


#ifdef __MICROLIB  // 有无启用MicroLIB库
#include <string.h>
#endif


many_data_s many_data_port[MANY_PORT_SIZE][MANY_DATA_BUF_MAX_LEN];

const uint8_t many_get_cmd[MANY_GET_MAX_NUM][2] =
{
    {0xFF, 0xFF},
    {0xFF, 0xFE},
    {0x17, 0x01},
};


p_many_data_s motor_get_many_pointer(port_t portx)
{
    if (portx < 1 || portx > MANY_PORT_SIZE)
    {
        MOTOR_ERR();
        return NULL;
    }

    return many_data_port[portx - 1];
}


/**
 * @brief 一拖多 DQ 电压模式
 * @param portx can通道（需要在 motor.c 中修改 port_maping 结构体数组进行映射）
 * @param id 电机 ID
 * @param volt Q 相电压，单位：（V），例：0.3 -> 0.3V
 */
void motor_many_dq_volt(port_t portx, const uint8_t id, const float vol)
{
    p_many_data_s p_many_data = motor_get_many_pointer(portx);
    const int16_t vol_int16 = vol_float2int(vol, TINT16);
    const uint16_t index = id - 1;

    if (p_many_data->mode != MODE_VOLTAGE)
    {
        p_many_data->mode = MODE_VOLTAGE;
        for (int i = 0; i < MANY_DATA_BUF_MAX_LEN / sizeof(int16_t); i++)
        {
            p_many_data->voltage[i] = 0;
        }
    }

    p_many_data->voltage[index] = vol_int16;
}


/**
 * @brief 一拖多 DQ 电流模式
 * @param portx can通道（需要在 motor.c 中修改 port_maping 结构体数组进行映射）
 * @param id 电机 ID
 * @param cur Q 相电流，单位：（A），例：0.3 -> 0.3A
 */
void motor_many_dq_current(port_t portx, const uint8_t id, const float cur)
{
    p_many_data_s p_many_data = motor_get_many_pointer(portx);
    const int16_t cur_int16 = cur_float2int(cur, TINT16);
    const uint16_t index = id - 1;

    if (p_many_data->mode != MODE_CURRENT)
    {
        p_many_data->mode = MODE_CURRENT;
        for (int i = 0; i < MANY_DATA_BUF_MAX_LEN / sizeof(int16_t); i++)
        {
            p_many_data->current[i] = 0;
        }
    }

    p_many_data->current[index] = cur_int16;
}


/**
 * @brief 一拖多 位置模式
 * @param portx can通道（需要在 motor.c 中修改 port_maping 结构体数组进行映射）
 * @param id 电机 ID
 * @param pos 目标位置，单位可为转（r）、弧度（rad）、或度（°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 */
void motor_many_pos(port_t portx, const uint8_t id, const float pos)
{
    p_many_data_s p_many_data = motor_get_many_pointer(portx);
    const float pos_turns = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const int16_t pos_int16 = pos_float2int(pos_turns, TINT16);
    const uint16_t index = id - 1;

    if (p_many_data->mode != MODE_POSITION)
    {
        p_many_data->mode = MODE_POSITION;
        for (int i = 0; i < MANY_DATA_BUF_MAX_LEN / sizeof(int16_t); i++)
        {
            p_many_data->position[i] = NAN_INT16;
        }
    }

    p_many_data->position[index] = pos_int16;
}


/**
 * @brief 一拖多 速度模式
 * @param portx can通道（需要在 motor.c 中修改 port_maping 结构体数组进行映射）
 * @param id 电机 ID
 * @param vel 目标速度，单位可为转（rps）、弧度（rad/s）、或度（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 */
void motor_many_vel(port_t portx, const uint8_t id, const float vel)
{
    p_many_data_s p_many_data = motor_get_many_pointer(portx);
    const float vel_turns = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const int16_t vel_int16 = vel_float2int(vel_turns, TINT16);
    const uint16_t index = id - 1;

    if (p_many_data->mode != MODE_VELOCITY)
    {
        p_many_data->mode = MODE_VELOCITY;
        for (int i = 0; i < MANY_DATA_BUF_MAX_LEN / sizeof(int16_t); i++)
        {
            p_many_data->velocity[i] = 0;
        }
    }

    p_many_data->velocity[index] = vel_int16;
}


/**
 * @brief 一拖多 力矩模式
 * @param portx can通道（需要在 motor.c 中修改 port_maping 结构体数组进行映射）
 * @param id 电机 ID
 * @param tqe 目标力矩，单位牛米（NM），注：需要在 motor.c 文件中修改电机数量和类型，以修正电机力矩
 */
void motor_many_tqe(port_t portx, const uint8_t id, const float tqe)
{
    p_many_data_s p_many_data = motor_get_many_pointer(portx);
    const float tqe_float = tqe_adjust(tqe, motor_get_model2(portx, id));
    const int16_t tqe_int16 = tqe_float2int(tqe_float, TINT16);
    const uint16_t index = id - 1;

    if (p_many_data->mode != MODE_TORQUE)
    {
        p_many_data->mode = MODE_TORQUE;
        for (int i = 0; i < MANY_DATA_BUF_MAX_LEN / sizeof(int16_t); i++)
        {
            p_many_data->torque[i] = 0;
        }
    }

    p_many_data->torque[index] = tqe_int16;
}


/**
 * @brief 一拖多 设置超时时间
 * @param portx can通道（需要在 motor.c 中修改 port_maping 结构体数组进行映射）
 * @param id 电机 ID
 * @param t_ms 超时时间，单位：毫秒（ms）
 */
void motor_many_time_out(port_t portx, const uint8_t id, const int16_t t_ms)
{
    p_many_data_s p_many_data = motor_get_many_pointer(portx);
    const uint16_t index = id - 1;

    if (p_many_data->mode != MODE_TIME_OUT)
    {
        p_many_data->mode = MODE_TIME_OUT;
        for (int i = 0; i < MANY_DATA_BUF_MAX_LEN / sizeof(int16_t); i++)
        {
            p_many_data->timeout[i] = 0;
        }
    }

    p_many_data->timeout[index] = t_ms;
}


/**
 * @brief 一拖多 位置速度模式，以目标速度运动到目标位置，不限制加速度和最大输出力矩
 * @param portx can通道（需要在 motor.c 中修改 port_maping 结构体数组进行映射）
 * @param id 电机 ID
 * @param pos 目标位置，单位可为转（r）、弧度（rad）、或度（°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param vel 目标速度，单位可为转（rps）、弧度（rad/s）、或度（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 */
void motor_many_pos_vel(port_t portx, const uint8_t id, const float pos, const float vel)
{
    p_many_data_s p_many_data = motor_get_many_pointer(portx);

    const float pos_turns = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float vel_turns = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);

    const int16_t pos_int16 = pos_float2int(pos_turns, TINT16);
    const int16_t vel_int16 = vel_float2int(vel_turns, TINT16);
    const uint16_t index = id - 1;

    if (p_many_data->mode != MODE_POS_VEL_TQE)
    {
        p_many_data->mode = MODE_POS_VEL_TQE;
        for (int i = 0; i < MANY_DATA_BUF_MAX_LEN / sizeof(many_pos_vel_tqe_s); i++)
        {
            p_many_data->pos_vel_tqe[i].pos = NAN_INT16;
            p_many_data->pos_vel_tqe[i].vel = 0;
            p_many_data->pos_vel_tqe[i].tqe = 0;
        }
    }

    p_many_data->pos_vel_tqe[index].pos = pos_int16;
    p_many_data->pos_vel_tqe[index].vel = vel_int16;
    p_many_data->pos_vel_tqe[index].tqe = NAN_INT16;
}


/**
 * @brief 一拖多 位置速度模式，以目标速度运动到目标位置，并限制最大输出力矩
 * @param portx can通道（需要在 motor.c 中修改 port_maping 结构体数组进行映射）
 * @param id 电机 ID
 * @param pos 目标位置，单位可为转（r）、弧度（rad）、或度（°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param vel 目标速度，单位可为转（rps）、弧度（rad/s）、或度（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param tqe 最大力矩，电机转动过程中输出力矩不会超过这个值，单位牛米（NM），注：需要在 motor.c 文件中修改电机数量和类型，以修正电机力矩
 */
void motor_many_pos_vel_MAXtqe(port_t portx, const uint8_t id, const float pos, const float vel, const float tqe)
{
    p_many_data_s p_many_data = motor_get_many_pointer(portx);


    const float pos_turns = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float vel_turns = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float tqe_float = tqe_adjust(tqe, motor_get_model2(portx, id));

    const int16_t pos_int16 = pos_float2int(pos_turns, TINT16);
    const int16_t vel_int16 = vel_float2int(vel_turns, TINT16);
    const int16_t tqe_int16 = tqe_float2int(tqe_float, TINT16);
    const uint16_t index = id - 1;

    if (p_many_data->mode != MODE_POS_VEL_TQE)
    {
        p_many_data->mode = MODE_POS_VEL_TQE;
        for (int i = 0; i < MANY_DATA_BUF_MAX_LEN / sizeof(many_pos_vel_tqe_s); i++)
        {
            p_many_data->pos_vel_tqe[i].pos = NAN_INT16;
            p_many_data->pos_vel_tqe[i].vel = 0;
            p_many_data->pos_vel_tqe[i].tqe = 0;
        }
    }

    p_many_data->pos_vel_tqe[index].pos = pos_int16;
    p_many_data->pos_vel_tqe[index].vel = vel_int16;
    p_many_data->pos_vel_tqe[index].tqe = tqe_int16;
}


/**
 * @brief 位置、速度、加速度模式（梯形控制）
 * @param portx can通道（需要在 motor.c 中修改 port_maping 结构体数组进行映射）
 * @param id 电机 ID
 * @param pos 目标位置，单位可为转（r）、弧度（rad）、或度（°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param vel 目标速度，单位可为转（rps）、弧度（rad/s）、或度（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param acc 目标加速度，单位可为转/秒2（rps2）、弧度/秒2（rad/s2）、或度/秒2（°/s2），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 */
void motor_many_pos_vel_acc(port_t portx, const uint8_t id, const float pos, const float vel, const float acc)
{
    p_many_data_s p_many_data = motor_get_many_pointer(portx);

    const float pos_turns = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float vel_turns = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float acc_turns = conv_to_turns(acc, MOTOR_DATA_TYPE_FLAG);

    const int16_t pos_int16 = pos_float2int(pos_turns, TINT16);
    const int16_t vel_int16 = vel_float2int(vel_turns, TINT16);
    const int16_t acc_int16 = acc_float2int(acc_turns, TINT16);
    const uint16_t index = id - 1;

    if (p_many_data->mode != MODE_POS_VEL_ACC)
    {
        p_many_data->mode = MODE_POS_VEL_ACC;
        for (int i = 0; i < MANY_DATA_BUF_MAX_LEN / sizeof(many_pos_vel_acc_s); i++)
        {
            p_many_data->pos_vel_tqe[i].pos = NAN_INT16;
            p_many_data->pos_vel_tqe[i].vel = 0;
            p_many_data->pos_vel_tqe[i].tqe = 0;
        }
    }

    p_many_data->pos_vel_tqe[index].pos = pos_int16;
    p_many_data->pos_vel_tqe[index].vel = vel_int16;
    p_many_data->pos_vel_tqe[index].tqe = acc_int16;
}


/**
 * @brief 运控模式 (输出力矩 = 位置偏差 * Mkp + 速度偏差 * Mkd + 前馈力矩)
 * @param portx can通道（需要在 motor.c 中修改 port_maping 结构体数组进行映射）
 * @param id 电机 ID
 * @param pos 位置，单位可为转（r）、弧度（rad）、或度（°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param vel 速度，单位可为转（rps）、弧度（rad/s）、或度（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param tqe 力矩，单位牛米（NM），注：需要在 motor.c 文件中修改电机数量和类型，以修正电机力矩
 * @param kp Mkp = kp * 1 (Mkp 表示电机内部 kp)
 * @param kd Mkd = kd * 1 (Mkd 表示电机内部 kd)
 */
void motor_many_pos_vel_tqe_kp_kd(port_t portx, const uint8_t id, const float pos, const float vel, const float tqe, const float kp, const float kd)
{
    p_many_data_s p_many_data = motor_get_many_pointer(portx);
    const motor_type_t model = motor_get_model2(portx, id);

    const float pos_turns = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float vel_turns = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float tqe_float = tqe_adjust(tqe, motor_get_model2(portx, id));

    const int16_t pos_int16 = pos_float2int(pos_turns, TINT16);
    const int16_t vel_int16 = vel_float2int(vel_turns, TINT16);
    const int16_t tqe_int16 = tqe_float2int(tqe_float, TINT16);

    const float kp2 = pid_adjust(kp, model);
    const float kd2 = pid_adjust(kd, model);

    const int16_t kp_int16 = pid_float2int(kp2, TINT16);
    const int16_t kd_int16 = pid_float2int(kd2, TINT16);
    const uint16_t index = id - 1;

    if (p_many_data->mode != MODE_POS_VEL_TQE_KP_KD)
    {
        p_many_data->mode = MODE_POS_VEL_TQE_KP_KD;
        for (int i = 0; i < MANY_DATA_BUF_MAX_LEN / sizeof(many_pos_vel_tqe_kp_kd_s); i++)
        {
            p_many_data->pos_vel_tqe_kp_kd[i].pos = NAN_INT16;
            p_many_data->pos_vel_tqe_kp_kd[i].vel = 0;
            p_many_data->pos_vel_tqe_kp_kd[i].tqe = 0;
            p_many_data->pos_vel_tqe_kp_kd[i].kp = 0;
            p_many_data->pos_vel_tqe_kp_kd[i].kd = 0;
        }
    }

    p_many_data->pos_vel_tqe_kp_kd[index].pos = pos_int16;
    p_many_data->pos_vel_tqe_kp_kd[index].vel = vel_int16;
    p_many_data->pos_vel_tqe_kp_kd[index].tqe = tqe_int16;
    p_many_data->pos_vel_tqe_kp_kd[index].kp = kp_int16;
    p_many_data->pos_vel_tqe_kp_kd[index].kd = kd_int16;
}


/**
 * @brief 真运控模式 (输出力矩 = 位置偏差 * Mkp + 速度偏差 * Mkd + 前馈力矩)
 * @param portx can通道（需要在 motor.c 中修改 port_maping 结构体数组进行映射）
 * @param id 电机 ID
 * @param pos 位置，单位可为转（r）、弧度（rad）、或度（°），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param vel 速度，单位可为转（rps）、弧度（rad/s）、或度（°/s），具体由宏定义 MOTOR_DATA_TYPE_FLAG 决定
 * @param tqe 力矩，单位牛米（NM），注：需要在 motor.c 文件中修改电机数量和类型，以修正电机力矩
 * @param kp Mkp = kp * 1 (Mkp 表示电机内部 kp)
 * @param kd Mkd = kd * 1 (Mkd 表示电机内部 kd)
 */
void motor_many_pos_vel_tqe_kp_kd_2(port_t portx, const uint8_t id, const float pos, const float vel, const float tqe, const float kp, const float kd)
{
    p_many_data_s p_many_data = motor_get_many_pointer(portx);
    const motor_type_t model = motor_get_model2(portx, id);

    const float pos_turns = conv_to_turns(pos, MOTOR_DATA_TYPE_FLAG);
    const float vel_turns = conv_to_turns(vel, MOTOR_DATA_TYPE_FLAG);
    const float tqe_float = tqe_adjust(tqe, motor_get_model2(portx, id));

    const int16_t pos_int16 = pos_float2int(pos_turns, TINT16);
    const int16_t vel_int16 = vel_float2int(vel_turns, TINT16);
    const int16_t tqe_int16 = tqe_float2int(tqe_float, TINT16);

    const float kp2 = pid_adjust(kp, model);
    const float kd2 = pid_adjust(kd, model);

    const int16_t kp_int16 = pid_float2int(kp2, TINT16);
    const int16_t kd_int16 = pid_float2int(kd2, TINT16);
    const uint16_t index = id - 1;

    if (p_many_data->mode != MODE_POS_VEL_TQE_KP_KD2)
    {
        p_many_data->mode = MODE_POS_VEL_TQE_KP_KD2;
        for (int i = 0; i < MANY_DATA_BUF_MAX_LEN / sizeof(many_pos_vel_tqe_kp_kd_s); i++)
        {
            p_many_data->pos_vel_tqe_kp_kd[i].pos = NAN_INT16;
            p_many_data->pos_vel_tqe_kp_kd[i].vel = 0;
            p_many_data->pos_vel_tqe_kp_kd[i].tqe = 0;
            p_many_data->pos_vel_tqe_kp_kd[i].kp = 0;
            p_many_data->pos_vel_tqe_kp_kd[i].kd = 0;
        }
    }

    p_many_data->pos_vel_tqe_kp_kd[index].pos = pos_int16;
    p_many_data->pos_vel_tqe_kp_kd[index].vel = vel_int16;
    p_many_data->pos_vel_tqe_kp_kd[index].tqe = tqe_int16;
    p_many_data->pos_vel_tqe_kp_kd[index].kp = kp_int16;
    p_many_data->pos_vel_tqe_kp_kd[index].kd = kd_int16;
}


static uint8_t get_data_max(uint8_t mode)
{
    switch (mode)
    {
    case (MODE_POS_VEL_KP_KD):
        return 56;
    default:
        return 60;
    }
}


static uint8_t get_motor_data_len(uint16_t size)
{
    uint8_t data_len = 0;

    size += 2;
    if (size <= 8)
    {
        data_len = size;
    }
    else if (size <= 12)
    {
        data_len = 12;
    }
    else if (size <= 16)
    {
        data_len = 16;
    }
    else if (size <= 20)
    {
        data_len = 20;
    }
    else if (size <= 24)
    {
        data_len = 24;
    }
    else if (size <= 32)
    {
        data_len = 32;
    }
    else if (size <= 48)
    {
        data_len = 48;
    }
    else
    {
        data_len = 64;
    }

    return data_len;
}


static uint8_t get_mode_data_len(uint8_t mode)
{
    switch(mode)
    {
    case MODE_POSITION:
    case MODE_VELOCITY:
    case MODE_TORQUE:
    case MODE_VOLTAGE:
    case MODE_CURRENT:
    case MODE_TIME_OUT:
        return 2;
    case MODE_POS_VEL_TQE:
    case MODE_POS_VEL_ACC:
        return 6;
    case MODE_POS_VEL_KP_KD:
        return 8;
    case MODE_POS_VEL_TQE_KP_KD:
    case MODE_POS_VEL_TQE_KP_KD2:
        return 10;
    }

    return 0;
}


/**
 * @brief 一拖多 发送
 * @param portx can通道（需要在 motor.c 中修改 port_maping 结构体数组进行映射）
 * @param request_type 决定电机返回帧包含的信息
 */
void motor_many_send(port_t portx, many_request_type_t request_type)
{
    p_many_data_s p_many_data = motor_get_many_pointer(portx);
    FDCAN_HandleTypeDef *fdcanHandle = motor_get_fdcan_pointer(portx);

    uint8_t *p_get_cmd = (uint8_t *)&many_get_cmd[request_type][0];
    static uint8_t cmd[64] = {0};
    uint8_t id = p_many_data->mode;

    uint16_t remaining_len = get_mode_data_len(id) * MANY_MOTOR_SIZE;
    uint8_t data_len_max = get_data_max(id);
    uint8_t *data = p_many_data->data;

    while (remaining_len > 0)
    {
        const uint8_t current_data_len = (remaining_len > data_len_max) ? data_len_max : remaining_len;
        uint8_t cmd_len = get_motor_data_len(current_data_len);

        my_memcpy(cmd, data, current_data_len);
        data += current_data_len;
        remaining_len -= current_data_len;
        my_memcpy(cmd + cmd_len - 2, p_get_cmd, 2);
        fdcan_send(fdcanHandle, 0x8000 | id, cmd, cmd_len);
        ++id;
    }
}

