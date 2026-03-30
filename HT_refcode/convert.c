#include "convert.h"

#ifdef __MICROLIB  // 有无启用MicroLIB库
#include <string.h>
#endif



const motor_tqe_adj_t motor_tqe_adj[MOTOR_TYPE_COUNT] =
{
    [MNULL] = {0.0f, 0.0f},
    [M3536_32] = {0.458100f, 0.0f},
    [M4438_30] = {0.525600f, 0.0f},
    [M4438_32] = {0.558400f, 0.0f},
    [M4538_19] = {0.445000f, 0.0f},
    [M5043_20] = {0.966000f, 0.0f},
    [M5046_20] = {0.528000f, 0.0f},
    [M5047_09] = {0.533000f, 0.0f},
    [M5047_36] = {0.803000f, 0.0f},
    [M6056_36] = {0.677000f, 0.0f},
    [M7256_35] = {0.677000f, 0.0f},  
    [M60SG_35] = {0.794200f, 0.0f},  
    [M60BM_35] = {0.794200f, 0.0f}, 
    [MGENERAL] = {0.5f, 0.0f},
    [MNONE] = {1.0f, 0.0f},
};



static float data_limit(const float in_data, const float max, const float min)
{
    if (in_data >= max)
    {
        return max;
    }
    else if (in_data <= min)
    {
        return -min;
    }

    return in_data;
}


static float data_float2int(const float in_data, const data_type_t type, const float rint8, const float rint16, const float rint32)
{
    if (isnan(in_data))
    {
        switch (type)
        {
        // case TINT8:
        //     return NAN_INT8;
        case TINT16:
            return NAN_INT16;
        case TINT32:
            return NAN_INT32;
        case TFLOAT:
            return NAN_FLOAT;
        default:
            MOTOR_ERR();
            return 0;
        }
    }

    switch (type)
    {
    // case TINT8:
    //     return data_limit(in_data * rint8, 127.0f, -128.0f);
    case TINT16:
        return data_limit(in_data * rint16, 32767.0f, -32768.0f);
    case TINT32:
        return data_limit(in_data * rint32, 2147483647.0f, -2147483648.0f);
    case TFLOAT:
        return in_data;
    default:
        MOTOR_ERR();
        return 0;
    }
}


static float data_int2float(const float in_data, const data_type_t type, const float rint8, const float rint16, const float rint32)
{
    switch (type)
    {
    // case (TINT8):
    //     return in_data / rint8;
    case (TINT16):
        return in_data / rint16;
    case (TINT32):
        return in_data / rint32;
    case (TFLOAT):
        return in_data;
    default:
        MOTOR_ERR();
        return 0;
    }
}


/**
 * @brief 将弧度或角度转化成圈数
 * @param in_data 弧度值或角度值
 * @param type in_data的类型
 * @return 圈数值
 */
float conv_to_turns(const float in_data, const pos_vel_type_t type)
{
    if (isnan(in_data))
    {
        return NAN_FLOAT;
    }

    switch (type)
    {
    case RADIAN_2PI:
        return in_data / MY_2PI;
    case ANGLE_360:
        return in_data / 360.0f;
    case TURNS:
        return in_data;
    default:
        MOTOR_ERR();
        return 0.0f;
    }
}


/**
 * @brief 将圈数转化成弧度或角度
 * @param in_data 圈数值
 * @param type 要转换成的类型
 * @return 弧度值或角度值
 */
float conv_from_turns(const float in_data, const pos_vel_type_t type)
{
    switch (type)
    {
    case RADIAN_2PI:
        return in_data * MY_2PI;
    case ANGLE_360:
        return in_data * 360.0f;
    case TURNS:
        return in_data;
    default:
        MOTOR_ERR();
        return 0.0f;
    }
}


/**
 * @brief 力矩补偿，写力矩时用
 * @param in_data 真实力矩数据
 * @param motor_type 电机型号
 * @return 给定力矩数据
 */
float tqe_adjust(const float in_data, const motor_type_t motor_type)
{
    if (isnan(in_data))
    {
        return NAN_FLOAT;
    }

    if (motor_type >= MOTOR_TYPE_COUNT)
    {
        MOTOR_ERR();
        return 0;
    }

    const motor_tqe_adj_t *p_motor_tqe_adj = &motor_tqe_adj[motor_type];
    return ((in_data - p_motor_tqe_adj->d) / p_motor_tqe_adj->k);
}


/**
 * @brief 力矩补偿，读力矩时用
 * @param in_data 读力矩数据
 * @param motor_type 电机型号
 * @return 真实力矩数据
 */
float tqe_restore(const float in_data, const motor_type_t motor_type)
{
    if (isnan(in_data))
    {
        return NAN_FLOAT;
    }

    if (motor_type >= MOTOR_TYPE_COUNT)
    {
        MOTOR_ERR();
        return 0;
    }

    const motor_tqe_adj_t *p_motor_tqe_adj = &motor_tqe_adj[motor_type];
    return (in_data * p_motor_tqe_adj->k + p_motor_tqe_adj->d);
}


/**
 * @brief PID 补偿
 * @param id_data PID 数据
 * @param motor_type 电机型号
 * @return 补偿的 PID 数据
 */
float pid_adjust(const float in_data, const motor_type_t motor_type)
{
    if (isnan(in_data))
    {
        return NAN_FLOAT;
    }

    if (motor_type >= MOTOR_TYPE_COUNT)
    {
        MOTOR_ERR();
        return 0;
    }

    const motor_tqe_adj_t *p_motor_tqe_adj = &motor_tqe_adj[motor_type];
    return (in_data / p_motor_tqe_adj->k);
}


float cur_float2int(const float in_data, const data_type_t type)
{
    return data_float2int(in_data, type, 1.0f, 10.0f, 1000.0f);
}


float cur_int2float(const float in_data, const data_type_t type)
{
    return data_int2float(in_data, type, 1.0f, 10.0f, 1000.0f);
}


float vol_float2int(const float in_data, const data_type_t type)
{
    return data_float2int(in_data, type, 2.0f, 10.0f, 1000.0f);
}


float vol_int2float(const float in_data, const data_type_t type)
{
    return data_int2float(in_data, type, 2.0f, 10.0f, 1000.0f);
}


float pos_float2int(const float in_data, const data_type_t type)
{
    return data_float2int(in_data, type, 100.0f, 10000.0f, 100000.0f);
}


float pos_int2float(const float in_data, const data_type_t type)
{
    return data_int2float(in_data, type, 100.0f, 10000.0f, 100000.0f);
}


float vel_float2int(const float in_data, const data_type_t type)
{
    return data_float2int(in_data, type, 100.0f, 4000.0f, 100000.0f);
}


float vel_int2float(const float in_data, const data_type_t type)
{
    return data_int2float(in_data, type, 100.0f, 4000.0f, 100000.0f);
}



float tqe_float2int(const float in_data, const data_type_t type)
{
    return data_float2int(in_data, type, 2.0f, 100.0f, 1000.0f);
}


float tqe_int2float(const float in_data, const data_type_t type)
{
    return data_int2float(in_data, type, 2.0f, 100.0f, 1000.0f);
}


float acc_float2int(const float in_data, const data_type_t type)
{
    return data_float2int(in_data, type, 20.0f, 1000.0f, 100000.0f);
}


float acc_int2float(const float in_data, const data_type_t type)
{
    return data_int2float(in_data, type, 20.0f, 1000.0f, 100000.0f);
}


float pid_float2int(const float in_data, const data_type_t type)
{
    return data_float2int(in_data, type, 1.0f, 10.0f, 1000.0f);
}


float pid_int2float(const float in_data, const data_type_t type)
{
    return data_int2float(in_data, type, 1.0f, 10.0f, 1000.0f);
}




void my_memcpy(void *p1, const void *p2, const int16_t len)
{
    if (len <= 0)
    {
        MOTOR_ERR();
        return;
    }

#ifdef __MICROLIB  // 有无启用MicroLIB库
    memcpy(p1, p2, len);
#else
    uint8_t *p11 = (uint8_t *)p1;
    const uint8_t *p22 = (uint8_t *)p2;
    for (int i = 0; i < len; i++)
    {
        p11[i] = p22[i];
    }
#endif
}

