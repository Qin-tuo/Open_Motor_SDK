#ifndef _CONVERT_H
#define _CONVERT_H


#include "main.h"
#include "math.h"
#include "led.h"


#define  MOTOR_DATA_TYPE_FLAG  TURNS
#define  BROADCAST_ID  0x7F


#ifdef  LED_ERR_FLAG  // 这个宏定义在 led.h 中
#define  MOTOR_ERR    led_toggle_err  // 所有 led 闪烁
#else
static inline void MOTOR_ERR(void) {}
#endif


#define MY_2PI (6.28318530717f)
#define MY_PI  (3.14159265358f)


/* 各个数据类型的无限制 */
#define  NAN_FLOAT  NAN
#define  NAN_INT32  0x80000000
#define  NAN_INT16  0x8000
#define  NAN_INT8   0x80


typedef enum
{
    RADIAN_2PI = 0,  // 弧度制
    ANGLE_360,       // 角度制
    TURNS,           // 圈数
} pos_vel_type_t;


typedef enum  // 数据类型
{
    TFLOAT = 0,
    TINT32,
    TINT16,
    // TINT8,
} data_type_t;


typedef enum __attribute__((packed)) // 电机型号
{
    MNULL = 0,
    M3536_32,
    // M4438_08,
    // M4438_16,
    M4438_30,
    M4438_32,
    M4538_19,
    M5043_20,
    // M5043_35,
    M5046_20,
    M5047_09,
    // M5047_19,
    // M5047_20,
    // M5047_30,
    M5047_36,
    // M6056_08,
    M6056_36,
    // M6057_36,
    // M7136_07,
    // M7233_08,
    M7256_35,
    M60SG_35,
    M60BM_35,
    MGENERAL,  // 无修正
    MNONE,
    MOTOR_TYPE_COUNT,  // 电机类型数量
} __attribute__((packed)) motor_type_t;


typedef struct
{
    float k;
    float d;
} motor_tqe_adj_t, *p_motor_tqe_adj_t;




/* 控制电机用 */
float conv_to_turns(const float in_data, const pos_vel_type_t type);
float tqe_adjust(const float in_data, const motor_type_t motor_type);
float pid_adjust(const float in_data, const motor_type_t motor_type);
float cur_float2int(const float in_data, const data_type_t type);
float vol_float2int(const float in_data, const data_type_t type);
float pos_float2int(const float in_data, const data_type_t type);
float vel_float2int(const float in_data, const data_type_t type);
float tqe_float2int(const float in_data, const data_type_t type);
float acc_float2int(const float in_data, const data_type_t type);
float pid_float2int(const float in_data, const data_type_t type);


/* 读取电机用 */
float conv_from_turns(const float in_data, const pos_vel_type_t type);
float tqe_restore(const float in_data, const motor_type_t motor_type);
float cur_int2float(const float in_data, const data_type_t type);
float vol_int2float(const float in_data, const data_type_t type);
float pos_int2float(const float in_data, const data_type_t type);
float vel_int2float(const float in_data, const data_type_t type);
float tqe_int2float(const float in_data, const data_type_t type);
float acc_int2float(const float in_data, const data_type_t type);
float pid_int2float(const float in_data, const data_type_t type);


/* 数据搬运 */
void my_memcpy(void *p1, const void *p2, const int16_t len);


#endif
