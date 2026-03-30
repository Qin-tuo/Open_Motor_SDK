#ifndef _MOTOR_H
#define _MOTOR_H




#define  MOTOR_PORT_NUM  2  // 使用 CAN 通道数量  
#define  MOTOR_MAX_NUM   2  // 单个 CAN 通道所连接的最大电机数量



#include "fdcan.h"
#include "my_fdcan.h"
#include "convert.h"
#include "livelybot_fdcan.h"


typedef enum
{
    PNULL = 0,
    PORT1,
    PORT2,
    PORT3,
} port_t;


typedef struct
{
    uint8_t major : 4;
    uint8_t minor : 8;
    uint8_t patch : 4;
} version_s, *p_version_s;


typedef struct
{
    float position;  // 位置
    float velocity;  // 速度
    float torque;    // 力矩
    uint8_t mode;    // 模式
    int8_t  temp;    // 温度（单位：摄氏度，分辨率：1度）
    uint8_t fault;   // 错误码，可在《寄存器功能、电机运行模式、报错代码、一托多模式说明.xlsx》中查询
    uint8_t ack;     // 应答，用于电机设置相关的应答
    const motor_type_t model;  // 电机型号（这个参数由用户自定义，此程序根据这个变量进行电机力矩修正）
    version_s version;  // 电机固件版本号
} motor_state_s, *p_motor_state_s;  // 这个结构体会定义成结构体数组，其中数组下标 +1 即为电机 ID


typedef struct
{
    const port_t port;
    FDCAN_HandleTypeDef *fdcan;
    const p_motor_state_s state;
} port_mapping_s, *p_port_mapping_s;


typedef enum
{
    MANY_GET_MODE_FLAUT_POS_VEL_TQE = 0,  // 模式、错误码、位置、速度、力矩 (电机从 v4.2.3 开始支持)
    MANY_GET_TEMP_FLAUT_POS_VEL_TQE,      // 温度、错误码、位置、速度、力矩（电机从 v4.5.0 开始支持）
    MANY_GET_POS_VEL_TQE,                 // 位置、速度、力矩 （所有版本都支持）
    MANY_GET_MAX_NUM,
} many_request_type_t;



void motor_print_state(void);
void motor_print_version(void);

p_motor_state_s motor_get_state(port_t portx, uint8_t id);

FDCAN_HandleTypeDef *motor_get_fdcan_pointer(port_t portx);
p_motor_state_s motor_get_state_pointer1(FDCAN_HandleTypeDef *fdcanHandle);
p_motor_state_s motor_get_state_pointer2(port_t portx);
motor_type_t motor_get_model2(port_t portx, uint8_t id);

void motor_process_state_all(void);




#define  MOTOR_SDK_VERSION   "3.1.2"

#endif
