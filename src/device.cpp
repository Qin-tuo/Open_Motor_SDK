#include "device.hpp"

// =========================================================
//  物理常数与系数定义 (保持原样)
// =========================================================
static const float RAD_TO_DEG = 57.2957795f;
static const float DEG_TO_RAD = 0.017453293f;

// 接收系数
static const float P_RIOT = 6.28318530718f / 65536.0f;
static const float V_ROIT = 57.29578f; 
static const float IQ_ROIT = 0.008056640625f; 

// 发送系数
static const float LK_CURRENT_SEND_FACTOR = 100.0f;
static const float LK_SPEED_SEND_FACTOR = 57.29578f * 100.0f;
static const float LK_POS_SEND_FACTOR = 57.29578f * 100.0f;

// =========================================================
//  DeviceX 实现
// =========================================================
bool DeviceX::Init(int32_t handle, const std::string& dev_name, int dev_idx, 
                      std::vector<Motor_CAN_Struct>* data_ptr, TopoMapper* mapper_ptr) {
    
    this->device_handle = handle;
    this->device_global_index = dev_idx;
    this->p_motors_data = data_ptr;
    this->p_mapper = mapper_ptr;
    
    is_running = true;
    rx_thread = std::thread(&DeviceX::ReceiveLoop, this);
    return true;
}

// =========================================================
//  DeviceX 公共接口 (Dispatcher / 分发层)
// =========================================================

void DeviceX::EnableMotor(int& motor_index) {
    if (!p_motors_data || motor_index < 0 || motor_index >= p_motors_data->size()) return;
    
    int type = (*p_motors_data)[motor_index].info.api_type;
    
    if (type == 1) {
        EnableMotor_Type1(motor_index);
    } else if (type == 2) {
        EnableMotor_Type2(motor_index);
    }
}

void DeviceX::DisableMotor(int& motor_index) {
    if (!p_motors_data || motor_index < 0 || motor_index >= p_motors_data->size()) return;
    
    int type = (*p_motors_data)[motor_index].info.api_type;
    
    if (type == 1) {
        DisableMotor_Type1(motor_index);
    } else if (type == 2) {
        DisableMotor_Type2(motor_index);
    }
}

void DeviceX::ClearError(int& motor_index) {
    if (!p_motors_data || motor_index < 0 || motor_index >= p_motors_data->size()) return;
    
    int type = (*p_motors_data)[motor_index].info.api_type;
    
    if (type == 1) {
        ClearError_Type1(motor_index);
    } else if (type == 2) {
        ClearError_Type2(motor_index);
    }
}

void DeviceX::SetMode(int& motor_index, int mode) {
    if (!p_motors_data || motor_index < 0 || motor_index >= p_motors_data->size()) return;
    
    int type = (*p_motors_data)[motor_index].info.api_type;
    
    if (type == 1) {
        SetMode_Type1(motor_index, mode);
    } else if (type == 2) {
        SetMode_Type2(motor_index, mode);
    }
}

void DeviceX::SendCommand(int& motor_index) {
    if (!p_motors_data || motor_index < 0 || motor_index >= p_motors_data->size()) {
        std::cerr << "[ERROR] SendCommand Failed! Index Out of Range or Null Ptr." << std::endl;
        return;
    }

    int type = (*p_motors_data)[motor_index].info.api_type;

    if (type == 1) {
        SendCommand_Type1(motor_index);
    } else if (type == 2) {
        SendCommand_Type2(motor_index);
    }
}

void DeviceX::QueryPos(int& motor_index) {
    if (!p_motors_data || motor_index < 0 || motor_index >= p_motors_data->size()) return;

    int type = (*p_motors_data)[motor_index].info.api_type;

    if (type == 1) {
        QueryPos_Type1(motor_index);
    } else if (type == 2) {
        QueryPos_Type2(motor_index);
    }
}

// =========================================================
//  Type 1 (灵足/LimX) 私有逻辑实现
//  (完全复制自原 DeviceX)
// =========================================================

void DeviceX::EnableMotor_Type1(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    const Motor_CAN_Struct& motor = (*p_motors_data)[motor_index];

    int channel = motor.info.chan;
    int id = motor.info.canid; 

    std::cout << "========================================" << std::endl;
    std::cout << "[Motor Enable Type1] Sending Enable Command..." << std::endl;
    std::cout << " - Motor Name   : " << info.name << std::endl;
    std::cout << " - Motor Number : " << info.num << std::endl;
    std::cout << " - Motor Index : " << motor_index << std::endl;
    std::cout << " - Device Index  : " << info.device_index << std::endl;
    std::cout << " - Using Device Index : " << this->device_global_index << std::endl;
    std::cout << " - CAN Channel   : " << channel << std::endl;
    std::cout << " - CAN ID (Hex) : 0x" << std::hex << (int)id << std::dec << std::endl;
    std::cout << "========================================" << std::endl;
    
    uint8_t data[8] = {0};
    sendRawFrame(info.chan, 3, 0xFD, info.canid, data);
}

void DeviceX::DisableMotor_Type1(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    uint8_t data[8] = {0}; // data[0]=0 仅停止
    sendRawFrame(info.chan, 4, 0xFD, info.canid, data);
    std::cout << "[LK] Disabled Motor ID: " << motor_index << std::endl;

}

void DeviceX::ClearError_Type1(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    uint8_t data[8] = {0};
    data[0] = 1; // Byte[0]=1 表示清除故障
    sendRawFrame(info.chan, 4, 0xFD, info.canid, data);
    std::cout << "[Motor] Clear Error Sent to: " << info.name << std::endl;
}

void DeviceX::SetMode_Type1(int& motor_index, int mode) {
    const auto& info = (*p_motors_data)[motor_index].info;
    
    uint8_t data[8] = {0};
    uint16_t index = 0x7005; // run_mode 寄存器地址
    uint8_t mode_val = (uint8_t)mode;
    
    memcpy(&data[0], &index, 2);
    memcpy(&data[4], &mode_val, 1);
    
    sendRawFrame(info.chan, 18, 0xFD, info.canid, data);
    std::cout << "[Motor] Mode Change to " << mode << " for: " << info.name << std::endl;
}

void DeviceX::SendCommand_Type1(int& motor_index) {
    const Motor_CAN_Struct& motor = (*p_motors_data)[motor_index];
    const auto& cmd = motor.send;
    const auto& info = motor.info;
    uint8_t data[8] = {0};
    
    // 1. ID 中的 DataArea 承载扭矩
    uint16_t t_int = float_to_uint(cmd.torque, info.t_min, info.t_max, 16);
    // 2. Data 域承载 P, V, Kp, Kd
    int p_int  = float_to_uint(cmd.position, info.p_min, info.p_max, 16);
    int v_int  = float_to_uint(cmd.speed, info.v_min, info.v_max, 16);
    int kp_int = float_to_uint(cmd.kp, info.kp_min, info.kp_max, 16);
    int kd_int = float_to_uint(cmd.kd, info.kd_min, info.kd_max, 16);

    // 高字节在前
    data[0] = p_int >> 8;  data[1] = p_int & 0xFF;
    data[2] = v_int >> 8;  data[3] = v_int & 0xFF;
    data[4] = kp_int >> 8; data[5] = kp_int & 0xFF;
    data[6] = kd_int >> 8; data[7] = kd_int & 0xFF;
    
    sendRawFrame(info.chan, 1, t_int, info.canid, data);
}
void DeviceX::QueryPos_Type1(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    
    uint8_t data[8] = {0};
    
    // 参数 Index: 0x7019 (mechPos 负载端计圈机械角度)
    // 协议要求：低字节在前
    data[0] = 0x19;
    data[1] = 0x70;
    // data[2]~data[7] 保持为 0
    
    // 发送指令：
    // Type: 17 (0x11) 单个参数读取
    // Data Area: 0xFD (主机 ID)
    // Motor ID: info.canid
    sendRawFrame(info.chan, 17, 0xFD, info.canid, data);
    
    // std::cout << "[Motor] Query Pos Sent to: " << info.name << std::endl;
}
void DeviceX::sendRawFrame(uint8_t chan, uint32_t type, uint16_t data_area, uint8_t motor_id, uint8_t* data) {
    FrameInfo txInfo;
    txInfo.canID = ((type & 0x1F) << 24) | ((data_area & 0xFFFF) << 8) | (motor_id & 0xFF);
    txInfo.frameType = EXTENDED;
    txInfo.dataLength = 8;
    sendUSBCAN(this->device_handle, chan, &txInfo, data);
}

// =========================================================
//  Type 2 (LK/宇树) 私有逻辑实现
//  (完全复制自原 Device_Type2, 仅改为私有成员函数)
// =========================================================

void DeviceX::EnableMotor_Type2(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;
    const Motor_CAN_Struct& motor = (*p_motors_data)[motor_index];
    int channel = motor.info.chan;
    int id = motor.info.canid; 


    std::cout << "========================================" << std::endl;
    std::cout << "[Motor Enable Type1] Sending Enable Command..." << std::endl;
    std::cout << " - Motor Name   : " << info.name << std::endl;
    std::cout << " - Motor Number : " << info.num << std::endl;
    std::cout << " - Motor Index : " << motor_index << std::endl;
    std::cout << " - Device Index  : " << info.device_index << std::endl;
    std::cout << " - Using Device Index : " << this->device_global_index << std::endl;
    std::cout << " - CAN Channel   : " << channel << std::endl;
    std::cout << " - CAN ID (Hex) : 0x" << std::hex << (int)id << std::dec << std::endl;
    std::cout << "========================================" << std::endl;
    

    FrameInfo txMsg = {0}; 
    txMsg.frameType = STANDARD;
    txMsg.dataLength = 8;
    txMsg.canID = 0x140 + motor.info.canid;

    uint8_t data[8] = {0};
    data[0] = 0x88; // LK 使能指令

    if (sendUSBCAN(this->device_handle, (uint8_t)motor.info.chan, &txMsg, data) == 1) {
        std::cout << "[LK] Enabled Motor ID: " << motor_index << std::endl;
    } else {
        std::cerr << "[LK] Enable Failed ID: " << motor_index << std::endl;
    }
}

void DeviceX::DisableMotor_Type2(int& motor_index) {
    const Motor_CAN_Struct& motor = (*p_motors_data)[motor_index];

    FrameInfo txMsg = {0};
    txMsg.frameType = STANDARD;
    txMsg.dataLength = 8;
    txMsg.canID = 0x140 + motor.info.canid;

    uint8_t data[8] = {0};
    data[0] = 0x80; 

    sendUSBCAN(this->device_handle, (uint8_t)motor.info.chan, &txMsg, data);
    std::cout << "[LK] Disabled Motor ID: " << motor_index << std::endl;
}

void DeviceX::ClearError_Type2(int& motor_index) {
    const Motor_CAN_Struct& motor = (*p_motors_data)[motor_index];
    std::cout << "[INFO][LK] ClearError (Reset) -> Motor: " << motor.info.name << std::endl;
    
    // 直接调用 Type2 的 Disable 逻辑
    DisableMotor_Type2(motor_index);
}

void DeviceX::SetMode_Type2(int& motor_index, int mode) {
    // Type 2 仅更新本地 struct，不发送指令
    (*p_motors_data)[motor_index].send.mode = (uint8_t)mode;
}

void DeviceX::SendCommand_Type2(int& motor_index) {
    const Motor_CAN_Struct &motor = (*p_motors_data)[motor_index];
    int id = motor.info.canid;
    uint8_t current_mode = motor.send.mode;

    FrameInfo txMsg = {};
    txMsg.frameType = STANDARD; 
    txMsg.dataLength = 8;
    txMsg.canID = 0x140 + id; 

    uint8_t data[8] = {0};

    // --- Mode 1: MIT / Torque Control ---
    if (current_mode == 1) {
        float current_p = motor.recv.current_position_f.load();
        float current_v = motor.recv.current_speed_f.load();
        
        float kp = motor.send.kp;
        float kd = motor.send.kd;
        float target_p = motor.send.position;
        float target_v = motor.send.speed;
        float t_ff = motor.send.torque;

        float iq_f = kp * (target_p - current_p) + kd * (target_v - current_v) + t_ff;
        int16_t iqControl = (int16_t)(iq_f * LK_CURRENT_SEND_FACTOR);

        if (iqControl > 2048) iqControl = 2048;
        if (iqControl < -2048) iqControl = -2048;

        data[0] = 0xA1;
        memcpy(&data[4], &iqControl, 2); 
    }
    // --- Mode 2: Position Control ---
    else if (current_mode == 2) {
        float target_rad = motor.send.position;
        int32_t angleControl = (int32_t)(target_rad * LK_POS_SEND_FACTOR);

        data[0] = 0xA3;
        memcpy(&data[4], &angleControl, 4);
    }
    // --- Mode 3: Speed Control ---
    else if (current_mode == 3) {
        float target_vel_rad = motor.send.speed;
        int32_t speedControl = (int32_t)(target_vel_rad * LK_SPEED_SEND_FACTOR);
        int16_t iqLimit = 2000; 

        data[0] = 0xA2;
        memcpy(&data[2], &iqLimit, 2);
        memcpy(&data[4], &speedControl, 4);
    }
    // --- Default: Read Status ---
    else {
        data[0] = 0x9C;
    }

    sendUSBCAN(this->device_handle, (uint8_t)motor.info.chan, &txMsg, data);
}

// =========================================================
//  Type 2 (LK/宇树/灵康) 私有逻辑实现 - 补全
// =========================================================

void DeviceX::QueryPos_Type2(int& motor_index) {
    const auto& info = (*p_motors_data)[motor_index].info;

    // 构造 FrameInfo
    FrameInfo txMsg = {0};
    txMsg.frameType = STANDARD; // 标准帧
    txMsg.dataLength = 8;
    // 根据文档：命令报文标识符：0x140 + ID(1~32)
    txMsg.canID = 0x140 + info.canid;

    uint8_t data[8] = {0};
    
    // 根据文档 Section 3: 读取电机状态 2 命令
    // Data[0] = 0x9C
    // Data[1]~Data[7] = 0x00
    data[0] = 0x9C; 

    // 发送指令
    // 注意：Type2 在您之前的代码中是直接调用 sendUSBCAN，未经过 sendRawFrame 封装
    sendUSBCAN(this->device_handle, (uint8_t)info.chan, &txMsg, data);

    // std::cout << "[LK] Query Pos (0x9C) Sent to ID: " << info.canid << std::endl;
}

// =========================================================
//  Unified Receive Loop (融合版)
// =========================================================

void DeviceX::ReceiveLoop() {
    FrameInfo rxInfo;
    uint8_t rxData[8];
    uint8_t rxChannel = 0;

    while(is_running) {
        // 读取 CAN 帧
        int32_t num = readUSBCAN(this->device_handle, &rxChannel, &rxInfo, rxData, 5000);
        if (num < 0) continue;

        int parsed_motor_id = -1;

        // --- Step 1: 根据帧类型解析 ID ---
        if (rxInfo.frameType == 1) { // Extended Frame -> Type 1
            uint32_t canID = rxInfo.canID;
            parsed_motor_id = (canID >> 8) & 0xFF; 
        }
        else { // Standard Frame -> Type 2
            uint32_t canID = rxInfo.canID;
            if (canID >= 0x140 && canID < 0x160) {
                parsed_motor_id = canID - 0x140;
            } else if (canID >= 0x180 && canID < 0x1A0) {
                parsed_motor_id = canID - 0x180;
            }
        }

        // --- Step 2: 查找 Mapper ---
        if (parsed_motor_id == -1) continue;
        int g_idx = p_mapper->get_id({(uint)this->device_global_index, (uint)rxChannel, (uint)parsed_motor_id});

        // --- Step 3: 根据 API Type 分发解析 ---
        if (g_idx != -1 && g_idx < (int)p_motors_data->size()) {
            Motor_CAN_Struct& motor = (*p_motors_data)[g_idx];

            // >>> TYPE 1 Logic (LimX) >>>
            if (motor.info.api_type == 1) { 
                uint32_t canID = rxInfo.canID;
                uint8_t type_field = (canID >> 24) & 0x1F;

                if (type_field == 2) { 
                    uint16_t p_int = (rxData[0] << 8) | rxData[1];
                    uint16_t v_int = (rxData[2] << 8) | rxData[3];
                    uint16_t t_int = (rxData[4] << 8) | rxData[5];
                    uint16_t temp_int = (rxData[6] << 8) | rxData[7];

                    motor.recv.current_position_f.store(uint_to_float(p_int, motor.info.p_min, motor.info.p_max, 16));
                    motor.recv.current_speed_f.store(uint_to_float(v_int, motor.info.v_min, motor.info.v_max, 16));
                    motor.recv.current_torque_f.store(uint_to_float(t_int, motor.info.t_min, motor.info.t_max, 16));
                    motor.recv.current_temp_f.store((float)temp_int / 10.0f);

                    motor.recv.fault_message = (canID >> 16) & 0x3F;
                    motor.recv.mode = (canID >> 22) & 0x03;
                }
                else if (type_field == 17) {
                    // 校验 Index 是否为我们查询的 0x7019 (低字节在前: Data[0]=0x19, Data[1]=0x70)
                    if (rxData[0] == 0x19 && rxData[1] == 0x70) {
                        float temp_val;
                        // 数据在 Byte4~7，低字节在前 (Little Endian)
                        memcpy(&temp_val, &rxData[4], 4);
                        
                        // 更新位置 (0x7019 是 mechPos，单位 rad)
                        motor.recv.current_position_f.store(temp_val);
                        
                        // 由于 Type 17 不包含速度和力矩，这里只更新位置
                    }
                }
                else if (type_field == 21) { 
                    uint32_t fault_word;
                    memcpy(&fault_word, &rxData[0], 4);
                    motor.recv.fault_message = (uint8_t)fault_word;
                }
            }
            // >>> TYPE 2 Logic (LK) >>>
            else if (motor.info.api_type == 2) {
                uint8_t status = rxData[0];
                if (status == 0x9C || status == 0xA1 || status == 0xA2 || status == 0xA3) {
                    int8_t temp = (int8_t)rxData[1];
                    int16_t iq_raw = (int16_t)(rxData[2] | (rxData[3] << 8));
                    int16_t spd_raw = (int16_t)(rxData[4] | (rxData[5] << 8));
                    uint16_t pos_raw = (uint16_t)(rxData[6] | (rxData[7] << 8));

                    motor.recv.current_temp_f.store((float)temp);
                    motor.recv.current_iq_f.store((float)iq_raw * IQ_ROIT); 
                    motor.recv.current_speed_f.store((float)spd_raw / V_ROIT); 
                    motor.recv.current_position_f.store((float)pos_raw * P_RIOT);

                    motor.recv.motor_id = parsed_motor_id;
                    motor.recv.mode = status; 
                    motor.recv.fault_message = 0; 
                }
            }
        }
    }
}

