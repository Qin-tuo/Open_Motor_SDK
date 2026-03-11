#pragma once
#include "types.hpp"
#include "mapper.hpp"
#include <thread>
#include <vector>
#include <atomic>
#include <string>
#include <iostream>
#include <cstring> 


// --- DeviceX (Unified Device) ---
// 现在它是唯一的设备类，内部根�?api_type 区分逻辑
class DeviceX{
protected:
    int socket_fd = -1;
    std::string iface_name;
    int device_global_index = -1;
    std::vector<Motor_CAN_Struct>* p_motors_data; 
    TopoMapper* p_mapper;
    std::atomic<bool> is_running{false};
    std::thread rx_thread;

public:

    ~DeviceX();
    // 公共接口：包�?api_type 判断分发逻辑
    bool Init(const std::string& iface, int dev_idx, 
                      std::vector<Motor_CAN_Struct>* data_ptr, TopoMapper* mapper_ptr);
    void SendCommand(int& motor_index) ;
    void EnableMotor(int& motor_index) ;
    void DisableMotor(int& motor_index) ;
    void ClearError(int& motor_index) ;
    void SetZero(int& motor_index);
    void SetMode(int& motor_index, int mode) ;
    void QueryPos(int& motor_index);
    // 接收线程：融合逻辑
    void ReceiveLoop() ;

private:
    // 辅助工具
    bool openSocket(const std::string& iface);
    void sendExtendedFrame(uint32_t type, uint16_t data_area, uint8_t motor_id, const uint8_t* data);
    void sendStandardFrame(uint32_t can_id, const uint8_t* data, uint8_t dlc = 8);
    void sendRawFrame(uint8_t chan, uint32_t type, uint16_t data_area, uint8_t motor_id, uint8_t* data);

    // --- Type 1 (灵足/LimX) 原始逻辑封装 ---
    void EnableMotor_Type1(int& motor_index);
    void DisableMotor_Type1(int& motor_index);
    void ClearError_Type1(int& motor_index);
    void SetZero_Type1(int& motor_index);
    void SetMode_Type1(int& motor_index, int mode);
    void SendCommand_Type1(int& motor_index);
    void QueryPos_Type1(int& motor_index);
    // --- Type 2 (LK/宇树早期) 原始逻辑封装 ---
    void EnableMotor_Type2(int& motor_index);
    void DisableMotor_Type2(int& motor_index);
    void ClearError_Type2(int& motor_index);
    void SetZero_Type2(int& motor_index);
    //  * mode: 1-运控(软闭�?, 2-位置(硬闭�?, 3-速度(硬闭�? 外部做了适配统一�?
    void SetMode_Type2(int& motor_index, int mode);
    void SendCommand_Type2(int& motor_index);
    void QueryPos_Type2(int& motor_index);

    // --- Type 3 (达妙/DM) 私有逻辑封装 ---
    void EnableMotor_Type3(int& motor_index);
    void DisableMotor_Type3(int& motor_index);
    void ClearError_Type3(int& motor_index);
    void SetZero_Type3(int& motor_index);
    //  * mode: 0-MIT, 1-位置速度(PV), 2-速度, 3-力矩(映射到MIT发�?
    void SetMode_Type3(int& motor_index, int mode);
    void SendCommand_Type3(int& motor_index);
    void QueryPos_Type3(int& motor_index);

    // --- Type 4 (RoboMaster C620) 私有逻辑封装 ---
    void EnableMotor_Type4(int& motor_index);
    void DisableMotor_Type4(int& motor_index);
    void ClearError_Type4(int& motor_index);
    void SetZero_Type4(int& motor_index);
    //  * mode: 保留字段，C620 仅支�?CAN 电流环指�?
    void SetMode_Type4(int& motor_index, int mode);
    void SendCommand_Type4(int& motor_index);
    void QueryPos_Type4(int& motor_index);
};

