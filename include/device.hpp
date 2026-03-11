#pragma once
#include "types.hpp"
#include "mapper.hpp"
#include "usb_can.h" // 假设这是你的CAN库头文件
#include <thread>
#include <vector>
#include <atomic>
#include <string>
#include <iostream>
#include <cstring> 


// --- DeviceX (Unified Device) ---
// 现在它是唯一的设备类，内部根据 api_type 区分逻辑
class DeviceX{
protected:
    int32_t device_handle = 0;
    int device_global_index = -1;
    std::vector<Motor_CAN_Struct>* p_motors_data; 
    TopoMapper* p_mapper;
    std::atomic<bool> is_running{false};
    std::thread rx_thread;

public:

    ~DeviceX() { 
        is_running = false; 
        if(rx_thread.joinable()) rx_thread.join(); 
    };
    // 公共接口：包含 api_type 判断分发逻辑
    bool Init(int32_t handle, const std::string& dev_name, int dev_idx, 
                      std::vector<Motor_CAN_Struct>* data_ptr, TopoMapper* mapper_ptr);
    void SendCommand(int& motor_index) ;
    void EnableMotor(int& motor_index) ;
    void DisableMotor(int& motor_index) ;
    void ClearError(int& motor_index) ;
    void SetMode(int& motor_index, int mode) ;
    void QueryPos(int& motor_index);
    int SetType5CurrentPI(int& motor_index, uint32_t kp, uint32_t ki, bool save = false);
    int SetType5SpeedPI(int& motor_index, uint32_t kp, uint32_t ki, bool save = false);
    int SetType5PositionPI(int& motor_index, uint32_t kp, uint32_t ki, bool save = false);
    int SaveType5Params(int& motor_index);
    // 接收线程：融合逻辑
    void ReceiveLoop() ;

private:
    // 辅助工具
    void sendRawFrame(uint8_t chan, uint32_t type, uint16_t data_area, uint8_t motor_id, uint8_t* data);
    void sendType5Write(uint8_t chan, uint16_t canid, uint8_t addr, int32_t value, bool fast_write);
    void sendType5Read(uint8_t chan, uint16_t canid, uint8_t addr);
    int setType5PIRegs(int& motor_index, uint8_t p_addr, uint32_t kp, uint8_t i_addr, uint32_t ki, bool save);

    // --- Type 1 (灵足/LimX) 原始逻辑封装 ---
    void EnableMotor_Type1(int& motor_index);
    void DisableMotor_Type1(int& motor_index);
    void ClearError_Type1(int& motor_index);
    void SetMode_Type1(int& motor_index, int mode);
    void SendCommand_Type1(int& motor_index);
    void QueryPos_Type1(int& motor_index);
    // --- Type 2 (LK/宇树早期) 原始逻辑封装 ---
    void EnableMotor_Type2(int& motor_index);
    void DisableMotor_Type2(int& motor_index);
    void ClearError_Type2(int& motor_index);
    //  * mode: 1-运控(软闭环), 2-位置(硬闭环), 3-速度(硬闭环) 外部做了适配统一。
    void SetMode_Type2(int& motor_index, int mode);
    void SendCommand_Type2(int& motor_index);
    void QueryPos_Type2(int& motor_index);

    // --- Type 3 (达妙/DM) 私有逻辑封装 ---
    void EnableMotor_Type3(int& motor_index);
    void DisableMotor_Type3(int& motor_index);
    void ClearError_Type3(int& motor_index);
    //  * mode: 0-MIT, 1-位置速度(PV), 2-速度, 3-力矩(映射到MIT发送)
    void SetMode_Type3(int& motor_index, int mode);
    void SendCommand_Type3(int& motor_index);
    void QueryPos_Type3(int& motor_index);

    // --- Type 4 (RoboMaster C620) 私有逻辑封装 ---
    void EnableMotor_Type4(int& motor_index);
    void DisableMotor_Type4(int& motor_index);
    void ClearError_Type4(int& motor_index);
    //  * mode: 保留字段，C620 仅支持 CAN 电流环指令
    void SetMode_Type4(int& motor_index, int mode);
    void SendCommand_Type4(int& motor_index);
    void QueryPos_Type4(int& motor_index);

    // --- Type 5 (意优 PP11 / 行星 V3) 私有逻辑封装 ---
    void EnableMotor_Type5(int& motor_index);
    void DisableMotor_Type5(int& motor_index);
    void ClearError_Type5(int& motor_index);
    void SetMode_Type5(int& motor_index, int mode);
    void SendCommand_Type5(int& motor_index);
    void QueryPos_Type5(int& motor_index);

    // --- Type 6 (ENCOS) 私有逻辑封装 ---
    void EnableMotor_Type6(int& motor_index);
    void DisableMotor_Type6(int& motor_index);
    void ClearError_Type6(int& motor_index);
    void SetMode_Type6(int& motor_index, int mode);
    void SendCommand_Type6(int& motor_index);
    void QueryPos_Type6(int& motor_index);
};
