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
    // 接收线程：融合逻辑
    void ReceiveLoop() ;

private:
    // 辅助工具
    void sendRawFrame(uint8_t chan, uint32_t type, uint16_t data_area, uint8_t motor_id, uint8_t* data);

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
};
