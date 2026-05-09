#pragma once

#include "mapper.hpp"
#include "haitai_protocol.hpp"
#include "types.hpp"

#include <atomic>
#include <cstddef>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

// Unified SocketCAN device class. Protocol behavior is selected by api_type.
class DeviceX {
protected:
    int socket_fd = -1;
    std::string iface_name;
    int device_global_index = -1;
    std::vector<Motor_CAN_Struct>* p_motors_data = nullptr;
    TopoMapper* p_mapper = nullptr;
    std::atomic<bool> is_running{false};
    std::thread rx_thread;

public:
    ~DeviceX();

    bool Init(const std::string& iface, int dev_idx,
              std::vector<Motor_CAN_Struct>* data_ptr, TopoMapper* mapper_ptr);

    void SendCommand(int& motor_index);
    void EnableMotor(int& motor_index);
    void DisableMotor(int& motor_index);
    void ClearError(int& motor_index);
    void SetZero(int& motor_index);
    void SetMode(int& motor_index, int mode);
    void QueryPos(int& motor_index);
    void QueryVersion(int& motor_index);

    void ReceiveLoop();

private:
    bool openSocket(const std::string& iface, bool enable_canfd, int dbitrate = 1000000);
    bool sendFrameWithRetry(const void* frame, std::size_t frame_size, const char* tag);

    void sendExtendedFrame(uint32_t type, uint16_t data_area, uint8_t motor_id, const uint8_t* data);
    void sendExtendedIdFrame(uint32_t can_id, const uint8_t* data, uint8_t dlc = 8);
    void sendExtendedIdFdFrame(uint32_t can_id, const uint8_t* data, uint8_t len);
    void sendStandardFrame(uint32_t can_id, const uint8_t* data, uint8_t dlc = 8);
    void sendStandardFdFrame(uint32_t can_id, const uint8_t* data, uint8_t len, bool brs);
    void sendRawFrame(uint8_t chan, uint32_t type, uint16_t data_area, uint8_t motor_id, uint8_t* data);
    std::atomic<unsigned long long> enobufs_drop_count{0};
    std::atomic<unsigned long long> last_enobufs_log_ms{0};

    // Type 1 (LimX)
    void EnableMotor_Type1(int& motor_index);
    void DisableMotor_Type1(int& motor_index);
    void ClearError_Type1(int& motor_index);
    void SetZero_Type1(int& motor_index);
    void SetMode_Type1(int& motor_index, int mode);
    void SendCommand_Type1(int& motor_index);
    void QueryPos_Type1(int& motor_index);

    // Type 2 (LK)
    void EnableMotor_Type2(int& motor_index);
    void DisableMotor_Type2(int& motor_index);
    void ClearError_Type2(int& motor_index);
    void SetZero_Type2(int& motor_index);
    void SetMode_Type2(int& motor_index, int mode);
    void SendCommand_Type2(int& motor_index);
    void QueryPos_Type2(int& motor_index);

    // Type 3 (DM)
    void EnableMotor_Type3(int& motor_index);
    void DisableMotor_Type3(int& motor_index);
    void ClearError_Type3(int& motor_index);
    void SetZero_Type3(int& motor_index);
    void SetMode_Type3(int& motor_index, int mode);
    void SendCommand_Type3(int& motor_index);
    void QueryPos_Type3(int& motor_index);

    // Type 5 (HighTorque/高擎)
    void EnableMotor_Type5(int& motor_index);
    void DisableMotor_Type5(int& motor_index);
    void ClearError_Type5(int& motor_index);
    void SetZero_Type5(int& motor_index);
    void SetMode_Type5(int& motor_index, int mode);
    void SendCommand_Type5(int& motor_index);
    void QueryPos_Type5(int& motor_index);

    // Type 6 (AgiBot PowerFlow L28/PFL28)
    void EnableMotor_Type6(int& motor_index);
    void DisableMotor_Type6(int& motor_index);
    void ClearError_Type6(int& motor_index);
    void SetZero_Type6(int& motor_index);
    void SetMode_Type6(int& motor_index, int mode);
    void SendCommand_Type6(int& motor_index);
    void QueryPos_Type6(int& motor_index);

    // Type 7 (Haitai/海泰)
    void EnableMotor_Type7(int& motor_index);
    void DisableMotor_Type7(int& motor_index);
    void ClearError_Type7(int& motor_index);
    void SetZero_Type7(int& motor_index);
    void SetMode_Type7(int& motor_index, int mode);
    void SendCommand_Type7(int& motor_index);
    void QueryPos_Type7(int& motor_index);
    void QueryVersion_Type7(int& motor_index);
};
