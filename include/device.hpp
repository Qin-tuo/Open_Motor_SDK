#pragma once

#include "mapper.hpp"
#include "types.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

struct HaitaiCommandFrame {
    uint32_t can_id = 0;
    uint8_t dlc = 0;
    std::array<uint8_t, 8> data{};
};

struct HaitaiFeedback {
    bool has_position = false;
    bool has_speed = false;
    bool has_current = false;
    bool has_torque = false;
    bool has_temperature = false;
    bool has_status = false;
    bool has_version = false;
    bool has_mit_limits = false;
    bool has_mit_state = false;

    float position_rad = 0.0f;
    float speed_rad_s = 0.0f;
    float current_a = 0.0f;
    float torque_nm = 0.0f;
    float temperature_c = 0.0f;
    float bus_voltage_v = 0.0f;
    float bus_current_a = 0.0f;
    float mit_pos_max_rad = 0.0f;
    float mit_vel_max_rad_s = 0.0f;
    float mit_torque_max_nm = 0.0f;

    uint16_t boot_version = 0;
    uint16_t app_version = 0;
    uint16_t hw_version = 0;
    uint8_t can_proto_version = 0;
    uint8_t mode = 0;
    uint8_t fault = 0;
    uint8_t mit_status = 0;
    bool mit_in_mode = false;
    bool mit_fault = false;
    uint8_t command = 0;
};

HaitaiCommandFrame build_haitai_command(uint8_t mode, float position_rad,
                                        float speed_rad_s, float current_a,
                                        uint32_t can_id = 0x01);

HaitaiCommandFrame build_haitai_mit_command(float position_rad, float speed_rad_s,
                                            float kp, float kd, float torque_nm,
                                            float pos_max_rad,
                                            float vel_max_rad_s,
                                            float torque_max_nm,
                                            uint32_t can_id = 0x01);

HaitaiCommandFrame build_haitai_mit_limits_config(float pos_max_rad,
                                                  float vel_max_rad_s,
                                                  float torque_max_nm,
                                                  uint32_t can_id = 0x01);

HaitaiCommandFrame make_haitai_simple_query(uint8_t cmd, uint32_t can_id);

bool parse_haitai_feedback(uint32_t can_id, const std::array<uint8_t, 8>& data,
                           uint8_t dlc, HaitaiFeedback& out,
                           float mit_pos_max_rad = 95.5f,
                           float mit_vel_max_rad_s = 45.0f,
                           float mit_torque_max_nm = 18.0f);

class FeetechServoDevice {
public:
    ~FeetechServoDevice();

    bool Init(const std::string& port, int baud, int dev_idx,
              std::vector<Motor_CAN_Struct>* data_ptr, TopoMapper* mapper_ptr);

    void EnableMotor(int& motor_index);
    void DisableMotor(int& motor_index);
    void ClearError(int& motor_index);
    void SetZero(int& motor_index);
    void SetMode(int& motor_index, int mode);
    void SendCommand(int& motor_index);
    void QueryPos(int& motor_index);
    void QueryVersion(int& motor_index);
    bool SetServoId(int old_id, int new_id);

private:
    int positionToCount(const Motor_CAN_Info_Struct& info, float position_rad) const;
    float countToPosition(const Motor_CAN_Info_Struct& info, int count) const;
    int speedToCount(float speed_rad_s) const;
    bool openSerial();
    void closeSerial();
    bool writePosition(int id, int position, int speed);
    bool enableTorque(int id, bool enable);
    int readRegisterByte(int id, uint8_t address);
    int readRegisterWord(int id, uint8_t address, bool signed_value, uint8_t sign_bit);
    bool writeRegisterByte(int id, uint8_t address, uint8_t value);
    bool writeRegister(int id, uint8_t address, const uint8_t* data, std::size_t length);
    bool readRegister(int id, uint8_t address, uint8_t* data, std::size_t length);
    bool writePacket(uint8_t id, uint8_t instruction, const uint8_t* params, std::size_t length);
    bool readStatusPacket(uint8_t expected_id, uint8_t* error, uint8_t* data, std::size_t length);
    void drainInput(int timeout_ms);

    int fd_ = -1;
    std::string port_;
    int baud_ = 500000;
    int device_global_index_ = -1;
    std::vector<Motor_CAN_Struct>* p_motors_data_ = nullptr;
    TopoMapper* p_mapper_ = nullptr;
    std::atomic<bool> is_open_{false};
};

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
    void writeType1Param(uint8_t motor_id, uint16_t index, float value);
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

    // Type 8 (ENCOS EC-A series)
    void EnableMotor_Type8(int& motor_index);
    void DisableMotor_Type8(int& motor_index);
    void ClearError_Type8(int& motor_index);
    void SetZero_Type8(int& motor_index);
    void SetMode_Type8(int& motor_index, int mode);
    void SendCommand_Type8(int& motor_index);
    void QueryPos_Type8(int& motor_index);
};
