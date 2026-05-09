#pragma once

#include "mapper.hpp"
#include "types.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

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
