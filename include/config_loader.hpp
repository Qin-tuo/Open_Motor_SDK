#pragma once
#include "types.hpp"
#include <string>
#include <vector>

class MotorConfigLoader {
public:
    static std::vector<Motor_CAN_Info_Struct> loadConfig(const std::string& filename);
};


