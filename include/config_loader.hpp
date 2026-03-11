#pragma once
#include "types.hpp"
#include <vector>
#include <string>
#include "toml.hpp" 
#include <iostream>

class MotorConfigLoader {
private:
    static std::string trim(const std::string& str);
public:
    static std::vector<Motor_CAN_Info_Struct> loadConfig(const std::string& filename);
};


