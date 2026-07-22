#pragma once
#include <cstdint>
#include <unordered_map>

class MotorMapper {
private:
    std::unordered_map<uint64_t, int> entries_;
    static uint64_t key(int device_index, bool extended_frame, int can_id);

public:
    bool add(int device_index, bool extended_frame, int can_id, int motor_index);
    int get_id(int device_index, bool extended_frame, int can_id) const;
};
