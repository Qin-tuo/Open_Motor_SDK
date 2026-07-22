#include "mapper.hpp"

uint64_t MotorMapper::key(int device_index, bool extended_frame, int can_id) {
    return (static_cast<uint64_t>(static_cast<uint32_t>(device_index)) << 32) |
           (static_cast<uint64_t>(extended_frame) << 31) |
           static_cast<uint32_t>(can_id);
}

bool MotorMapper::add(int device_index, bool extended_frame, int can_id, int motor_index) {
    return entries_.emplace(key(device_index, extended_frame, can_id), motor_index).second;
}

int MotorMapper::get_id(int device_index, bool extended_frame, int can_id) const {
    const auto it = entries_.find(key(device_index, extended_frame, can_id));
    return it == entries_.end() ? -1 : it->second;
}
