#pragma once
#include "types.hpp"
#include <vector>

class MotorMapper {
private:
    uint _num_dims;
    std::vector<uint> _strides;
    std::vector<int> _loc_to_id_lut;
    std::vector<uint> _coord_pool;
    std::vector<int> _id_to_pool_offset;

    inline uint _calculate_flat_index(const std::vector<uint>& coords) const {
        uint flat_index = 0;
        for (uint i = 0; i < _num_dims; ++i) {
            flat_index += coords[i] * _strides[i];
        }
        return flat_index;
    }

public:
    MotorMapper() : _num_dims(0) {}

    MotorMapper(std::vector<uint> dimension_limits, std::vector<std::vector<uint>> data_matrix);

    int get_id(const std::vector<uint>& loc) const;

    std::vector<uint> get_loc(uint uid) const;
};

using TopoMapper = MotorMapper;