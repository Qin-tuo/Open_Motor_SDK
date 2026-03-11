#include "mapper.hpp"

MotorMapper::MotorMapper(std::vector<uint> dimension_limits, std::vector<std::vector<uint>> data_matrix) {
    _num_dims = dimension_limits.size();

    _strides.resize(_num_dims);
    size_t total_capacity = 1;

    for (int i = _num_dims - 1; i >= 0; --i) {
        _strides[i] = total_capacity;
        total_capacity *= dimension_limits[i];
    }

    _loc_to_id_lut.assign(total_capacity, -1);

    uint max_user_id = 0;
    for (const auto& row : data_matrix) {
        if (!row.empty()) {
            uint uid = row.back();
            if (uid > max_user_id) max_user_id = uid;
        }
    }

    _id_to_pool_offset.assign(max_user_id + 1, -1);
    _coord_pool.reserve(data_matrix.size() * _num_dims);

    for (const auto& row : data_matrix) {
        if (row.size() != _num_dims + 1) continue;

        uint user_id = row.back();
        std::vector<uint> coords(row.begin(), row.begin() + _num_dims);

        uint flat_idx = _calculate_flat_index(coords);
        if (flat_idx < _loc_to_id_lut.size()) {
            _loc_to_id_lut[flat_idx] = (int)user_id;
        }

        _id_to_pool_offset[user_id] = (int)_coord_pool.size();
        for (uint val : coords) {
            _coord_pool.push_back(val);
        }
    }
}

int MotorMapper::get_id(const std::vector<uint>& loc) const {
    uint flat_index = _calculate_flat_index(loc);
    if (flat_index < _loc_to_id_lut.size()) {
        return _loc_to_id_lut[flat_index];
    }
    return -1;
}

std::vector<uint> MotorMapper::get_loc(uint uid) const {
    if (uid >= _id_to_pool_offset.size()) return {};

    int start_offset = _id_to_pool_offset[uid];
    if (start_offset == -1) return {};

    std::vector<uint> result;
    result.reserve(_num_dims);
    for (uint i = 0; i < _num_dims; ++i) {
        result.push_back(_coord_pool[start_offset + i]);
    }
    return result;
}