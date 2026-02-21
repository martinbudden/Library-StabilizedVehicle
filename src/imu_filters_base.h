#pragma once

struct xyz_t;

class ImuFiltersBase {
public:
    virtual ~ImuFiltersBase() = default;
    virtual void filter(xyz_t& gyro_rps, xyz_t& acc, float delta_t) { (void)gyro_rps; (void)acc; (void)delta_t; }
};
