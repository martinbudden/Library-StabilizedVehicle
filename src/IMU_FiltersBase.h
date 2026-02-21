#pragma once

struct xyz_t;

class IMU_FiltersBase {
public:
    virtual ~IMU_FiltersBase() = default;
    virtual void filter(xyz_t& gyro_rps, xyz_t& acc, float delta_t) { (void)gyro_rps; (void)acc; (void)delta_t; }
};
