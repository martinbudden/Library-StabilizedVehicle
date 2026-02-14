#pragma once

struct xyz_t;

class IMU_FiltersBase {
public:
    virtual ~IMU_FiltersBase() = default;
    virtual void filter(xyz_t& gyroRPS, xyz_t& acc, float delta_t) { (void)gyroRPS; (void)acc; (void)delta_t; }
};
