#pragma once

struct xyz_t;

class IMU_FiltersBase {
public:
    virtual ~IMU_FiltersBase() = default;
    virtual void setFilters() {};
    virtual void filter(xyz_t& gyroRPS, xyz_t& acc, float deltaT) { (void)gyroRPS; (void)acc; (void)deltaT; }
};
