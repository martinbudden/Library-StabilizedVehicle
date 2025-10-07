#pragma once

struct xyz_t;

class IMU_FiltersBase {
public:
    virtual void setFilters() {};
    virtual void filter(xyz_t& gyroRPS, xyz_t& acc, float deltaT) {};
};
