#pragma once

struct xyz_t;

class IMU_FiltersBase {
public:
    virtual void setFilters(const xyz_t& gyroRPS) = 0;
    virtual void filter(xyz_t& gyroRPS, xyz_t& acc, float deltaT) = 0;
};
