#pragma once

#include "IMU_FiltersBase.h"

/*!
NULL IMU Filters class, useful for test code.
*/
class IMU_FiltersNull : public IMU_FiltersBase {
public:
    virtual ~IMU_FiltersNull() = default;
    IMU_FiltersNull() = default;

    // IMU_FiltersNull is not copyable or moveable
    IMU_FiltersNull(const IMU_FiltersNull&) = delete;
    IMU_FiltersNull& operator=(const IMU_FiltersNull&) = delete;
    IMU_FiltersNull(IMU_FiltersNull&&) = delete;
    IMU_FiltersNull& operator=(IMU_FiltersNull&&) = delete;

    void setFilters(const xyz_t& gyroRPS) override { (void)gyroRPS; };
    void filter(xyz_t& gyroRPS, xyz_t& acc, float deltaT) override {
        (void)gyroRPS;
        (void)acc;
        (void)deltaT;
    }
};
