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

    void filter(xyz_t& gyroRPS, xyz_t& acc, float deltaT) override;
    void setFilters(const filters_t& filters, float deltaT) override;
};

void IMU_FiltersNull::filter(xyz_t& gyroRPS, xyz_t& acc, float deltaT)
{
    (void)gyroRPS;
    (void)acc;
    (void)deltaT;
}

void IMU_FiltersNull::setFilters(const filters_t& filters, float deltaT)
{
    (void)filters;
    (void)deltaT;
}
