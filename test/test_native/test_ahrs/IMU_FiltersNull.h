#pragma once

#include "ImuFiltersBase.h"

/*!
NULL IMU Filters class.
*/
class IMU_FiltersNull : public ImuFiltersBase {
public:
    virtual ~IMU_FiltersNull() = default;
    IMU_FiltersNull() = default;

    // IMU_FiltersNull is not copyable or moveable
    IMU_FiltersNull(const IMU_FiltersNull&) = delete;
    IMU_FiltersNull& operator=(const IMU_FiltersNull&) = delete;
    IMU_FiltersNull(IMU_FiltersNull&&) = delete;
    IMU_FiltersNull& operator=(IMU_FiltersNull&&) = delete;
};
