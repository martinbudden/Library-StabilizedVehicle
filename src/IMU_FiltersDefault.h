#pragma once

#include <Filters.h>
#include <IMU_FiltersBase.h>
#include <xyz_type.h>

/*!
Simple default with a LPF on each of the gyro and accelerometer axes.
*/
class IMU_FiltersDefault : public IMU_FiltersBase {
public:
    IMU_FiltersDefault(float frequencyCutoff, float deltaT);
    IMU_FiltersDefault();
public:
    virtual void filter(xyz_t& gyroRPS, xyz_t& acc, float deltaT) override;
    virtual void setFilters(const filters_t& filters, float deltaT) override;
protected:
    float _deltaT;
    PowerTransferFilter1 _gyroX_PT1_LPF1 {};
    PowerTransferFilter1 _gyroY_PT1_LPF1 {};
    PowerTransferFilter1 _gyroZ_PT1_LPF1 {};
    PowerTransferFilter1 _accX_PT1_LPF1 {};
    PowerTransferFilter1 _accY_PT1_LPF1 {};
    PowerTransferFilter1 _accZ_PT1_LPF1 {};
};