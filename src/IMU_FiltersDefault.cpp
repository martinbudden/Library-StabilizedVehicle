#include "IMU_FiltersDefault.h"


IMU_FiltersDefault::IMU_FiltersDefault(float frequencyCutoff, float deltaT) :
    _deltaT(deltaT),
    _gyroX_PT1_LPF1(frequencyCutoff, deltaT),
    _gyroY_PT1_LPF1(frequencyCutoff, deltaT),
    _gyroZ_PT1_LPF1(frequencyCutoff, deltaT),
    _accX_PT1_LPF1(frequencyCutoff, deltaT),
    _accY_PT1_LPF1(frequencyCutoff, deltaT),
    _accZ_PT1_LPF1(frequencyCutoff, deltaT)
{
}

IMU_FiltersDefault::IMU_FiltersDefault() :
    _deltaT(1.0F)
{
    _gyroX_PT1_LPF1.setToPassthrough();
    _gyroY_PT1_LPF1.setToPassthrough();
    _gyroZ_PT1_LPF1.setToPassthrough();

    _accX_PT1_LPF1.setToPassthrough();
    _accY_PT1_LPF1.setToPassthrough();
    _accZ_PT1_LPF1.setToPassthrough();
}

void IMU_FiltersDefault::filter(xyz_t& gyroRPS, xyz_t& acc, float deltaT) // cppcheck-suppress constParameterReference; false positive
{
    (void)deltaT;
    gyroRPS.x = _gyroX_PT1_LPF1.update(gyroRPS.x);
    gyroRPS.y = _gyroY_PT1_LPF1.update(gyroRPS.y);
    gyroRPS.z = _gyroZ_PT1_LPF1.update(gyroRPS.z);
    acc.x = _accX_PT1_LPF1.update(acc.x);
    acc.y = _accY_PT1_LPF1.update(acc.y);
    acc.z = _accZ_PT1_LPF1.update(acc.z);
};

void IMU_FiltersDefault::setFilters(const filters_t& filters, float deltaT)
{
    _filters = filters;

    if (filters.gyro_lpf1_hz == 0) {
        _gyroX_PT1_LPF1.setToPassthrough();
        _gyroY_PT1_LPF1.setToPassthrough();
        _gyroZ_PT1_LPF1.setToPassthrough();
    } else {
        // if the user has selected a filter, then provide a PowerTransfer1 filter.
        // If no filter selected, then set the filter to passthrough
        switch (filters.gyro_lpf1_type) {
        case filters_t::PT1: // NOLINT(bugprone-branch-clone)
            [[fallthrough]];
        case filters_t::PT2:
            [[fallthrough]];
        case filters_t::PT3:
            [[fallthrough]];
        case filters_t::BIQUAD:
            _gyroX_PT1_LPF1.setCutoffFrequencyAndReset(filters.gyro_lpf1_hz, deltaT);
            _gyroY_PT1_LPF1.setCutoffFrequencyAndReset(filters.gyro_lpf1_hz, deltaT);
            _gyroZ_PT1_LPF1.setCutoffFrequencyAndReset(filters.gyro_lpf1_hz, deltaT);
            break;
        default:
            _gyroX_PT1_LPF1.setToPassthrough();
            _gyroY_PT1_LPF1.setToPassthrough();
            _gyroZ_PT1_LPF1.setToPassthrough();
            break;
        }
    }
}