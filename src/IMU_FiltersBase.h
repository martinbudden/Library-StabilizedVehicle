#pragma once

#include <cstdint>

struct xyz_t;

class IMU_FiltersBase {
public:
    // Filter parameters choosen to be compatible with MultiWii Serial Protocol MSP_FILTER_CONFIG and MSP_SET_FILTER_CONFIG
    struct filters_t {
        enum { PT1 = 0, BIQUAD, PT2, PT3 }; // filter types
        uint16_t gyro_notch1_hz;
        uint16_t gyro_notch1_cutoff;
        uint16_t gyro_notch2_hz;
        uint16_t gyro_notch2_cutoff;
        uint16_t gyro_lpf1_hz;
        uint16_t gyro_lpf2_hz;
        uint16_t gyro_dynamic_lpf1_min_hz;
        uint16_t gyro_dynamic_lpf1_max_hz;
        uint8_t gyro_lpf1_type;
        uint8_t gyro_lpf2_type;
        uint8_t gyro_hardware_lpf;
        uint8_t rpm_filter_harmonics;
        uint8_t rpm_filter_min_hz;
    };
public:
    virtual void filter(xyz_t& gyroRPS, xyz_t& acc, float deltaT) = 0;
    virtual void setFilters(const filters_t& filters, float deltaT) = 0;
    const filters_t& getFilters() const { return _filters; }
protected:
    filters_t _filters {};
};