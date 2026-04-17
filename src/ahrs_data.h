#pragma once

#include <cassert>
#include <cstdint>
#include <quaternion.h>


struct ahrs_data_t {
    acc_gyro_rps_t acc_gyro_rps;
    xyz_t gyro_rps_unfiltered;
    Quaternion orientation;
    float delta_t;
    uint32_t time_microseconds;
    uint32_t filler; // pad ahrs_data_t to exactly 64 bytes
};

static_assert(sizeof(ahrs_data_t) == 64);