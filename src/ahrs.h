#pragma once

#include <array>
#include <cassert>
#include <cstdint>

#include <quaternion.h>

#if defined(FRAMEWORK_USE_FREERTOS)
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#else
#if defined(FRAMEWORK_ARDUINO_STM32)
#include <STM32FreeRTOS.h>
#endif
#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>
#endif
#endif
#if defined(FRAMEWORK_RPI_PICO)
#include <pico/critical_section.h>
#include <pico/mutex.h>
#endif

class ImuBase;
class ImuFiltersBase; 
class SensorFusionFilterBase;
class TaskBase;
class VehicleControllerBase;

struct ahrs_data_t {
    acc_gyro_rps_t acc_gyro_rps;
    xyz_t gyro_rps_unfiltered;
    Quaternion orientation;
    float delta_t;
    uint32_t time_microseconds;
    uint32_t filler; // pad ahrs_data_t to exactly 64 bytes
};

/*!
Attitude and Heading Reference System.
*/
class Ahrs {
public:
    static constexpr int TIME_CHECKS_COUNT = 8;
    enum task_e { INTERRUPT_DRIVEN, TIMER_DRIVEN };

    static constexpr uint32_t IMU_AUTO_CALIBRATES = 0x01;
    static constexpr uint32_t IMU_PERFORMS_SENSOR_FUSION = 0x02;
    static constexpr uint32_t SENSOR_FUSION_REQUIRES_INITIALIZATION = 0x04;

    static constexpr float RADIANS_TO_DEGREES = 180.0F / 3.14159265358979323846F;
    static constexpr float DEGREES_TO_RADIANS = 3.14159265358979323846F / 180.0F;
public:
    Ahrs(task_e taskType, SensorFusionFilterBase& sensor_fusion_filter, ImuBase& imuSensor);
public:
    // class is not copyable or moveable
    Ahrs(const Ahrs&) = delete;
    Ahrs& operator=(const Ahrs&) = delete;
    Ahrs(Ahrs&&) = delete;
    Ahrs& operator=(Ahrs&&) = delete;
public:
    const ImuBase& get_imu() const { return _IMU; }
    ImuBase& get_imu_mutable() { return _IMU; };

    xyz_t get_gyro_offset() const;
    void set_gyro_offset(const xyz_t& offset);
    xyz_t get_acc_offset() const;
    void set_acc_offset(const xyz_t& offset);

    xyz_t get_gyro_offset_mapped() const;
    void set_gyro_offset_mapped(const xyz_t& offset);
    xyz_t get_acc_offset_mapped() const;
    void set_acc_offset_mapped(const xyz_t& offset);

    void read_gyro_raw(int32_t& x, int32_t& y, int32_t& z) const;
    void read_acc_raw(int32_t& x, int32_t& y, int32_t& z) const;
    ahrs_data_t get_ahrs_data_for_test() const;

    void check_fusion_filter_convergence(const xyz_t& acc, const Quaternion& orientation, VehicleControllerBase& vehicle_controller);
    inline uint32_t get_flags() const { return _flags; }

    inline uint32_t get_time_checks_microseconds(size_t index) const { return _timeChecksMicroseconds[index]; } //!< Instrumentation time checks
    inline const TaskBase* get_task() const { return _task; } //!< Used to get task data for instrumentation
    inline void set_task(const TaskBase* task) { _task = task; }
private:
    static uint32_t flags(const SensorFusionFilterBase& sensor_fusion_filter, const ImuBase& imuSensor);
public:
    const ahrs_data_t& read_imu_and_update_orientation(uint32_t time_microseconds, uint32_t time_microsecondsDelta, ImuFiltersBase& imu_filters, VehicleControllerBase& vehicle_controller);
    void set_overflow_sign_change_threshold_rps(float overflowSignChangeThresholdRPS) { _overflow_sign_change_threshold_rps_squared = overflowSignChangeThresholdRPS*overflowSignChangeThresholdRPS; }
    // Check for overflow on z axis, ie sign of z-value has changed when the z-value is large
    inline void check_gyro_overflow_z() {
        if (_ahrs_data.acc_gyro_rps.gyro_rps.z * _gyro_rps_previous.z < -_overflow_sign_change_threshold_rps_squared) {
            // we've had a sign change of a large value, ie from (say) 1900 to -1950, so this is an overflow, so don't accept the new gyro z-value
            _ahrs_data.acc_gyro_rps.gyro_rps.z = _gyro_rps_previous.z;
        } else {
            // normal sign change, ie from (say) 20 to -10, so set _gyro_rps_previous for next time round
            _gyro_rps_previous.z = _ahrs_data.acc_gyro_rps.gyro_rps.z;
        }
    }
    void set_acc_gyro_rps(const acc_gyro_rps_t& acc_gyro_rps) { _ahrs_data.acc_gyro_rps = acc_gyro_rps; } //!< For testing
private:
    SensorFusionFilterBase& _sensor_fusion_filter;
    ImuBase& _IMU;
    const TaskBase* _task {nullptr};

    float _overflow_sign_change_threshold_rps_squared {1500.0F * DEGREES_TO_RADIANS * 1500.0F * DEGREES_TO_RADIANS};
    xyz_t _gyro_rps_previous {}; //!< For overflow checking
    ahrs_data_t _ahrs_data {};
    uint32_t _sensor_fusion_filter_is_initializing {true};
    const uint32_t _flags;
    task_e _taskType;

    // instrumentation data
    std::array<uint32_t, TIME_CHECKS_COUNT + 1> _timeChecksMicroseconds {};
};
