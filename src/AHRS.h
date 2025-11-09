#pragma once

#include <IMU_Base.h>
#include <IMU_FiltersBase.h>

#include <cassert>

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

class VehicleControllerBase;
class SensorFusionFilterBase;
class TaskBase;

/*!
Attitude and Heading Reference System.
*/
class AHRS {
public:
    struct ahrs_data_t {
        IMU_Base::accGyroRPS_t accGyroRPS;
        xyz_t gyroRPS_unfiltered;
        Quaternion orientation;
        float deltaT;
        uint32_t timeMicroseconds;
        uint32_t filler; // pad ahrs_data_t to exactly 64 bytes
    };
    static constexpr int TIME_CHECKS_COUNT = 8;
    enum task_e { INTERRUPT_DRIVEN, TIMER_DRIVEN };
    enum : uint32_t {
        IMU_AUTO_CALIBRATES = 0x01,
        IMU_PERFORMS_SENSOR_FUSION = 0x02,
        SENSOR_FUSION_REQUIRES_INITIALIZATION = 0x04
     };
    static constexpr float degreesToRadians = static_cast<float>(M_PI / 180.0);
public:
    AHRS(task_e taskType, VehicleControllerBase& vehicleController, SensorFusionFilterBase& sensorFusionFilter, IMU_Base& imuSensor, IMU_FiltersBase& imuFilters);
public:
    // class is not copyable or moveable
    AHRS(const AHRS&) = delete;
    AHRS& operator=(const AHRS&) = delete;
    AHRS(AHRS&&) = delete;
    AHRS& operator=(AHRS&&) = delete;
public:
    const IMU_Base& getIMU() const { return _IMU; }
    IMU_Base& getIMU() { return _IMU; };

    xyz_t getGyroOffset() const;
    void setGyroOffset(const xyz_t& offset);
    xyz_t getAccOffset() const;
    void setAccOffset(const xyz_t& offset);

    xyz_t getGyroOffsetMapped() const;
    void setGyroOffsetMapped(const xyz_t& offset);
    xyz_t getAccOffsetMapped() const;
    void setAccOffsetMapped(const xyz_t& offset);

    void readGyroRaw(int32_t& x, int32_t& y, int32_t& z) const;
    void readAccRaw(int32_t& x, int32_t& y, int32_t& z) const;
    ahrs_data_t getAhrsDataForTest() const;

    void checkFusionFilterConvergence(const xyz_t& acc, const Quaternion& orientation);
    inline bool sensorFusionFilterIsInitializing() const { return  (_flags & SENSOR_FUSION_REQUIRES_INITIALIZATION) && _sensorFusionInitializing; }
    void setSensorFusionInitializing(bool sensorFusionInitializing);
    inline uint32_t getFlags() const { return _flags; }

    IMU_FiltersBase& getIMU_Filters() const { return _imuFilters; }

    inline uint32_t getTimeChecksMicroseconds(size_t index) const { return _timeChecksMicroseconds[index]; } //!< Instrumentation time checks
    inline const TaskBase* getTask() const { return _task; } //!< Used to get task data for instrumentation
    inline void setTask(const TaskBase* task) { _task = task; }
private:
    static uint32_t flags(const SensorFusionFilterBase& sensorFusionFilter, const IMU_Base& imuSensor);
public:
    bool readIMUandUpdateOrientation(uint32_t timeMicroseconds, uint32_t timeMicrosecondsDelta);
    void setOverflowSignChangeThresholdRPS(float overflowSignChangeThresholdRPS) { _overflowSignChangeThresholdRPS_squared = overflowSignChangeThresholdRPS*overflowSignChangeThresholdRPS; }
    // Check for overflow on z axis, ie sign of z-value has changed when the z-value is large
    inline void checkGyroOverflowZ() {
        if (_ahrsData.accGyroRPS.gyroRPS.z * _gyroRPS_previous.z < -_overflowSignChangeThresholdRPS_squared) {
            // we've had a sign change of a large value, ie from (say) 1900 to -1950, so this is an overflow, so don't accept the new gyro z-value
            _ahrsData.accGyroRPS.gyroRPS.z = _gyroRPS_previous.z;
        } else {
            // normal sign change, ie from (say) 20 to -10, so set _gyroRPS_previous for next time round
            _gyroRPS_previous.z = _ahrsData.accGyroRPS.gyroRPS.z;
        }
    }
    void setAccGyroRPS(const IMU_Base::accGyroRPS_t& accGyroRPS) { _ahrsData.accGyroRPS = accGyroRPS; } //!< For testing
private:
    SensorFusionFilterBase& _sensorFusionFilter;
    IMU_Base& _IMU;
    IMU_FiltersBase& _imuFilters;
    VehicleControllerBase& _vehicleController;
    const TaskBase* _task {nullptr};

    float _overflowSignChangeThresholdRPS_squared {1500.0F * degreesToRadians * 1500.0F * degreesToRadians};
    xyz_t _gyroRPS_previous {}; //!< For overflow checking
    ahrs_data_t _ahrsData {};
    uint32_t _sensorFusionInitializing {true};
    const uint32_t _flags;
    task_e _taskType;

    uint32_t _updateOutputsUsingPIDs {false};
    // instrumentation data
    std::array<uint32_t, TIME_CHECKS_COUNT + 1> _timeChecksMicroseconds {};
};
