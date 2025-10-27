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

The AHRS uses the ENU (East North Up) coordinate frame.
*/
class AHRS {
public:
    struct data_t {
        float deltaT;
        xyz_t gyroRPS;
        xyz_t gyroRPS_unfiltered;
        xyz_t acc;
    };
    static constexpr int TIME_CHECKS_COUNT = 8;
    enum task_e { INTERRUPT_DRIVEN, TIMER_DRIVEN };
    enum : uint32_t {
        IMU_AUTO_CALIBRATES = 0x01,
        IMU_PERFORMS_SENSOR_FUSION = 0x02,
        SENSOR_FUSION_REQUIRES_INITIALIZATION = 0x04
     };
    enum  sensors_e {
        SENSOR_GYROSCOPE = 0x01,
        SENSOR_ACCELEROMETER = 0x02,
        SENSOR_BAROMETER = 0x04,
        SENSOR_MAGNETOMETER = 0x08,
        SENSOR_RANGEFINDER = 0x10,
        SENSOR_GPS = 0x20,
        SENSOR_GPS_MAGNETOMETER = 0x40
    };
    static constexpr float degreesToRadians = static_cast<float>(M_PI / 180.0);
public:
    AHRS(task_e taskType, SensorFusionFilterBase& sensorFusionFilter, IMU_Base& imuSensor, IMU_FiltersBase& imuFilters);
public:
    void setVehicleController(VehicleControllerBase* vehicleController);
public:
    // class is not copyable or moveable
    AHRS(const AHRS&) = delete;
    AHRS& operator=(const AHRS&) = delete;
    AHRS(AHRS&&) = delete;
    AHRS& operator=(AHRS&&) = delete;
public:
    bool isSensorAvailable(sensors_e sensor) const;

    // MSP compatible sensor IDs
    enum magnetometer_e { MAGNETOMETER_DEFAULT = 0, MAGNETOMETER_NONE = 1 };
    enum barometer_e { BAROMETER_DEFAULT = 0, BAROMETER_NONE = 1, BAROMETER_VIRTUAL = 11 };
    enum rangefinder_e { RANGEFINDER_NONE = 0 };
    uint8_t getBarometerID_MSP() const { return BAROMETER_NONE; }
    uint8_t getMagnetometerID_MSP() const {return MAGNETOMETER_NONE; }
    uint8_t getRangeFinderID_MSP() const { return RANGEFINDER_NONE; }

    const IMU_Base& getIMU() const { return _IMU; }
    IMU_Base& getIMU() { return _IMU; };

    IMU_Base::xyz_int32_t getGyroOffset() const;
    void setGyroOffset(const IMU_Base::xyz_int32_t& offset);
    IMU_Base::xyz_int32_t getAccOffset() const;
    void setAccOffset(const IMU_Base::xyz_int32_t& offset);

    static IMU_Base::xyz_int32_t mapOffset(const IMU_Base::xyz_int32_t& offset, IMU_Base::axis_order_e axisOrder);
    IMU_Base::xyz_int32_t getGyroOffsetMapped() const;
    void setGyroOffsetMapped(const IMU_Base::xyz_int32_t& offset);
    IMU_Base::xyz_int32_t getAccOffsetMapped() const;
    void setAccOffsetMapped(const IMU_Base::xyz_int32_t& offset);

    void readGyroRaw(int32_t& x, int32_t& y, int32_t& z) const;
    void readAccRaw(int32_t& x, int32_t& y, int32_t& z) const;
    void readMagRaw(int32_t& x, int32_t& y, int32_t& z) const;
    int32_t getAccOneG_Raw() const;
    data_t getAhrsDataForTest() const;

    void checkFusionFilterConvergence(const xyz_t& acc, const Quaternion& orientation);
    inline bool sensorFusionFilterIsInitializing() const { return  (_flags & SENSOR_FUSION_REQUIRES_INITIALIZATION) && _sensorFusionInitializing; }
    inline void setSensorFusionInitializing(bool sensorFusionInitializing) { _sensorFusionInitializing = sensorFusionInitializing; }
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
        if (_accGyroRPS.gyroRPS.z * _gyroRPS_previous.z < -_overflowSignChangeThresholdRPS_squared) {
            // we've had a sign change of a large value, ie from (say) 1900 to -1950, so this is an overflow, so don't accept the new gyro z-value
            _accGyroRPS.gyroRPS.z = _gyroRPS_previous.z;
        } else {
            // normal sign change, ie from (say) 20 to -10, so set _gyroRPS_previous for next time round
            _gyroRPS_previous.z = _accGyroRPS.gyroRPS.z;
        }
    }
    void setAccGyroRPS(const IMU_Base::accGyroRPS_t& accGyroRPS) { _accGyroRPS = accGyroRPS; } //!< For testing
private:
    SensorFusionFilterBase& _sensorFusionFilter;
    IMU_Base& _IMU;
    IMU_FiltersBase& _imuFilters;
    VehicleControllerBase* _vehicleController {nullptr};
    const TaskBase* _task {nullptr};

    float _overflowSignChangeThresholdRPS_squared {1500.0F * degreesToRadians * 1500.0F * degreesToRadians};
    xyz_t _gyroRPS_previous {};
    IMU_Base::accGyroRPS_t _accGyroRPS {};
    IMU_Base::accGyroRPS_t _accGyroRPS_unfiltered {};

    Quaternion _orientation {};
    uint32_t _sensorFusionInitializing {true};
    const uint32_t _flags;
    task_e _taskType;
    float _deltaT {};

    uint32_t _updateOutputsUsingPIDs {false};
    // instrumentation data
    std::array<uint32_t, TIME_CHECKS_COUNT + 1> _timeChecksMicroseconds {};
};
