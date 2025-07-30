#include "AHRS.h"
#include "AHRS_MessageQueueBase.h"
#include "IMU_FiltersBase.h"
#include "VehicleControllerBase.h"

#include <SensorFusion.h>
#include <TimeMicroSeconds.h>
#include <cmath>

/*!
Constructor: sets the sensor fusion filter, IMU, and IMU filters
*/
AHRS::AHRS(uint32_t taskIntervalMicroSeconds, SensorFusionFilterBase& sensorFusionFilter, IMU_Base& imuSensor, IMU_FiltersBase& imuFilters) :
    _sensorFusionFilter(sensorFusionFilter),
    _IMU(imuSensor),
    _imuFilters(imuFilters),
    _flags(flags(sensorFusionFilter, imuSensor)),
    _taskIntervalMicroSeconds(taskIntervalMicroSeconds),
    _taskIntervalSeconds(static_cast<float>(taskIntervalMicroSeconds)/1000.0F) // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
#if defined(USE_AHRS_DATA_MUTEX) && defined(USE_FREERTOS)
    , _ahrsDataMutex(xSemaphoreCreateRecursiveMutexStatic(&_ahrsDataMutexBuffer)) // statically allocate the imuDataMutex
#endif
{
#if defined(AHRS_IS_INTERRUPT_DRIVEN)
    _IMU.setInterruptDriven();
#endif

    setSensorFusionInitializing(_flags & SENSOR_FUSION_REQUIRES_INITIALIZATION);

#if defined(USE_FREERTOS)

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-offsetof"
#if defined(USE_AHRS_DATA_MUTEX)
    // ensure _ahrsDataMutexBuffer declared before _ahrsDataMutex
    static_assert(offsetof(AHRS, _ahrsDataMutex) > offsetof(AHRS, _ahrsDataMutexBuffer));
#endif
#pragma GCC diagnostic pop

#elif defined(FRAMEWORK_RPI_PICO)

    mutex_init(&_ahrsDataMutex);

#endif
}

uint32_t AHRS::flags(const SensorFusionFilterBase& sensorFusionFilter, const IMU_Base& imuSensor)
{
    uint32_t flags = 0;
    if (imuSensor.getFlags() & IMU_Base::IMU_AUTO_CALIBRATES) {
        flags |= IMU_AUTO_CALIBRATES;
    }
    if (imuSensor.getFlags() & IMU_Base::IMU_PERFORMS_SENSOR_FUSION) {
        flags |= IMU_PERFORMS_SENSOR_FUSION;
    }
    if (sensorFusionFilter.requiresInitialization()) {
        flags |= SENSOR_FUSION_REQUIRES_INITIALIZATION;
    }
    return flags;
}

bool AHRS::isSensorAvailable(sensors_e sensor) const
{
    switch (sensor) {
    case SENSOR_GYROSCOPE: // NOLINT(bugprone-branch-clone)
        return true;
    case SENSOR_ACCELEROMETER:
        return true;
    case SENSOR_BAROMETER: // NOLINT(bugprone-branch-clone)
        return false;
    case SENSOR_MAGNETOMETER:
        return false;
    case SENSOR_RANGEFINDER:
        return false;
    case SENSOR_GPS:
        return false;
    case SENSOR_GPS_MAGNETOMETER:
        return false;
    default:
        return false;
    }
}
void AHRS::setVehicleController(VehicleControllerBase* vehicleController)
{
    _vehicleController = vehicleController;
}

void AHRS::setMessageQueue(AHRS_MessageQueueBase* messageQueue)
{
    _messageQueue = messageQueue;
}

/*!
Main AHRS task function. Reads the IMU and uses the sensor fusion filter to update the orientation quaternion.
Returns false if there was no new data to be read from the IMU.

NOTE: calls to YIELD_TASK have no effect on multi-core implementations, but are useful for single-core variants.
*/
//bool AHRS::readIMUandUpdateOrientation(float deltaT, uint32_t tickCountDelta)
bool AHRS::readIMUandUpdateOrientation(uint32_t timeMicroSeconds, uint32_t timeMicroSecondsDelta)
{
    _tickCountDelta = timeMicroSecondsDelta / 1000; // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    const float deltaT = static_cast<float>(timeMicroSecondsDelta) * 0.000001F;

    const timeUs32_t time0 = timeMicroSeconds;

#if defined(IMU_DOES_SENSOR_FUSION)
    // Some IMUs, eg the BNO085, do on-chip sensor fusion
    _accGyroRPS.gyroRPS = _IMU.readGyroRPS();
    const timeUs32_t time1 = timeUs();
    _timeChecksMicroSeconds[0] = time1 - time0;
    _timeChecksMicroSeconds[1] = 0; // filter time set to zero, since filtering is as part of IMU sensor fusion
    const Quaternion orientation = _IMU.readOrientation();
    const timeUs32_t time4 = timeUs();
    _timeChecksMicroSeconds[2] = time4 - time1;
    _timeChecksMicroSeconds[3] = 0;

#else

#if defined(AHRS_IS_INTERRUPT_DRIVEN)
    // the data was read in the IMU interrupt service routine, so we can just get the data, rather than read it
    _accGyroRPS = _IMU.getAccGyroRPS();
#else
    _accGyroRPS = _IMU.readAccGyroRPS();
#endif
#if defined(AHRS_TIME_CHECKS_FINE)
    const timeUs32_t time1 = timeUs();
    _timeChecksMicroSeconds[0] = time1 - time0;
#endif

    _imuFilters.setFilters();

#if defined(AHRS_TIME_CHECKS_FINE)
    const timeUs32_t time2 = timeUs();
    _timeChecksMicroSeconds[1] = time2 - time1;
#endif

    _accGyroRPS_unfiltered = _accGyroRPS;
    _imuFilters.filter(_accGyroRPS.gyroRPS, _accGyroRPS.acc, deltaT); // 15us, 207us

#if defined(AHRS_TIME_CHECKS_FINE)
    const timeUs32_t time3 = timeUs();
    _timeChecksMicroSeconds[2] = time3 - time2;
#endif

    const Quaternion orientation = _sensorFusionFilter.update(_accGyroRPS.gyroRPS, _accGyroRPS.acc, deltaT); // 15us, 140us

#if defined(AHRS_TIME_CHECKS_FINE)
    const timeUs32_t time4 = timeUs();
    _timeChecksMicroSeconds[3] = time4 - time3;
#endif

    if (sensorFusionFilterIsInitializing()) {
        checkFusionFilterConvergence(_accGyroRPS.acc, orientation);
    }
#endif // IMU_DOES_SENSOR_FUSION

    if (_vehicleController) {
        // If _updateOutputsUsingPIDs is set, then things have been configured so that updateOutputsUsingPIDs
        // is called by the AHRS (ie here) rather than the motor controller.
        _vehicleController->updateOutputsUsingPIDs(_accGyroRPS.gyroRPS, _accGyroRPS.acc, orientation, deltaT); //25us, 900us
    }

#if defined(AHRS_TIME_CHECKS_FINE)
    const timeUs32_t time5 = timeUs();
    _timeChecksMicroSeconds[4] = time5 - time4;
#endif

    if (_messageQueue) {
        _messageQueue->append(timeMicroSeconds, _accGyroRPS.gyroRPS, _accGyroRPS_unfiltered.gyroRPS, _accGyroRPS.acc);
    }

#if defined(AHRS_TIME_CHECKS_FINE)
    const timeUs32_t time6 = timeUs();
    _timeChecksMicroSeconds[5] = time6 - time5;
#endif

    const timeUs32_t time7 = timeUs();
    _timeChecksMicroSeconds[6] = time7 - time0;

    // If _vehicleController != nullptr then the locked data is only used for instrumentation (screen display and telemetry),
    // so it might be possible not to use the lock in this case.
    LOCK_AHRS_DATA();
    _ahrsDataUpdatedSinceLastRead = true;
    _orientationUpdatedSinceLastRead = true;
    _orientation = orientation;
    _accGyroRPS_locked = _accGyroRPS;
    _accGyroRPS_unfilteredLocked = _accGyroRPS_unfiltered;
    UNLOCK_AHRS_DATA();

    return true;
}

/*!
Read the raw gyro values. Used in calibration.
*/
void AHRS::readGyroRaw(int32_t& x, int32_t& y, int32_t& z) const
{
    const IMU_Base::xyz_int32_t gyro = _IMU.readGyroRaw();
    x = gyro.x;
    y = gyro.y;
    z = gyro.z;
}

/*!
Read the raw accelerometer values. Used in calibration.
*/
void AHRS::readAccRaw(int32_t& x, int32_t& y, int32_t& z) const
{
    const IMU_Base::xyz_int32_t acc = _IMU.readAccRaw();
    x = acc.x;
    y = acc.y;
    z = acc.z;
}

void AHRS::readMagRaw(int32_t& x, int32_t& y, int32_t& z) const
{
    x = 0;
    y = 0;
    z = 0;
}

int32_t AHRS::getAccOneG_Raw() const
{
    return _IMU.getAccOneG_Raw();
}


IMU_Base::xyz_int32_t AHRS::getGyroOffset() const
{
    return _IMU.getGyroOffset();
}

/*!
Set the gyro offset. Used in calibration.
*/
void AHRS::setGyroOffset(const IMU_Base::xyz_int32_t& offset)
{
    _IMU.setGyroOffset(offset);
}

IMU_Base::xyz_int32_t AHRS::getAccOffset() const
{
    return _IMU.getAccOffset();
}

/*!
Set the accelerometer offset. Used in calibration.
*/
void AHRS::setAccOffset(const IMU_Base::xyz_int32_t& offset)
{
    _IMU.setAccOffset(offset);
}

IMU_Base::xyz_int32_t AHRS::mapOffset(const IMU_Base::xyz_int32_t& offset, IMU_Base::axis_order_e axisOrder)
{
    xyz_t offsetF = { static_cast<float>(offset.x), static_cast<float>(offset.y), static_cast<float>(offset.z) };
    offsetF = IMU_Base::mapAxes(offsetF, axisOrder);
    const IMU_Base::xyz_int32_t offsetMapped = { static_cast<int32_t>(offsetF.x), static_cast<int32_t>(offsetF.y), static_cast<int32_t>(offsetF.z) };
    return offsetMapped;
}

IMU_Base::xyz_int32_t AHRS::getGyroOffsetMapped() const
{
    return mapOffset(_IMU.getGyroOffset(), _IMU.getAxisOrder());
}

void AHRS::setGyroOffsetMapped(const IMU_Base::xyz_int32_t& offset)
{
    _IMU.setGyroOffset(mapOffset(offset, IMU_Base::axisOrderInverse(_IMU.getAxisOrder())));
}

IMU_Base::xyz_int32_t AHRS::getAccOffsetMapped() const
{
    return mapOffset(_IMU.getAccOffset(), _IMU.getAxisOrder());
}

void AHRS::setAccOffsetMapped(const IMU_Base::xyz_int32_t& offset)
{
    _IMU.setAccOffset(mapOffset(offset, IMU_Base::axisOrderInverse(_IMU.getAxisOrder())));
}

Quaternion AHRS::getOrientationUsingLock(bool& updatedSinceLastRead) const
{
    LOCK_AHRS_DATA();
    updatedSinceLastRead = _orientationUpdatedSinceLastRead;
    _orientationUpdatedSinceLastRead = false;
    const Quaternion ret = _orientation;
    UNLOCK_AHRS_DATA();

    return ret;
}

Quaternion AHRS::getOrientationUsingLock() const
{
    LOCK_AHRS_DATA();
    _orientationUpdatedSinceLastRead = false;
    const Quaternion ret = _orientation;
    UNLOCK_AHRS_DATA();

    return ret;
}

/*!
Returns orientation without clearing the _orientationUpdatedSinceLastRead flag.
*/
Quaternion AHRS::getOrientationForInstrumentationUsingLock() const
{
    LOCK_AHRS_DATA();
    const Quaternion ret = _orientation;
    UNLOCK_AHRS_DATA();

    return ret;
}

AHRS::data_t AHRS::getAhrsDataUsingLock(bool& updatedSinceLastRead) const
{
    LOCK_AHRS_DATA();
    updatedSinceLastRead = _ahrsDataUpdatedSinceLastRead;
    _ahrsDataUpdatedSinceLastRead = false;
    const data_t ret {
        .tickCountDelta = _tickCountDelta,
        .gyroRPS = _accGyroRPS_locked.gyroRPS,
        .gyroRPS_unfiltered = _accGyroRPS_unfilteredLocked.gyroRPS,
        .acc = _accGyroRPS_locked.acc
    };
    UNLOCK_AHRS_DATA();

    return ret;
}

AHRS::data_t AHRS::getAhrsDataUsingLock() const
{
    LOCK_AHRS_DATA();
    _ahrsDataUpdatedSinceLastRead = false;
    const data_t ret {
        .tickCountDelta = _tickCountDelta,
        .gyroRPS = _accGyroRPS_locked.gyroRPS,
        .gyroRPS_unfiltered = _accGyroRPS_unfilteredLocked.gyroRPS,
        .acc = _accGyroRPS_locked.acc
    };
    UNLOCK_AHRS_DATA();

    return ret;
}

/*!
Returns AHRS data without clearing the _ahrsDataUpdatedSinceLastRead flag.
*/
AHRS::data_t AHRS::getAhrsDataForInstrumentationUsingLock() const
{
    LOCK_AHRS_DATA();
    const data_t ret {
        .tickCountDelta = _tickCountDelta,
        .gyroRPS = _accGyroRPS.gyroRPS,
        .gyroRPS_unfiltered = _accGyroRPS_unfilteredLocked.gyroRPS,
        .acc = _accGyroRPS.acc
    };
    UNLOCK_AHRS_DATA();

    return ret;
}

void AHRS::checkFusionFilterConvergence(const xyz_t& acc, const Quaternion& orientation)
{
    constexpr float twoDegreesInRadians = 2.0F * Quaternion::degreesToRadians;

    // NOTE COORDINATE TRANSFORM: Madgwick filter uses Euler angles where roll is defined as rotation around the x-axis and pitch is rotation around the y-axis.
    // For the Self Balancing Robot, pitch is rotation around the x-axis and roll is rotation around the y-axis,
    // so SBR.roll = Madgwick.pitch and SPR.pitch = Madgwick.roll
    const float madgwickRollAngleRadians = orientation.calculateRollRadians();
    const float accPitchAngleRadians = std::atan2(acc.y, acc.z);
    //const float accRollAngleRadians = std::atan2(-acc.x, sqrtf(acc.y*acc.y + acc.z*acc.z));

    //Serial.printf("acc:P%5.1f mag:P%5.1f         diff:%5.1f\r\n", accPitchAngleRadians/Quaternion::degreesToRadians, madgwickRollAngleRadians/Quaternion::degreesToRadians, fabsf(accPitchAngleRadians - madgwickRollAngleRadians)/Quaternion::degreesToRadians);
    if (fabsf(accPitchAngleRadians - madgwickRollAngleRadians) < twoDegreesInRadians && accPitchAngleRadians != madgwickRollAngleRadians) {
        // the angles have converged to within 2 degrees, so we can reduce the gain.
        setSensorFusionInitializing(false);
        static constexpr float gyroMeasurementError = 0.615F; // corresponds to gyro measurement error of 15*2.7 degrees/second, as discussed by Madgwick
        _sensorFusionFilter.setFreeParameters(gyroMeasurementError, 0.0F);
    }
}
