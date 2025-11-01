#include "VehicleControllerBase.h"

#include <SensorFusion.h>
#include <TimeMicroseconds.h>
#include <cmath>

/*!
Constructor: sets the sensor fusion filter, IMU, and IMU filters
*/
AHRS::AHRS(task_e taskType, VehicleControllerBase& vehicleController, SensorFusionFilterBase& sensorFusionFilter, IMU_Base& imuSensor, IMU_FiltersBase& imuFilters) :
    _sensorFusionFilter(sensorFusionFilter),
    _IMU(imuSensor),
    _imuFilters(imuFilters),
    _vehicleController(vehicleController),
    _flags(flags(sensorFusionFilter, imuSensor)),
    _taskType(taskType)
{
    if (_taskType == INTERRUPT_DRIVEN) {
        _IMU.setInterruptDriven();
    }

    setSensorFusionInitializing(_flags & SENSOR_FUSION_REQUIRES_INITIALIZATION);
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

void AHRS::setSensorFusionInitializing(bool sensorFusionInitializing)
{
    _sensorFusionInitializing = sensorFusionInitializing;
    _vehicleController.setSensorFusionFilterIsInitializing(sensorFusionInitializing);
}

/*!
Main AHRS task function.

1. Reads the IMU, or if interrupt driven gets the IMU value that was read in the ISR.
2. Filters the IMU reading.
3. Perfroms sensor fusion to calculate the orientation quaternion.
4. Calls vehicle controller `updateOutputsUsingPIDs`.
*/
bool AHRS::readIMUandUpdateOrientation(uint32_t timeMicroseconds, uint32_t timeMicrosecondsDelta)
{
    _ahrsData.deltaT = static_cast<float>(timeMicrosecondsDelta) * 0.000001F; // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    _ahrsData.timeMicroseconds = timeMicroseconds;

    const timeUs32_t time0 = timeMicroseconds;

#if defined(LIBRARY_STABILIZED_VEHICLE_IMU_DOES_SENSOR_FUSION)
    // Some IMUs, eg the BNO085, do on-chip sensor fusion
    _ahrsData.accGyroRPS.gyroRPS = _IMU.readGyroRPS();
    const timeUs32_t time1 = timeUs();
    _timeChecksMicroseconds[0] = time1 - time0;
    _timeChecksMicroseconds[1] = 0; // filter time set to zero, since filtering is as part of IMU sensor fusion
    _ahrsData.orientation = _IMU.readOrientation();
    const timeUs32_t time4 = timeUs();
    _timeChecksMicroseconds[2] = time4 - time1;
    _timeChecksMicroseconds[3] = 0;

#else

    if (_taskType == INTERRUPT_DRIVEN) { // NOLINT(bugprone-branch-clone)
        // the data was read in the IMU interrupt service routine, so we can just get the data, rather than read it
        _ahrsData.accGyroRPS = _IMU.getAccGyroRPS();
    } else {
        _ahrsData.accGyroRPS = _IMU.readAccGyroRPS();
    }

    // Gyros are generally specified to +/- 2000 DPS, in a crash this limit can be exceeded and cause an overflow and a sign reversal in the output.
    checkGyroOverflowZ();

#if defined(LIBRARY_STABILIZED_VEHICLE_USE_AHRS_TIME_CHECKS_FINE)
    const timeUs32_t time1 = timeUs();
    _timeChecksMicroseconds[0] = time1 - time0;
#endif

#if defined(LIBRARY_STABILIZED_VEHICLE_USE_AHRS_SET_FILTERS)
    // the filter parameters, in particular the dynamic notch filters, if any, can be set here
    _imuFilters.setFilters();
#endif

#if defined(LIBRARY_STABILIZED_VEHICLE_USE_AHRS_TIME_CHECKS_FINE)
    const timeUs32_t time2 = timeUs();
    _timeChecksMicroseconds[1] = time2 - time1;
#endif

    // apply the filters
    _ahrsData.gyroRPS_unfiltered = _ahrsData.accGyroRPS.gyroRPS; // unfiltered value saved for blackbox recording
    _imuFilters.filter(_ahrsData.accGyroRPS.gyroRPS, _ahrsData.accGyroRPS.acc, _ahrsData.deltaT); // 15us, 207us

#if defined(LIBRARY_STABILIZED_VEHICLE_USE_AHRS_TIME_CHECKS_FINE)
    const timeUs32_t time3 = timeUs();
    _timeChecksMicroseconds[2] = time3 - time2;
#endif

    _ahrsData.orientation = _sensorFusionFilter.update(_ahrsData.accGyroRPS.gyroRPS, _ahrsData.accGyroRPS.acc, _ahrsData.deltaT); // 15us, 140us

#if defined(LIBRARY_STABILIZED_VEHICLE_USE_AHRS_TIME_CHECKS_FINE)
    const timeUs32_t time4 = timeUs();
    _timeChecksMicroseconds[3] = time4 - time3;
#endif

    if (sensorFusionFilterIsInitializing()) {
        checkFusionFilterConvergence(_ahrsData.accGyroRPS.acc, _ahrsData.orientation);
    }
#endif // LIBRARY_STABILIZED_VEHICLE_IMU_DOES_SENSOR_FUSION

    _vehicleController.updateOutputsUsingPIDs(_ahrsData);

    const timeUs32_t time5 = timeUs();
    _timeChecksMicroseconds[4] = time5 - time0;

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

/*!
Returns the AHRS data.
*/
AHRS::ahrs_data_t AHRS::getAhrsDataForTest() const
{
    return _ahrsData;
}

void AHRS::checkFusionFilterConvergence(const xyz_t& acc, const Quaternion& orientation)
{
    static constexpr float twoDegreesInRadians = 2.0F * Quaternion::degreesToRadians;

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
