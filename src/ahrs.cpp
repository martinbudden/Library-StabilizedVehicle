#include "ahrs.h"
#include "imu_filters_base.h"
#include "vehicle_controller_base.h"

#include <cmath>
#include <imu_base.h>
#include <sensor_fusion.h>
#include <time_microseconds.h>

/*!
Constructor: sets the sensor fusion filter, IMU, and IMU filters
*/
Ahrs::Ahrs(task_e taskType, SensorFusionFilterBase& sensor_fusion_filter, ImuBase& imuSensor) :
    _sensor_fusion_filter(sensor_fusion_filter),
    _IMU(imuSensor),
    _flags(flags(sensor_fusion_filter, imuSensor)),
    _taskType(taskType)
{
    // ensure ahrs_data_t padded to 64 bytes, may result in marginally faster copying
    static_assert(sizeof(ahrs_data_t)==64); // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

    if (_taskType == INTERRUPT_DRIVEN) {
        _IMU.set_interrupt_driven();
    }
}

uint32_t Ahrs::flags(const SensorFusionFilterBase& sensor_fusion_filter, const ImuBase& imuSensor)
{
    uint32_t flags = 0;
    if (imuSensor.get_flags() & ImuBase::IMU_AUTO_CALIBRATES) {
        flags |= IMU_AUTO_CALIBRATES;
    }
    if (imuSensor.get_flags() & ImuBase::IMU_PERFORMS_SENSOR_FUSION) {
        flags |= IMU_PERFORMS_SENSOR_FUSION;
    }
    if (sensor_fusion_filter.requires_initialization()) {
        flags |= SENSOR_FUSION_REQUIRES_INITIALIZATION;
    }
    return flags;
}

/*!
Main AHRS task function.

1. Reads the IMU, or if interrupt driven gets the IMU value that was read in the ISR.
2. Filters the IMU reading.
3. Perfroms sensor fusion to calculate the orientation quaternion.
4. Calls vehicle controller `update_outputs_using_pids`.
*/
const ahrs_data_t& Ahrs::read_imu_and_update_orientation(uint32_t time_microseconds, uint32_t time_microsecondsDelta, ImuFiltersBase& imu_filters, VehicleControllerBase& vehicle_controller, Debug& debug)
{
    _ahrs_data.delta_t = static_cast<float>(time_microsecondsDelta) * 0.000001F; // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    _ahrs_data.time_microseconds = time_microseconds;

#if defined(LIBRARY_STABILIZED_VEHICLE_USE_AHRS_TIME_CHECKS_FINE)
    const time_us32_t time0 = time_microseconds;
#endif

#if defined(LIBRARY_STABILIZED_VEHICLE_IMU_DOES_SENSOR_FUSION)

    // Some IMUs, eg the BNO085, do on-chip sensor fusion
    _ahrs_data.acc_gyro_rps.gyro_rps = _IMU.read_gyro_rps();
#if defined(LIBRARY_STABILIZED_VEHICLE_USE_AHRS_TIME_CHECKS_FINE)
    const time_us32_t time1 = time_us();
    _timeChecksMicroseconds[0] = time1 - time0;
    _timeChecksMicroseconds[1] = 0; // filter time set to zero, since filtering is as part of IMU sensor fusion
#endif
    _ahrs_data.orientation = _IMU.read_orientation();
#if defined(LIBRARY_STABILIZED_VEHICLE_USE_AHRS_TIME_CHECKS_FINE)
    const time_us32_t time3 = time_us();
    _timeChecksMicroseconds[2] = time3 - time1;
    _timeChecksMicroseconds[3] = 0; // fusion time set to zero, since fusion done on-chip
#endif

#else

    // if the data was read in the IMU interrupt service routine we can just get the data, rather than read it
    _ahrs_data.acc_gyro_rps = (_taskType == INTERRUPT_DRIVEN) ? _IMU.get_acc_gyro_rps() : _IMU.read_acc_gyro_rps();

    // Gyros are generally specified to +/- 2000 DPS, in a crash this limit can be exceeded and cause an overflow and a sign reversal in the output.
    check_gyro_overflow_z();

#if defined(LIBRARY_STABILIZED_VEHICLE_USE_AHRS_TIME_CHECKS_FINE)
    const time_us32_t time1 = time_us();
    _timeChecksMicroseconds[0] = time1 - time0;
#endif

    // apply the filters
    _ahrs_data.gyro_rps_unfiltered = _ahrs_data.acc_gyro_rps.gyro_rps; // unfiltered value saved for blackbox recording
    imu_filters.filter(_ahrs_data.acc_gyro_rps.gyro_rps, _ahrs_data.acc_gyro_rps.acc, _ahrs_data.delta_t, debug); // 15us, 207us

#if defined(LIBRARY_STABILIZED_VEHICLE_USE_AHRS_TIME_CHECKS_FINE)
    const time_us32_t time2 = time_us();
    _timeChecksMicroseconds[1] = time2 - time1;
#endif

    _ahrs_data.orientation = _sensor_fusion_filter.update_orientation(_ahrs_data.acc_gyro_rps.gyro_rps, _ahrs_data.acc_gyro_rps.acc, _ahrs_data.delta_t); // 15us, 140us

    if (_sensor_fusion_filter_is_initializing) {
        if (fusion_filter_has_converged(_ahrs_data.acc_gyro_rps.acc, _ahrs_data.orientation)) {
            _sensor_fusion_filter_is_initializing = false;
            vehicle_controller.set_sensor_fusion_filter_is_initializing(false);
        }
    }

#if defined(LIBRARY_STABILIZED_VEHICLE_USE_AHRS_TIME_CHECKS_FINE)
    const time_us32_t time3 = time_us();
    _timeChecksMicroseconds[2] = time3 - time2;
#endif


#endif // LIBRARY_STABILIZED_VEHICLE_IMU_DOES_SENSOR_FUSION

    return _ahrs_data;;
}

    /*!
Returns the AHRS data.
*/
ahrs_data_t Ahrs::get_ahrs_data_for_test() const
{
    return _ahrs_data;
}

bool Ahrs::fusion_filter_has_converged(const xyz_t& acc, const Quaternion& orientation)
{
    if ((_flags & SENSOR_FUSION_REQUIRES_INITIALIZATION) == 0) {
        return true;
    }
    static constexpr float twoDegreesInRadians = 2.0F * DEGREES_TO_RADIANS;

    // NOTE COORDINATE TRANSFORM: Madgwick filter uses Euler angles where roll is defined as rotation around the x-axis and pitch is rotation around the y-axis.
    // For the Self Balancing Robot, pitch is rotation around the x-axis and roll is rotation around the y-axis,
    // so SBR.roll = Madgwick.pitch and SPR.pitch = Madgwick.roll
    const float madgwickRollAngleRadians = orientation.calculate_roll_radians();
    const float accPitchAngleRadians = std::atan2(acc.y, acc.z);
    //const float accRollAngleRadians = std::atan2(-acc.x, sqrtf(acc.y*acc.y + acc.z*acc.z));

    //Serial.printf("acc:P%5.1f mag:P%5.1f         diff:%5.1f\r\n", accPitchAngleRadians/DEGREES_TO_RADIANS, madgwickRollAngleRadians/DEGREES_TO_RADIANS, fabsf(accPitchAngleRadians - madgwickRollAngleRadians)/DEGREES_TO_RADIANS);
    if (fabsf(accPitchAngleRadians - madgwickRollAngleRadians) < twoDegreesInRadians && accPitchAngleRadians != madgwickRollAngleRadians) {
        // the angles have converged to within 2 degrees, so we can reduce the gain.
        static constexpr float gyroMeasurementError = 0.615F; // corresponds to gyro measurement error of 15*2.7 degrees/second, as discussed by Madgwick
        _sensor_fusion_filter.set_free_parameters(gyroMeasurementError, 0.0F);
        return true;
    }
    return false;
}
