#include "AHRS.h"
#include "VehicleControllerBase.h"
#include <imu_null.h>
#include <sensor_fusion.h>

#include <unity.h>

void setUp()
{
}

void tearDown()
{
}
class VehicleController : public VehicleControllerBase {
public:
    VehicleController() : VehicleControllerBase(VehicleControllerBase::TYPE_NOT_SET, 0, 0) {}
public:
    void update_outputs_using_pids(const ahrs_data_t& ahrsData, AhrsMessageQueue& ahrs_message_queue, MotorMixerMessageQueue& motor_mixer_message_queue) override
        { (void)ahrsData; (void)ahrs_message_queue; (void)motor_mixer_message_queue; }
};

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,misc-const-correctness,readability-magic-numbers)
void test_ahrs()
{
    MadgwickFilter sensor_fusion_filter;
    ImuNull imu(ImuBase::XPOS_YPOS_ZPOS);
    Ahrs ahrs(Ahrs::TIMER_DRIVEN, sensor_fusion_filter, imu);
    (void)ahrs;
}

void test_gyro_overflow()
{
    MadgwickFilter sensor_fusion_filter;
    ImuNull imu(ImuBase::XPOS_YPOS_ZPOS); // NOLINT(misc-const-correctness) false positive
    Ahrs ahrs(Ahrs::TIMER_DRIVEN, sensor_fusion_filter, imu);

    static constexpr float DEGREES_TO_RADIANS = static_cast<float>(M_PI / 180.0);
    acc_gyro_rps_t acc_gyro_rps {};
    float gyroZ {};

    acc_gyro_rps.gyro_rps.z = 10.0F * DEGREES_TO_RADIANS;
    ahrs.set_acc_gyro_rps(acc_gyro_rps);
    ahrs.check_gyro_overflow_z();
    gyroZ = ahrs.get_ahrs_data_for_test().acc_gyro_rps.gyro_rps.z;
    TEST_ASSERT_EQUAL_FLOAT(acc_gyro_rps.gyro_rps.z, gyroZ);

    acc_gyro_rps.gyro_rps.z = -10.0F * DEGREES_TO_RADIANS;
    ahrs.set_acc_gyro_rps(acc_gyro_rps);
    ahrs.check_gyro_overflow_z();
    gyroZ = ahrs.get_ahrs_data_for_test().acc_gyro_rps.gyro_rps.z;
    TEST_ASSERT_EQUAL_FLOAT(acc_gyro_rps.gyro_rps.z, gyroZ);

    acc_gyro_rps.gyro_rps.z = 1000.0F * DEGREES_TO_RADIANS;
    ahrs.set_acc_gyro_rps(acc_gyro_rps);
    ahrs.check_gyro_overflow_z();
    gyroZ = ahrs.get_ahrs_data_for_test().acc_gyro_rps.gyro_rps.z;
    TEST_ASSERT_EQUAL_FLOAT(acc_gyro_rps.gyro_rps.z, gyroZ);

    acc_gyro_rps.gyro_rps.z = 1900.0F * DEGREES_TO_RADIANS;
    ahrs.set_acc_gyro_rps(acc_gyro_rps);
    ahrs.check_gyro_overflow_z();
    gyroZ = ahrs.get_ahrs_data_for_test().acc_gyro_rps.gyro_rps.z;
    TEST_ASSERT_EQUAL_FLOAT(acc_gyro_rps.gyro_rps.z, gyroZ);

    acc_gyro_rps.gyro_rps.z = -1900.0F * DEGREES_TO_RADIANS;
    ahrs.set_acc_gyro_rps(acc_gyro_rps);
    ahrs.check_gyro_overflow_z();
    gyroZ = ahrs.get_ahrs_data_for_test().acc_gyro_rps.gyro_rps.z;
    // we have had overflow, so gyro value should have been rejected
    TEST_ASSERT_EQUAL_FLOAT(1900.0F * DEGREES_TO_RADIANS, gyroZ);

    acc_gyro_rps.gyro_rps.z = -1950.0F * DEGREES_TO_RADIANS;
    ahrs.set_acc_gyro_rps(acc_gyro_rps);
    ahrs.check_gyro_overflow_z();
    gyroZ = ahrs.get_ahrs_data_for_test().acc_gyro_rps.gyro_rps.z;
    // overflow, so gyro value rejected
    TEST_ASSERT_EQUAL_FLOAT(1900.0F * DEGREES_TO_RADIANS, gyroZ);

    acc_gyro_rps.gyro_rps.z = 1999.0F * DEGREES_TO_RADIANS;
    ahrs.set_acc_gyro_rps(acc_gyro_rps);
    ahrs.check_gyro_overflow_z();
    gyroZ = ahrs.get_ahrs_data_for_test().acc_gyro_rps.gyro_rps.z;
    // overflow corrected, so gyro value accepted
    TEST_ASSERT_EQUAL_FLOAT(1999.0F * DEGREES_TO_RADIANS, gyroZ);

    acc_gyro_rps.gyro_rps.z = -10.0F * DEGREES_TO_RADIANS;
    ahrs.set_acc_gyro_rps(acc_gyro_rps);
    ahrs.check_gyro_overflow_z();
    gyroZ = ahrs.get_ahrs_data_for_test().acc_gyro_rps.gyro_rps.z;
    TEST_ASSERT_EQUAL_FLOAT(-10.0F * DEGREES_TO_RADIANS, gyroZ);

    acc_gyro_rps.gyro_rps.z = -1499.0F * DEGREES_TO_RADIANS;
    ahrs.set_acc_gyro_rps(acc_gyro_rps);
    ahrs.check_gyro_overflow_z();
    gyroZ = ahrs.get_ahrs_data_for_test().acc_gyro_rps.gyro_rps.z;
    TEST_ASSERT_EQUAL_FLOAT(-1499.0F * DEGREES_TO_RADIANS, gyroZ);

    // large change, but below overflow threshold
    acc_gyro_rps.gyro_rps.z = 1499.0F * DEGREES_TO_RADIANS;
    ahrs.set_acc_gyro_rps(acc_gyro_rps);
    ahrs.check_gyro_overflow_z();
    gyroZ = ahrs.get_ahrs_data_for_test().acc_gyro_rps.gyro_rps.z;
    TEST_ASSERT_EQUAL_FLOAT(1499.0F * DEGREES_TO_RADIANS, gyroZ);

    // large change, above overflow threshold
    acc_gyro_rps.gyro_rps.z = -1502.0F * DEGREES_TO_RADIANS;
    ahrs.set_acc_gyro_rps(acc_gyro_rps);
    ahrs.check_gyro_overflow_z();
    gyroZ = ahrs.get_ahrs_data_for_test().acc_gyro_rps.gyro_rps.z;
    TEST_ASSERT_EQUAL_FLOAT(1499.0F * DEGREES_TO_RADIANS, gyroZ);

    // large change, but below overflow threshold
    acc_gyro_rps.gyro_rps.z = -1499.0F * DEGREES_TO_RADIANS;
    ahrs.set_acc_gyro_rps(acc_gyro_rps);
    ahrs.check_gyro_overflow_z();
    gyroZ = ahrs.get_ahrs_data_for_test().acc_gyro_rps.gyro_rps.z;
    TEST_ASSERT_EQUAL_FLOAT(-1499.0F * DEGREES_TO_RADIANS, gyroZ);

    // large change, above overflow threshold
    acc_gyro_rps.gyro_rps.z = 1502.0F * DEGREES_TO_RADIANS;
    ahrs.set_acc_gyro_rps(acc_gyro_rps);
    ahrs.check_gyro_overflow_z();
    gyroZ = ahrs.get_ahrs_data_for_test().acc_gyro_rps.gyro_rps.z;
    TEST_ASSERT_EQUAL_FLOAT(-1499.0F * DEGREES_TO_RADIANS, gyroZ);

    acc_gyro_rps.gyro_rps.z = -1900.0F * DEGREES_TO_RADIANS;
    ahrs.set_acc_gyro_rps(acc_gyro_rps);
    ahrs.check_gyro_overflow_z();
    gyroZ = ahrs.get_ahrs_data_for_test().acc_gyro_rps.gyro_rps.z;
    TEST_ASSERT_EQUAL_FLOAT(-1900.0F * DEGREES_TO_RADIANS, gyroZ);

    acc_gyro_rps.gyro_rps.z = 1950.0F * DEGREES_TO_RADIANS;
    ahrs.set_acc_gyro_rps(acc_gyro_rps);
    ahrs.check_gyro_overflow_z();
    gyroZ = ahrs.get_ahrs_data_for_test().acc_gyro_rps.gyro_rps.z;
    // overflow, so gyro value rejected
    TEST_ASSERT_EQUAL_FLOAT(-1900.0F * DEGREES_TO_RADIANS, gyroZ);

    acc_gyro_rps.gyro_rps.z = 1980.0F * DEGREES_TO_RADIANS;
    ahrs.set_acc_gyro_rps(acc_gyro_rps);
    ahrs.check_gyro_overflow_z();
    gyroZ = ahrs.get_ahrs_data_for_test().acc_gyro_rps.gyro_rps.z;
    // overflow, so gyro value rejected
    TEST_ASSERT_EQUAL_FLOAT(-1900.0F * DEGREES_TO_RADIANS, gyroZ);

    acc_gyro_rps.gyro_rps.z = -1990.0F * DEGREES_TO_RADIANS;
    ahrs.set_acc_gyro_rps(acc_gyro_rps);
    ahrs.check_gyro_overflow_z();
    gyroZ = ahrs.get_ahrs_data_for_test().acc_gyro_rps.gyro_rps.z;
    // overflow corrected, so gyro value accepted
    TEST_ASSERT_EQUAL_FLOAT(-1990.0F * DEGREES_TO_RADIANS, gyroZ);
}

// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,misc-const-correctness,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_ahrs);
    RUN_TEST(test_gyro_overflow);

    UNITY_END();
}
