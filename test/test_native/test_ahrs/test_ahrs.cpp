#include "AHRS.h"
#include "IMU_FiltersNull.h"
#include "VehicleControllerBase.h"
#include <IMU_Null.h>
#include <SensorFusion.h>

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
    void outputToMixer(float deltaT, uint32_t tickCount, const VehicleControllerMessageQueue::queue_item_t& queueItem) override
        { (void)deltaT; (void)tickCount; (void)queueItem; }
    void updateOutputsUsingPIDs(const ahrs_data_t& ahrsData) override { (void)ahrsData; }
};

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,misc-const-correctness,readability-magic-numbers)
void test_ahrs()
{
    VehicleController vehicleController;
    MadgwickFilter sensorFusionFilter;
    IMU_Null imu(IMU_Base::XPOS_YPOS_ZPOS);
    IMU_FiltersNull imuFilters;
    AHRS ahrs(AHRS::TIMER_DRIVEN, vehicleController, sensorFusionFilter, imu, imuFilters);

    TEST_ASSERT_TRUE(ahrs.sensorFusionFilterIsInitializing()); // initializing should be set on construction
    ahrs.setSensorFusionInitializing(true);
    TEST_ASSERT_TRUE(ahrs.sensorFusionFilterIsInitializing());
    ahrs.setSensorFusionInitializing(false);
    TEST_ASSERT_FALSE(ahrs.sensorFusionFilterIsInitializing());
    ahrs.setSensorFusionInitializing(true);
    TEST_ASSERT_TRUE(ahrs.sensorFusionFilterIsInitializing());
}

void test_gyro_overflow()
{
    VehicleController vehicleController;
    MadgwickFilter sensorFusionFilter;
    IMU_Null imu(IMU_Base::XPOS_YPOS_ZPOS); // NOLINT(misc-const-correctness) false positive
    IMU_FiltersNull imuFilters; // NOLINT(misc-const-correctness) false positive
    AHRS ahrs(AHRS::TIMER_DRIVEN, vehicleController, sensorFusionFilter, imu, imuFilters);

    static constexpr float DEGREES_TO_RADIANS = static_cast<float>(M_PI / 180.0);
    acc_gyro_rps_t accGyroRPS {};
    float gyroZ {};

    accGyroRPS.gyroRPS.z = 10.0F * DEGREES_TO_RADIANS;
    ahrs.setAccGyroRPS(accGyroRPS);
    ahrs.checkGyroOverflowZ();
    gyroZ = ahrs.getAhrsDataForTest().accGyroRPS.gyroRPS.z;
    TEST_ASSERT_EQUAL_FLOAT(accGyroRPS.gyroRPS.z, gyroZ);

    accGyroRPS.gyroRPS.z = -10.0F * DEGREES_TO_RADIANS;
    ahrs.setAccGyroRPS(accGyroRPS);
    ahrs.checkGyroOverflowZ();
    gyroZ = ahrs.getAhrsDataForTest().accGyroRPS.gyroRPS.z;
    TEST_ASSERT_EQUAL_FLOAT(accGyroRPS.gyroRPS.z, gyroZ);

    accGyroRPS.gyroRPS.z = 1000.0F * DEGREES_TO_RADIANS;
    ahrs.setAccGyroRPS(accGyroRPS);
    ahrs.checkGyroOverflowZ();
    gyroZ = ahrs.getAhrsDataForTest().accGyroRPS.gyroRPS.z;
    TEST_ASSERT_EQUAL_FLOAT(accGyroRPS.gyroRPS.z, gyroZ);

    accGyroRPS.gyroRPS.z = 1900.0F * DEGREES_TO_RADIANS;
    ahrs.setAccGyroRPS(accGyroRPS);
    ahrs.checkGyroOverflowZ();
    gyroZ = ahrs.getAhrsDataForTest().accGyroRPS.gyroRPS.z;
    TEST_ASSERT_EQUAL_FLOAT(accGyroRPS.gyroRPS.z, gyroZ);

    accGyroRPS.gyroRPS.z = -1900.0F * DEGREES_TO_RADIANS;
    ahrs.setAccGyroRPS(accGyroRPS);
    ahrs.checkGyroOverflowZ();
    gyroZ = ahrs.getAhrsDataForTest().accGyroRPS.gyroRPS.z;
    // we have had overflow, so gyro value should have been rejected
    TEST_ASSERT_EQUAL_FLOAT(1900.0F * DEGREES_TO_RADIANS, gyroZ);

    accGyroRPS.gyroRPS.z = -1950.0F * DEGREES_TO_RADIANS;
    ahrs.setAccGyroRPS(accGyroRPS);
    ahrs.checkGyroOverflowZ();
    gyroZ = ahrs.getAhrsDataForTest().accGyroRPS.gyroRPS.z;
    // overflow, so gyro value rejected
    TEST_ASSERT_EQUAL_FLOAT(1900.0F * DEGREES_TO_RADIANS, gyroZ);

    accGyroRPS.gyroRPS.z = 1999.0F * DEGREES_TO_RADIANS;
    ahrs.setAccGyroRPS(accGyroRPS);
    ahrs.checkGyroOverflowZ();
    gyroZ = ahrs.getAhrsDataForTest().accGyroRPS.gyroRPS.z;
    // overflow corrected, so gyro value accepted
    TEST_ASSERT_EQUAL_FLOAT(1999.0F * DEGREES_TO_RADIANS, gyroZ);

    accGyroRPS.gyroRPS.z = -10.0F * DEGREES_TO_RADIANS;
    ahrs.setAccGyroRPS(accGyroRPS);
    ahrs.checkGyroOverflowZ();
    gyroZ = ahrs.getAhrsDataForTest().accGyroRPS.gyroRPS.z;
    TEST_ASSERT_EQUAL_FLOAT(-10.0F * DEGREES_TO_RADIANS, gyroZ);

    accGyroRPS.gyroRPS.z = -1499.0F * DEGREES_TO_RADIANS;
    ahrs.setAccGyroRPS(accGyroRPS);
    ahrs.checkGyroOverflowZ();
    gyroZ = ahrs.getAhrsDataForTest().accGyroRPS.gyroRPS.z;
    TEST_ASSERT_EQUAL_FLOAT(-1499.0F * DEGREES_TO_RADIANS, gyroZ);

    // large change, but below overflow threshold
    accGyroRPS.gyroRPS.z = 1499.0F * DEGREES_TO_RADIANS;
    ahrs.setAccGyroRPS(accGyroRPS);
    ahrs.checkGyroOverflowZ();
    gyroZ = ahrs.getAhrsDataForTest().accGyroRPS.gyroRPS.z;
    TEST_ASSERT_EQUAL_FLOAT(1499.0F * DEGREES_TO_RADIANS, gyroZ);

    // large change, above overflow threshold
    accGyroRPS.gyroRPS.z = -1502.0F * DEGREES_TO_RADIANS;
    ahrs.setAccGyroRPS(accGyroRPS);
    ahrs.checkGyroOverflowZ();
    gyroZ = ahrs.getAhrsDataForTest().accGyroRPS.gyroRPS.z;
    TEST_ASSERT_EQUAL_FLOAT(1499.0F * DEGREES_TO_RADIANS, gyroZ);

    // large change, but below overflow threshold
    accGyroRPS.gyroRPS.z = -1499.0F * DEGREES_TO_RADIANS;
    ahrs.setAccGyroRPS(accGyroRPS);
    ahrs.checkGyroOverflowZ();
    gyroZ = ahrs.getAhrsDataForTest().accGyroRPS.gyroRPS.z;
    TEST_ASSERT_EQUAL_FLOAT(-1499.0F * DEGREES_TO_RADIANS, gyroZ);

    // large change, above overflow threshold
    accGyroRPS.gyroRPS.z = 1502.0F * DEGREES_TO_RADIANS;
    ahrs.setAccGyroRPS(accGyroRPS);
    ahrs.checkGyroOverflowZ();
    gyroZ = ahrs.getAhrsDataForTest().accGyroRPS.gyroRPS.z;
    TEST_ASSERT_EQUAL_FLOAT(-1499.0F * DEGREES_TO_RADIANS, gyroZ);

    accGyroRPS.gyroRPS.z = -1900.0F * DEGREES_TO_RADIANS;
    ahrs.setAccGyroRPS(accGyroRPS);
    ahrs.checkGyroOverflowZ();
    gyroZ = ahrs.getAhrsDataForTest().accGyroRPS.gyroRPS.z;
    TEST_ASSERT_EQUAL_FLOAT(-1900.0F * DEGREES_TO_RADIANS, gyroZ);

    accGyroRPS.gyroRPS.z = 1950.0F * DEGREES_TO_RADIANS;
    ahrs.setAccGyroRPS(accGyroRPS);
    ahrs.checkGyroOverflowZ();
    gyroZ = ahrs.getAhrsDataForTest().accGyroRPS.gyroRPS.z;
    // overflow, so gyro value rejected
    TEST_ASSERT_EQUAL_FLOAT(-1900.0F * DEGREES_TO_RADIANS, gyroZ);

    accGyroRPS.gyroRPS.z = 1980.0F * DEGREES_TO_RADIANS;
    ahrs.setAccGyroRPS(accGyroRPS);
    ahrs.checkGyroOverflowZ();
    gyroZ = ahrs.getAhrsDataForTest().accGyroRPS.gyroRPS.z;
    // overflow, so gyro value rejected
    TEST_ASSERT_EQUAL_FLOAT(-1900.0F * DEGREES_TO_RADIANS, gyroZ);

    accGyroRPS.gyroRPS.z = -1990.0F * DEGREES_TO_RADIANS;
    ahrs.setAccGyroRPS(accGyroRPS);
    ahrs.checkGyroOverflowZ();
    gyroZ = ahrs.getAhrsDataForTest().accGyroRPS.gyroRPS.z;
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
