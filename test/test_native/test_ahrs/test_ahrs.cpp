#include "AHRS.h"
#include "IMU_FiltersNull.h"
#include <IMU_Null.h>
#include <SensorFusion.h>
#include <unity.h>

#if !defined(AHRS_TASK_INTERVAL_MICROSECONDS)
enum { AHRS_TASK_INTERVAL_MICROSECONDS = 5000 };
#endif


void setUp()
{
}

void tearDown()
{
}

void test_ahrs()
{
    MadgwickFilter sensorFusionFilter; // NOLINT(misc-const-correctness)
    IMU_Null imu(IMU_Base::XPOS_YPOS_ZPOS); // NOLINT(misc-const-correctness) false positive
    IMU_FiltersNull imuFilters; // NOLINT(misc-const-correctness) false positive
    AHRS ahrs(AHRS_TASK_INTERVAL_MICROSECONDS, sensorFusionFilter, imu, imuFilters);

    TEST_ASSERT_TRUE(ahrs.sensorFusionFilterIsInitializing()); // initializing should be set on construction
    ahrs.setSensorFusionInitializing(true);
    TEST_ASSERT_TRUE(ahrs.sensorFusionFilterIsInitializing());
    ahrs.setSensorFusionInitializing(false);
    TEST_ASSERT_FALSE(ahrs.sensorFusionFilterIsInitializing());
    ahrs.setSensorFusionInitializing(true);
    TEST_ASSERT_TRUE(ahrs.sensorFusionFilterIsInitializing());
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_ahrs);

    UNITY_END();
}
