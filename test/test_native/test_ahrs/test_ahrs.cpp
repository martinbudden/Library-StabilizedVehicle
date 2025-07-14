#include "AHRS.h"
#include "IMU_FiltersNull.h"
#include <IMU_Null.h>
#include <SV_TelemetryData.h>
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

void test_sv_telemetry_data()
{
    enum { MAX_TD_PACKET_SIZE = 250 };
    static_assert(sizeof(TD_RESERVED) <= MAX_TD_PACKET_SIZE);
    static_assert(sizeof(TD_MINIMAL) <= MAX_TD_PACKET_SIZE);
    static_assert(sizeof(TD_TASK_INTERVALS) <= MAX_TD_PACKET_SIZE);
    static_assert(sizeof(TD_TASK_INTERVALS_EXTENDED) <= MAX_TD_PACKET_SIZE);
    static_assert(sizeof(TD_AHRS) <= MAX_TD_PACKET_SIZE);
    static_assert(sizeof(TD_PIDS) <= MAX_TD_PACKET_SIZE);
    static_assert(sizeof(TD_PIDS_EXTENDED) <= MAX_TD_PACKET_SIZE);
    static_assert(sizeof(TD_FC_QUADCOPTER) <= MAX_TD_PACKET_SIZE);

    enum { MAX_TD_MSP_PACKET_SIZE = 260 };
    static_assert(sizeof(TD_MSP) <= MAX_TD_MSP_PACKET_SIZE);
    static_assert(sizeof(TD_BLACKBOX_E) <= MAX_TD_MSP_PACKET_SIZE);
    static_assert(sizeof(TD_BLACKBOX_I) <= MAX_TD_MSP_PACKET_SIZE);
    static_assert(sizeof(TD_BLACKBOX_P) <= MAX_TD_MSP_PACKET_SIZE);
    static_assert(sizeof(TD_BLACKBOX_S) <= MAX_TD_MSP_PACKET_SIZE);

    static_assert(TD_TASK_INTERVALS_EXTENDED::TIME_CHECKS_COUNT == AHRS::TIME_CHECKS_COUNT);
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_ahrs);
    RUN_TEST(test_sv_telemetry_data);

    UNITY_END();
}
