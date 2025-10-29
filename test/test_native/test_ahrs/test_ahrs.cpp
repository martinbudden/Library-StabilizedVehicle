#include "AHRS.h"
#include "IMU_FiltersNull.h"
#include "VehicleControllerBase.h"
#include <IMU_Null.h>
#include <SV_TelemetryData.h>
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
    virtual ~VehicleController() = default;
    VehicleController() : VehicleControllerBase(VehicleControllerBase::TYPE_NOT_SET, 0, 0) {}
    // VehicleController is not copyable or moveable
    VehicleController(const VehicleController&) = delete;
    VehicleController& operator=(const VehicleController&) = delete;
    VehicleController(VehicleController&&) = delete;
    VehicleController& operator=(VehicleController&&) = delete;
public:
    void outputToMixer(float deltaT, uint32_t tickCount, const VehicleControllerMessageQueue::queue_item_t& queueItem) override
        { (void)deltaT; (void)tickCount; (void)queueItem; }
    void updateOutputsUsingPIDs(const AHRS::imu_data_t& imuDataNED) override { (void)imuDataNED; }

    uint32_t getOutputPowerTimeMicroseconds() const override { return 0; }
    PIDF_uint16_t getPID_MSP(size_t index) const override { (void)index; return PIDF_uint16_t{}; }
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

    static constexpr float degreesToRadians = static_cast<float>(M_PI / 180.0);
    IMU_Base::accGyroRPS_t accGyroRPS {};
    float gyroZ {};

    accGyroRPS.gyroRPS.z = 10.0F * degreesToRadians;
    ahrs.setAccGyroRPS(accGyroRPS);
    ahrs.checkGyroOverflowZ();
    gyroZ = ahrs.getAhrsDataForTest().gyroRPS.z;
    TEST_ASSERT_EQUAL_FLOAT(accGyroRPS.gyroRPS.z, gyroZ);

    accGyroRPS.gyroRPS.z = -10.0F * degreesToRadians;
    ahrs.setAccGyroRPS(accGyroRPS);
    ahrs.checkGyroOverflowZ();
    gyroZ = ahrs.getAhrsDataForTest().gyroRPS.z;
    TEST_ASSERT_EQUAL_FLOAT(accGyroRPS.gyroRPS.z, gyroZ);

    accGyroRPS.gyroRPS.z = 1000.0F * degreesToRadians;
    ahrs.setAccGyroRPS(accGyroRPS);
    ahrs.checkGyroOverflowZ();
    gyroZ = ahrs.getAhrsDataForTest().gyroRPS.z;
    TEST_ASSERT_EQUAL_FLOAT(accGyroRPS.gyroRPS.z, gyroZ);

    accGyroRPS.gyroRPS.z = 1900.0F * degreesToRadians;
    ahrs.setAccGyroRPS(accGyroRPS);
    ahrs.checkGyroOverflowZ();
    gyroZ = ahrs.getAhrsDataForTest().gyroRPS.z;
    TEST_ASSERT_EQUAL_FLOAT(accGyroRPS.gyroRPS.z, gyroZ);

    accGyroRPS.gyroRPS.z = -1900.0F * degreesToRadians;
    ahrs.setAccGyroRPS(accGyroRPS);
    ahrs.checkGyroOverflowZ();
    gyroZ = ahrs.getAhrsDataForTest().gyroRPS.z;
    // we have had overflow, so gyro value should have been rejected
    TEST_ASSERT_EQUAL_FLOAT(1900.0F * degreesToRadians, gyroZ);

    accGyroRPS.gyroRPS.z = -1950.0F * degreesToRadians;
    ahrs.setAccGyroRPS(accGyroRPS);
    ahrs.checkGyroOverflowZ();
    gyroZ = ahrs.getAhrsDataForTest().gyroRPS.z;
    // overflow, so gyro value rejected
    TEST_ASSERT_EQUAL_FLOAT(1900.0F * degreesToRadians, gyroZ);

    accGyroRPS.gyroRPS.z = 1999.0F * degreesToRadians;
    ahrs.setAccGyroRPS(accGyroRPS);
    ahrs.checkGyroOverflowZ();
    gyroZ = ahrs.getAhrsDataForTest().gyroRPS.z;
    // overflow corrected, so gyro value accepted
    TEST_ASSERT_EQUAL_FLOAT(1999.0F * degreesToRadians, gyroZ);

    accGyroRPS.gyroRPS.z = -10.0F * degreesToRadians;
    ahrs.setAccGyroRPS(accGyroRPS);
    ahrs.checkGyroOverflowZ();
    gyroZ = ahrs.getAhrsDataForTest().gyroRPS.z;
    TEST_ASSERT_EQUAL_FLOAT(-10.0F * degreesToRadians, gyroZ);

    accGyroRPS.gyroRPS.z = -1499.0F * degreesToRadians;
    ahrs.setAccGyroRPS(accGyroRPS);
    ahrs.checkGyroOverflowZ();
    gyroZ = ahrs.getAhrsDataForTest().gyroRPS.z;
    TEST_ASSERT_EQUAL_FLOAT(-1499.0F * degreesToRadians, gyroZ);

    // large change, but below overflow threshold
    accGyroRPS.gyroRPS.z = 1499.0F * degreesToRadians;
    ahrs.setAccGyroRPS(accGyroRPS);
    ahrs.checkGyroOverflowZ();
    gyroZ = ahrs.getAhrsDataForTest().gyroRPS.z;
    TEST_ASSERT_EQUAL_FLOAT(1499.0F * degreesToRadians, gyroZ);

    // large change, above overflow threshold
    accGyroRPS.gyroRPS.z = -1502.0F * degreesToRadians;
    ahrs.setAccGyroRPS(accGyroRPS);
    ahrs.checkGyroOverflowZ();
    gyroZ = ahrs.getAhrsDataForTest().gyroRPS.z;
    TEST_ASSERT_EQUAL_FLOAT(1499.0F * degreesToRadians, gyroZ);

    // large change, but below overflow threshold
    accGyroRPS.gyroRPS.z = -1499.0F * degreesToRadians;
    ahrs.setAccGyroRPS(accGyroRPS);
    ahrs.checkGyroOverflowZ();
    gyroZ = ahrs.getAhrsDataForTest().gyroRPS.z;
    TEST_ASSERT_EQUAL_FLOAT(-1499.0F * degreesToRadians, gyroZ);

    // large change, above overflow threshold
    accGyroRPS.gyroRPS.z = 1502.0F * degreesToRadians;
    ahrs.setAccGyroRPS(accGyroRPS);
    ahrs.checkGyroOverflowZ();
    gyroZ = ahrs.getAhrsDataForTest().gyroRPS.z;
    TEST_ASSERT_EQUAL_FLOAT(-1499.0F * degreesToRadians, gyroZ);

    accGyroRPS.gyroRPS.z = -1900.0F * degreesToRadians;
    ahrs.setAccGyroRPS(accGyroRPS);
    ahrs.checkGyroOverflowZ();
    gyroZ = ahrs.getAhrsDataForTest().gyroRPS.z;
    TEST_ASSERT_EQUAL_FLOAT(-1900.0F * degreesToRadians, gyroZ);

    accGyroRPS.gyroRPS.z = 1950.0F * degreesToRadians;
    ahrs.setAccGyroRPS(accGyroRPS);
    ahrs.checkGyroOverflowZ();
    gyroZ = ahrs.getAhrsDataForTest().gyroRPS.z;
    // overflow, so gyro value rejected
    TEST_ASSERT_EQUAL_FLOAT(-1900.0F * degreesToRadians, gyroZ);

    accGyroRPS.gyroRPS.z = 1980.0F * degreesToRadians;
    ahrs.setAccGyroRPS(accGyroRPS);
    ahrs.checkGyroOverflowZ();
    gyroZ = ahrs.getAhrsDataForTest().gyroRPS.z;
    // overflow, so gyro value rejected
    TEST_ASSERT_EQUAL_FLOAT(-1900.0F * degreesToRadians, gyroZ);

    accGyroRPS.gyroRPS.z = -1990.0F * degreesToRadians;
    ahrs.setAccGyroRPS(accGyroRPS);
    ahrs.checkGyroOverflowZ();
    gyroZ = ahrs.getAhrsDataForTest().gyroRPS.z;
    // overflow corrected, so gyro value accepted
    TEST_ASSERT_EQUAL_FLOAT(-1990.0F * degreesToRadians, gyroZ);
}

void test_sv_telemetry_data()
{
    enum { MAX_TD_PACKET_SIZE = 250 };
    static_assert(sizeof(TD_RESERVED) <= MAX_TD_PACKET_SIZE);
    static_assert(sizeof(TD_MINIMAL) <= MAX_TD_PACKET_SIZE);
    static_assert(sizeof(TD_TASK_INTERVALS) <= MAX_TD_PACKET_SIZE);
    static_assert(sizeof(TD_TASK_INTERVALS_EXTENDED) <= MAX_TD_PACKET_SIZE);
    static_assert(sizeof(TD_AHRS) <= MAX_TD_PACKET_SIZE);
    static_assert(sizeof(TD_PID) <= MAX_TD_PACKET_SIZE);
    static_assert(sizeof(TD_PID_EXTENDED) <= MAX_TD_PACKET_SIZE);
    static_assert(sizeof(TD_FC_QUADCOPTER) <= MAX_TD_PACKET_SIZE);

    enum { MAX_TD_MSP_PACKET_SIZE = 260 };
    static_assert(sizeof(TD_MSP) <= MAX_TD_MSP_PACKET_SIZE);
    static_assert(sizeof(TD_BLACKBOX_E) <= MAX_TD_MSP_PACKET_SIZE);
    static_assert(sizeof(TD_BLACKBOX_I) <= MAX_TD_MSP_PACKET_SIZE);
    static_assert(sizeof(TD_BLACKBOX_P) <= MAX_TD_MSP_PACKET_SIZE);
    static_assert(sizeof(TD_BLACKBOX_S) <= MAX_TD_MSP_PACKET_SIZE);

    static_assert(TD_TASK_INTERVALS_EXTENDED::TIME_CHECKS_COUNT == AHRS::TIME_CHECKS_COUNT);
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,misc-const-correctness,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_ahrs);
    RUN_TEST(test_gyro_overflow);
    RUN_TEST(test_sv_telemetry_data);

    UNITY_END();
}
