# pragma once

/*!
Packet definitions of Telemetry Data (TD) useful to any Stabilized Vehicle.
*/
#include <array>
#include <xyz_type.h>

#pragma pack(push, 1)
/*!
Telemetry data type 0 is reserved for future use.
*/
struct TD_RESERVED {
    enum { TYPE = 0 };
    uint32_t id {0};

    uint8_t type {TYPE};
    uint8_t len {sizeof(TD_RESERVED)}; //!< length of whole packet, ie sizeof(TD_RESERVED)
    uint8_t subType {0};
    uint8_t sequenceNumber {0};
};

/*!
Minimal sized packet.
*/
struct TD_MINIMAL {
    enum { TYPE = 1 };
    uint32_t id {0};

    uint8_t type {TYPE};
    uint8_t len {sizeof(TD_MINIMAL)}; //!< length of whole packet, ie sizeof(TD_MINIMAL)
    uint8_t subType {0};
    uint8_t sequenceNumber {0};
};

/*!
Packet for the the transmission of AHRS, Vehicle Controller, and MAIN task tick intervals and timings;
*/
struct TD_TASK_INTERVALS {
    enum { TYPE = 2 };
    uint32_t id {0};

    uint8_t type {TYPE};
    uint8_t len {sizeof(TD_TASK_INTERVALS)}; //!< length of whole packet, ie sizeof(TD_TASK_INTERVALS)
    uint8_t subType {0};
    uint8_t sequenceNumber {0};

    struct data_t {
        uint8_t mainTaskIntervalTicks; //!< tick interval of the MAIN_LOOP_TASK
        uint8_t ahrsTaskIntervalTicks; //!< tick interval of the AHRS_TASK
        uint8_t vcTaskIntervalTicks; //!< tick interval of the Vehicle Controller task
        uint8_t transceiverTickCountDelta; //<<! tick interval of the ESP_NOW transceiver
    };
    data_t data {};
};

/*!
Packet for the the transmission of AHRS, Vehicle Controller, and MAIN tick intervals and timings;
*/
struct TD_TASK_INTERVALS_EXTENDED {
    enum { TYPE = 3 };
    uint32_t id {0};

    uint8_t type {TYPE};
    uint8_t len {sizeof(TD_TASK_INTERVALS_EXTENDED)}; //!< length of whole packet, ie sizeof(TD_TASK_INTERVALS)
    uint8_t subType {0};
    uint8_t sequenceNumber {0};

    enum { TIME_CHECKS_COUNT = 8 };
    struct data_t {
        uint8_t mainTaskIntervalTicks; //!< tick interval of the MAIN_LOOP_TASK
        uint8_t ahrsTaskIntervalTicks; //!< tick interval of the AHRS_TASK
        uint8_t vcTaskIntervalTicks; //!< tick interval of the Vehicle Controller task
        uint8_t transceiverTickCountDelta; //<<! tick interval of the ESP_NOW transceiver
        std::array<uint16_t, TIME_CHECKS_COUNT> ahrsTimeChecksMicroseconds;

        uint16_t ahrsTaskIntervalMicroseconds; //!< execution interval of AHRS_TASK in microseconds
        uint16_t vcTaskIntervalMicroseconds; //!< execution interval of the Vehicle Controller task in microseconds
        uint16_t vcOutputPowerTimeMicroseconds; //!< time taken to set the Vehicle output power

        uint16_t receiverDroppedPacketCount; //!< the number of packets dropped by the receiver
    };
    data_t data {};
};

/*!
Packet for the transmission of AHRS telemetry data.
*/
struct TD_AHRS {
    enum { TYPE = 4 };
    uint32_t id {0};

    uint8_t type {TYPE};
    uint8_t len {sizeof(TD_AHRS)}; //!< length of whole packet, ie sizeof(TD_AHRS)
    uint8_t subType {0};
    uint8_t sequenceNumber {0};

    uint8_t taskIntervalTicks {0}; //!< interval of the AHRS task, in ticks
    enum : uint8_t { IMU_AUTO_CALIBRATES = 0x01, SENSOR_FUSION_REQUIRES_INITIALIZATION = 0x02, SENSOR_FUSION_IS_INITIALIZING = 0x04};
    uint8_t flags {0};
    uint16_t reserved {0};

    struct xyz_int16_t {
        int16_t x;
        int16_t y;
        int16_t z;
    };
    struct data_t {
        float roll; //!< estimated roll value calculated by Sensor Fusion Filter
        float pitch; //!< estimated pitch value calculated by Sensor Fusion Filter
        float yaw; //!< estimated yaw value calculated by Sensor Fusion Filter
        xyz_t gyroRPS; //!< gyro outputs from IMU
        xyz_t acc; //!< acceleration outputs from IMU
        xyz_int16_t gyroOffset;
        xyz_int16_t accOffset;
    };
    data_t data {};
};

/*!
Packet for the the transmission of PID constants, setpoints, and some general-purpose parameters, to enable remote tuning.
*/
struct TD_PID {
    enum { TYPE = 5 };
    uint32_t id {0};

    uint8_t type {TYPE};
    uint8_t len {sizeof(TD_PID)}; //!< length of whole packet, ie sizeof(TD_PID)
    uint8_t subType {0};
    uint8_t sequenceNumber {0};

    struct PIDFS_t {
        uint16_t kp;
        uint16_t ki;
        uint16_t kd;
        uint16_t kf;
        uint16_t ks;
    };
    enum { TYPE_NOT_SET= 0, SELF_BALANCING_ROBOT = 1, AIRCRAFT = 2 };
    enum { MAX_PID_COUNT = 8 };  // allow up to 8 PIDs
    struct data_t {
        uint8_t pidCount;
        uint8_t pidProfile;
        uint8_t vehicleType;
        uint8_t controlMode;
        // general use parameters
        float f0; // typically used for pitchBalanceAngleDegrees
        float f1;
        std::array<PIDFS_t, MAX_PID_COUNT> pids;
    };
    data_t data {};
};


struct TD_PID_EXTENDED {
    enum { TYPE = 6 };
    uint32_t id {0};

    uint8_t type {TYPE};
    uint8_t len {sizeof(TD_PID_EXTENDED)}; //!< length of whole packet, ie sizeof(TD_PID_EXTENDED)
    uint8_t subType {0};
    uint8_t sequenceNumber {0};

    enum vehicle_type_e { SELF_BALANCING_ROBOT = 0, AIRCRAFT = 1 };
    enum { MAX_PID_COUNT = 12 };  // allow up to 12 PIDs

    struct PIDFS_t {
        uint16_t kp;
        uint16_t ki;
        uint16_t kd;
        uint16_t kf;
        uint16_t ks;
    };
    struct SPID_t {
        float setpoint;
        PIDFS_t pid;
    };
    struct data_t {
        uint8_t pidCount;
        uint8_t pidProfile;
        uint8_t vehicleType;
        uint8_t controlMode;
        // general use parameters
        float f0; // typically used for pitchBalanceAngleDegrees
        float f1;
        float f2;
        float f3;
        std::array<SPID_t, MAX_PID_COUNT> spids;
    };
    data_t data {};
};

/*!
Packet for the the transmission of debug data
*/
struct TD_DEBUG {
    enum { TYPE = 7 };
    uint32_t id {0};

    uint8_t type {TYPE};
    uint8_t len {sizeof(TD_DEBUG)}; //!< length of whole packet, ie sizeof(TD_DEBUG)
    uint8_t subType {0};
    uint8_t sequenceNumber {0};

    enum { VALUE_COUNT = 8 };
    struct data_t {
        std::array<int16_t, VALUE_COUNT> values;
        uint16_t mode;
    };
    data_t data {};
};



/*!
TYPE RANGE of 30-40 reserved for MultiWii Serial Protocol (MSP)

MSP V1 packet is of the form:

3 bytes header: two start bytes $M followed by message direction (< or >) or the error message indicator (!).
< - from the flight controller (FC →),
> - to the flight controller (→ FC).
! - Error Message.

one byte payload length
one byte message type
payload
checksum - XOR of the size, type, and payload bytes.

The checksum of a request (ie a message with no payload) equals the type.

NOTE: enough space is reserved for a full-size MSP packet, this is more than
can be accommodated in an ESP_NOW packet, so size payloadSize must be checked
before the packet is sent over ESP_NOW.
*/

struct TD_MSP {
    enum { TYPE = 36 }; // '$'
    uint32_t id {0};

    enum { MAX_MSP_DATA_SIZE = 256 };
    enum { MSP_HEADER_AND_CHECKSUM_SIZE = 6 };
    enum { 
        PACKET_OVERHEAD = sizeof(id) + MSP_HEADER_AND_CHECKSUM_SIZE,
        ESP_NOW_MAX_DATA_SIZE = 250,
        MAX_PAYLOAD_SIZE_FOR_ESP = ESP_NOW_MAX_DATA_SIZE - PACKET_OVERHEAD
    };

    struct msp_t {
        uint8_t headerDollar;
        uint8_t headerM;
        uint8_t headerDirection;
        uint8_t payloadSize;
        uint8_t messageType;
        std::array<uint8_t, MAX_MSP_DATA_SIZE - 5> payload; // includes checksum
    };
    union u {
        msp_t msp;
        std::array<uint8_t, MAX_MSP_DATA_SIZE> buffer;
    };
    u data {};
};

/*!
TYPE RANGE of 40-59 reserved for multi-rotors
*/

/*!
Packet for the transmission of FlightController telemetry data for a Quadcopter.
*/
struct TD_FC_QUADCOPTER {
    enum { TYPE = 40 };
    enum { MOTOR_COUNT = 4 };

    uint32_t id {0};
    uint8_t type {TYPE};
    uint8_t len {sizeof(TD_FC_QUADCOPTER)}; //!< length of whole packet, ie sizeof(TD_FC_QUADCOPTER)
    uint8_t subType {0};
    uint8_t sequenceNumber {0};

    uint16_t taskIntervalTicks {0}; //!< tick interval of the FC task
    enum : uint16_t { MOTORS_ON_FLAG = 0x8000, CONTROL_MODE_MASK = 0x00FF };
    uint16_t flags {0};

    struct power_rpm_t {
        float power;
        int32_t rpm;
    };
    struct data_t {
        std::array<power_rpm_t, MOTOR_COUNT> motors;
    };
    data_t data {};
};

/*!
TYPE RANGE of 60-90 reserved for blackbox
This includes ASCII 'A' (65) to ASCII 'Z' (90)
Blackbox currently uses 'E'(69), 'G'(71), 'H'(72), 'I'(73), 'P'(80), and 'S'(83) frames, so this reservation is probably too broad.
*/

struct TD_BLACKBOX_E {
    enum { TYPE = 69 }; // 'E'
    uint32_t id {0};

    enum { MAX_BLACKBOX_DATA_SIZE = 246 };
    enum { ESP_NOW_MAX_DATA_SIZE = 250 };

    struct blackbox_t {
        uint8_t headerI;
        std::array<uint8_t, MAX_BLACKBOX_DATA_SIZE - 1> payload;
    };
    union u {
        blackbox_t blackbox;
        std::array<uint8_t, MAX_BLACKBOX_DATA_SIZE> buffer;
    };
    u data {};
};

struct TD_BLACKBOX_I {
    enum { TYPE = 73 }; // 'I'
    uint32_t id {0};

    enum { MAX_BLACKBOX_DATA_SIZE = 246 };
    enum { ESP_NOW_MAX_DATA_SIZE = 250 };

    struct blackbox_t {
        uint8_t headerI;
        std::array<uint8_t, MAX_BLACKBOX_DATA_SIZE - 1> payload;
    };
    union u {
        blackbox_t blackbox;
        std::array<uint8_t, MAX_BLACKBOX_DATA_SIZE> buffer;
    };
    u data;
};

struct TD_BLACKBOX_P {
    enum { TYPE = 80 }; // 'P'
    uint32_t id {0};

    enum { MAX_BLACKBOX_DATA_SIZE = 246 };
    enum { ESP_NOW_MAX_DATA_SIZE = 250 };

    struct blackbox_t {
        uint8_t headerI;
        std::array<uint8_t, MAX_BLACKBOX_DATA_SIZE - 1> payload;
    };
    union u {
        blackbox_t blackbox;
        std::array<uint8_t, MAX_BLACKBOX_DATA_SIZE> buffer;
    };
    u data {};
};

struct TD_BLACKBOX_S {
    enum { TYPE = 83 }; // 'S'
    uint32_t id {0};

    enum { MAX_BLACKBOX_DATA_SIZE = 246 };
    enum { ESP_NOW_MAX_DATA_SIZE = 250 };

    struct blackbox_t {
        uint8_t headerI;
        std::array<uint8_t, MAX_BLACKBOX_DATA_SIZE - 1> payload;
    };
    union u {
        blackbox_t blackbox;
        std::array<uint8_t, MAX_BLACKBOX_DATA_SIZE> buffer;
    };
    u data {};
};


/*!
TYPE RANGE of 100-109 reserved for Self Balancing Robots
*/

struct motor_pair_controller_telemetry_t {
    int32_t encoderLeft {0}; //!< value read from left motor encoder, raw
    int32_t encoderRight {0}; //!< value read from right motor encoder, raw
    int16_t encoderLeftDelta {0}; //!< difference between current left motor encoder value and previous value, raw
    int16_t encoderRightDelta {0}; //!< difference between current right motor encoder value and previous value, raw

    float motorMaxSpeedDPS {0};
    float speedLeftDPS {0}; //!< rotation speed of left motor, degrees per second
    float speedRightDPS {0}; //!< rotation speed of right motor, degrees per second
    float speedDPS_Filtered {0}; //!< speed calculated as the average of speedLeftDPS and speedRightDPS, then filtered

    float powerLeft {0}; //!< power value sent to left motor
    float powerRight {0}; //!< power value sent to right motor

    float pitchAngleOutput {0}; //!< pitch output value calculated by PID
    float speedOutput {0}; //!< speed output value calculated by PID
    float positionOutput {0}; //!< position output value calculated by PID
    float yawRateOutput {0}; //!< yawRate output value calculated by PID

    struct pidfs_error_t {
        float P;
        float I;
        float D;
        float F;
        float S;
    };
    pidfs_error_t pitchError {0, 0, 0, 0, 0}; //!< P, I, D, F, and S errors calculated in pitch PID update
    pidfs_error_t speedError {0, 0, 0, 0, 0}; //!< P, I, D, F, and S errors calculated in speed PID update
    pidfs_error_t positionError {0, 0, 0, 0, 0}; //!< P, I, D, F, and S errors calculated in yawRate PID update
};

/*!
Packet for the transmission of MotorPairController telemetry data.
*/
struct TD_MPC {
    enum { TYPE = 100 };
    uint32_t id {0};

    uint8_t type {TYPE};
    uint8_t len {sizeof(TD_MPC)}; //!< length of whole packet, ie sizeof(TD_MPC)
    uint8_t subType {0};
    uint8_t sequenceNumber {0};

    uint8_t taskIntervalTicks {0}; //!< interval of the MPC task, in ticks
    uint8_t motors {0};
    uint8_t controlMode {0};
    uint8_t filler {0};
    motor_pair_controller_telemetry_t data {};
};

/*!
Packet for the the transmission of PID constants, setpoints, and the balance angle, for self-balancing robots, to enable remote tuning.
*/
struct TD_SBR_PID {
    enum { TYPE = 101 };
    uint32_t id {0};

    uint8_t type {TYPE};
    uint8_t len {sizeof(TD_SBR_PID)}; //!< length of whole packet, ie sizeof(TD_SBR_PID)
    uint8_t subType {0};
    uint8_t sequenceNumber {0};

    enum { ROLL_ANGLE=0, PITCH_ANGLE=1, YAW_RATE=2, SPEED_SERIAL=3, SPEED_PARALLEL=4, POSITION=5, PID_COUNT=6, PID_BEGIN=0 };
    struct PIDFS_t {
        uint16_t kp;
        uint16_t ki;
        uint16_t kd;
        uint16_t kf;
        uint16_t ks;
    };
    struct SPID_t {
        float setpoint;
        PIDFS_t pid;
    };
    struct data_t {
        std::array<SPID_t, PID_COUNT> spids;
        float pitchBalanceAngleDegrees;
    };
    data_t data {};
};


#pragma pack(pop)
