# pragma once

#include <array>
#include <cstdint>

#pragma pack(push, 1)
/*!
Command packet type 0 is reserved for future use.
*/
struct CommandPacketReserved {
    enum { TYPE = 0 };
    uint32_t id;
    uint8_t type;
    uint8_t len; // length of whole packet, ie sizeof(CommandPacketReserved)
    uint16_t value;
};

struct CommandPacketControl {
    enum { TYPE = 1 };
    enum {
        NO_ACTION = 0,
        MOTORS_SWITCH_OFF = 1,
        MOTORS_SWITCH_ON = 2,
        RESET = 3,
        SET_MODE = 4,
        SET_PID_PROFILE = 5,
        SET_RATES_PROFILE = 6,
    };
    uint32_t id;
    uint8_t type;
    uint8_t len; // length of whole packet, ie sizeof(CommandPacketControl)
    uint8_t value;
    uint8_t control;
};

struct CommandPacketRequestData {
    enum { TYPE = 2 };
    enum {
        NO_REQUEST = 0,
        REQUEST_STOP_SENDING_DATA = 1,
        REQUEST_TASK_INTERVAL_DATA = 2,
        REQUEST_TASK_INTERVAL_EXTENDED_DATA = 3,
        REQUEST_AHRS_DATA = 4,
        REQUEST_RECEIVER_DATA = 5,
        REQUEST_PID_DATA = 6,
        REQUEST_VEHICLE_CONTROLLER_DATA = 7,
        REQUEST_MSP_DATA = 8
    };
    uint32_t id;
    uint8_t type;
    uint8_t len; // length of whole packet, ie sizeof(CommandPacketRequestData)
    uint8_t requestType;
    uint8_t valueType;
};

struct CommandPacketSetPID {
    enum { TYPE = 3 };
    enum {
        NO_ACTION = 0,
        SET_P = 1,  SET_I = 2,  SET_D = 3,  SET_F = 4,
        SAVE_P = 5, SAVE_I = 6, SAVE_D = 7, SAVE_F = 8,
        RESET_PID = 9,
        SET_SETPOINT = 10,
        SET_PITCH_BALANCE_ANGLE = 11,
        SAVE_PITCH_BALANCE_ANGLE = 12
    };
    enum { MPC_ROLL_ANGLE=0, MPC_PITCH_ANGLE=1, MPC_YAW_RATE=2, MPC_SPEED_SERIAL=3, MPC_SPEED_PARALLEL=4, MPC_POSITION=5, MPC_PID_COUNT=6, MPC_PID_BEGIN=0 };
    enum { FC_ROLL_RATE=0,  FC_PITCH_RATE=1, FC_YAW_RATE=2, FC_ROLL_ANGLE=3, FC_PITCH_ANGLE=4, FC_PID_COUNT=5, FC_PID_BEGIN=0 };
    uint32_t id;

    uint8_t type;
    uint8_t len; // length of whole packet, ie sizeof(CommandPacketSetPID)
    uint8_t pidIndex;
    uint8_t setType;

    float f0;
    uint8_t value;
};

struct CommandPacketSetOffset {
    enum { TYPE = 4 };
    enum {
        NO_ACTION = 0,
        SET_GYRO_OFFSET_X = 1, SET_GYRO_OFFSET_Y = 2, SET_GYRO_OFFSET_Z = 3, SAVE_GYRO_OFFSET = 4,
        SET_ACC_OFFSET_X  = 5, SET_ACC_OFFSET_Y  = 6, SET_ACC_OFFSET_Z  = 7, SAVE_ACC_OFFSET  = 8
    };
    uint32_t id;
    uint8_t type;
    uint8_t len; // length of whole packet, ie sizeof(CommandPacketSetOffset)
    uint8_t setType;
    uint8_t filler;
    int32_t value;
};

struct CommandPacketSetFilter {
    enum { TYPE = 5 };
    enum {
        GYRO_ALL_LPF, GYRO_X_LPF, GYRO_Y_LPF, GYRO_Z_LPF,
        ACC_ALL_LPF,  ACC_X_LPF,  ACC_Y_LPF,  ACC_Z_LPF
    };
    uint32_t id;
    uint8_t type;
    uint8_t len; // length of whole packet, ie sizeof(CommandPacketSetFilter)
    uint8_t itemIndex;
    uint8_t filterType;
    float value0;
    float value1;
    float value2;
    float value3;
};

/*
NOTE: enough space is reserved for a full-size MSP packet, this is more than
can be accommodated in an ESP_NOW packet, so size payloadSize must be checked
before the packet is sent over ESP_NOW.
*/
struct CommandPacketMSP {
    enum { TYPE = 36 }; // '$'
    uint32_t id;

    enum { MAX_MSP_DATA_SIZE = 256 };
    enum { MSP_HEADER_AND_CHECKSUM_SIZE = 6 };
    enum { PACKET_OVERHEAD = sizeof(id) + MSP_HEADER_AND_CHECKSUM_SIZE };
    enum { ESP_NOW_MAX_DATA_SIZE = 250 };
    enum { MAX_PAYLOAD_SIZE_FOR_ESP = ESP_NOW_MAX_DATA_SIZE - PACKET_OVERHEAD };

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
    u data;
};
#pragma pack(pop)
