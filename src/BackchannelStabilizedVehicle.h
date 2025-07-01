#pragma once

#include <BackchannelBase.h>
#include <CommandPacket.h>

class AHRS_Task;
class ReceiverBase;
class TaskBase;
class VehicleControllerBase;
class VehicleControllerTask;

/*!
Backchannel that sends and receives packets that contain data for a stabilized vehicle.
*/
class BackchannelStabilizedVehicle : public BackchannelBase {
public:
    BackchannelStabilizedVehicle(
        const base_init_t& baseInit,
        uint32_t backchannelID,
        uint32_t telemetryID,
        VehicleControllerBase& vehicleController,
        AHRS& ahrs,
        const ReceiverBase& receiver,
        SV_Preferences& preferences,
        const TaskBase* mainTask
    );
    BackchannelStabilizedVehicle(
        const base_init_t& baseInit,
        uint32_t backchannelID,
        uint32_t telemetryID,
        VehicleControllerBase& vehicleController,
        AHRS& ahrs,
        const ReceiverBase& receiver, 
        SV_Preferences& preferences
    );
public:
    virtual bool sendPacket(uint8_t subCommand) override;
    static uint32_t idFromMacAddress(const uint8_t* macAddress);
protected:
    virtual bool processedReceivedPacket() override;
    virtual bool packetRequestData(const CommandPacketRequestData& packet);
    virtual bool packetSetOffset(const CommandPacketSetOffset& packet);
    virtual bool packetControl(const CommandPacketControl& packet);
    virtual bool packetSetPID(const CommandPacketSetPID& packet);
protected:
    VehicleControllerBase& _vehicleController;
    AHRS& _ahrs;
    const ReceiverBase& _receiver;
    SV_Preferences& _preferences;
    const TaskBase* _mainTask;
    const uint32_t _backchannelID;
    const uint32_t _telemetryID;
    uint32_t _requestType { CommandPacketRequestData::REQUEST_STOP_SENDING_DATA }; // So on startup a reset screen packet is sent
    uint32_t _sequenceNumber {0};
};
