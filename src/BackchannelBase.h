#pragma once

#include <cstddef>
#include <cstdint>

class AHRS;
class SV_Preferences;
class TaskBase;

/*!
Virtual base class for the backchannel transceiver.
Can be subclassed to provide transceivers over ESPNOW or UDP (for example).
*/
class BackchannelTransceiverBase {
public:
    virtual int sendData(const uint8_t* data, size_t len) const = 0;
    virtual void WAIT_FOR_DATA_RECEIVED() = 0;
    virtual size_t getReceivedDataLength() const = 0;
    virtual void setReceivedDataLengthToZero() = 0;
    virtual uint32_t getTickCountDeltaAndReset() = 0;
};

class BackchannelBase {
public:
    struct base_init_t {
        BackchannelTransceiverBase* backchannelTransceiverPtr;
        uint8_t* transmitDataBufferPtr;
        size_t transmitDataBufferSize;
        uint8_t* receivedDataBufferPtr;
        size_t receivedDataBufferSize;
    };
protected:
    BackchannelBase(const base_init_t& init) :
        _backchannelTransceiverPtr(init.backchannelTransceiverPtr),
        _transmitDataBufferPtr(init.transmitDataBufferPtr),
        _transmitDataBufferSize(init.transmitDataBufferSize),
        _receivedDataBufferPtr(init.receivedDataBufferPtr),
        _receivedDataBufferSize(init.receivedDataBufferSize)
    {}
public:
    void WAIT_FOR_DATA_RECEIVED() { _backchannelTransceiverPtr->WAIT_FOR_DATA_RECEIVED(); }
    int sendData(const uint8_t* data, size_t len) const { return _backchannelTransceiverPtr->sendData(data, len); }

    virtual bool processedReceivedPacket() = 0;
    virtual bool sendPacket(uint8_t subCommand) = 0;
    bool sendPacket() { return sendPacket(0); }

protected:
    BackchannelTransceiverBase* const _backchannelTransceiverPtr {};
    uint8_t* const _transmitDataBufferPtr {};
    const size_t _transmitDataBufferSize {};
    uint8_t* const _receivedDataBufferPtr {};
    const size_t _receivedDataBufferSize {};
};
