#pragma once

#include "BackchannelBase.h"
#include <ESPNOW_Transceiver.h>


class BackchannelTransceiverESPNOW : public BackchannelTransceiverBase {
public:
    BackchannelTransceiverESPNOW(
        ESPNOW_Transceiver& espnowTransceiver,
        const uint8_t* backchannelMacAddress,
        uint8_t* receivedDataBuffer,
        size_t receivedDataBufferSize
    );
public:
    virtual int sendData(const uint8_t* data, size_t len) const override;
    virtual void WAIT_FOR_DATA_RECEIVED() override;
    virtual size_t getReceivedDataLength() const override;
    virtual void setReceivedDataLengthToZero() override;
    virtual uint32_t getTickCountDeltaAndReset() override;
#if defined(USE_ESPNOW)
protected:
    ESPNOW_Transceiver& _espnowTransceiver;
    ESPNOW_Transceiver::received_data_t _received_data;
    ESPNOW_Transceiver::peer_data_t _peer_data {};
#endif
};
