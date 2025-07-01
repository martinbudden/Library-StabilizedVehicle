#include "BackchannelTransceiverESPNOW.h"


#if defined(USE_ESPNOW)

BackchannelTransceiverESPNOW::BackchannelTransceiverESPNOW(
        ESPNOW_Transceiver& espnowTransceiver,
        const uint8_t* backchannelMacAddress,
        uint8_t* receivedDataBuffer,
        size_t receivedDataBufferSize
    ) :
    _espnowTransceiver(espnowTransceiver),
    _received_data(receivedDataBuffer, receivedDataBufferSize)
{
    assert(receivedDataBufferSize >= ESP_NOW_MAX_DATA_LEN);

    _peer_data.receivedDataPtr = &_received_data;
    _espnowTransceiver.addSecondaryPeer(_received_data, backchannelMacAddress);
}

void BackchannelTransceiverESPNOW::WAIT_FOR_DATA_RECEIVED()
{
    _espnowTransceiver.WAIT_FOR_SECONDARY_DATA_RECEIVED();
}

int BackchannelTransceiverESPNOW::sendData(const uint8_t* data, size_t len) const
{
    return _espnowTransceiver.sendDataSecondary(data, len);
}

size_t BackchannelTransceiverESPNOW::getReceivedDataLength() const
{
    return _received_data.len;
}

void BackchannelTransceiverESPNOW::setReceivedDataLengthToZero()
{
    _received_data.len = 0;
}

uint32_t BackchannelTransceiverESPNOW::getTickCountDeltaAndReset()
{
    return _espnowTransceiver.getTickCountDeltaAndReset();
}

#endif // USE_ESPNOW
