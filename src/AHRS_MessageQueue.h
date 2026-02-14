#pragma once

#include <AHRS.h>
#include <MessageQueueBase.h>

#if defined(FRAMEWORK_USE_FREERTOS)
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#else
#if defined(FRAMEWORK_ARDUINO_STM32)
#include <STM32FreeRTOS.h>
#endif
#include <FreeRTOS.h>
#include <queue.h>
#endif
#endif


class AHRS_MessageQueue : public MessageQueueBase {
public:
#if defined(FRAMEWORK_USE_FREERTOS)
    AHRS_MessageQueue() {
        _synchronizationQueueHandle = xQueueCreateStatic(QUEUE_LENGTH, sizeof(_ahrsData), &_synchronizationQueueStorageArea[0], &_synchronizationQueueStatic);
        _ahrsDataQueueHandle = xQueueCreateStatic(QUEUE_LENGTH, sizeof(_ahrsData), &_ahrsDataQueueStorageArea[0], &_ahrsDataQueueStatic);
    }
    // Blackbox Task calls WAIT and then `update` when wait completes
    virtual int32_t WAIT(uint32_t& time_microseconds) override {
        const int32_t ret = xQueueReceive(_synchronizationQueueHandle, &_ahrsData, portMAX_DELAY);
        if (ret) {
            time_microseconds = _ahrsData.time_microseconds;
        }
        return ret;
    }
    // typical xQueueOverwrite overhead ~50 CPU cycles for 60-byte queue
    inline void SIGNAL(const ahrs_data_t& ahrsData) { xQueueOverwrite(_synchronizationQueueHandle, &ahrsData); }
    inline void SEND_AHRS_DATA(const ahrs_data_t& ahrsData) { xQueueOverwrite(_ahrsDataQueueHandle, &ahrsData); }
    // typical xQueuePeek overhead ~40 CPU cycles for 60-byte queue
    inline int32_t PEEK_AHRS_DATA(ahrs_data_t& ahrsData) const { return xQueuePeek(_ahrsDataQueueHandle, &ahrsData, portMAX_DELAY); }
#else
    AHRS_MessageQueue() = default;
    virtual int32_t WAIT(uint32_t& time_microseconds) override { time_microseconds = 0; return 0; }
    inline void SIGNAL(const ahrs_data_t& ahrsData) { _ahrsData = ahrsData; }
    inline void SEND_AHRS_DATA(const ahrs_data_t& ahrsData) { _ahrsData = ahrsData; }
    inline int32_t PEEK_AHRS_DATA(ahrs_data_t& ahrsData) const { ahrsData = _ahrsData; return 0; }
#endif // USE_FREERTOS
    const ahrs_data_t& getReceivedAHRS_Data() const { return _ahrsData; } //!< May only be called within task after WAIT has completed
private:
    ahrs_data_t _ahrsData {};
#if defined(FRAMEWORK_USE_FREERTOS)
    enum { QUEUE_LENGTH = 1 };
    QueueHandle_t _synchronizationQueueHandle;
    StaticQueue_t _synchronizationQueueStatic {};
    std::array<uint8_t, QUEUE_LENGTH * sizeof(_ahrsData)> _synchronizationQueueStorageArea {};
    QueueHandle_t _ahrsDataQueueHandle;
    StaticQueue_t _ahrsDataQueueStatic {};
    std::array<uint8_t, QUEUE_LENGTH * sizeof(_ahrsData)> _ahrsDataQueueStorageArea {};
#endif
};
