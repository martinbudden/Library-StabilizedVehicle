#pragma once

#include <array>
#include <cstdint>

#if defined(USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#endif

class VehicleControllerMessageQueue {
public:
    struct queue_item_t {
        float roll;
        float pitch;
        float yaw;
    };
public:
#if defined(USE_FREERTOS)
    VehicleControllerMessageQueue()
        : _queue(xQueueCreateStatic(QUEUE_LENGTH, sizeof(_queueItem), &_queueStorageArea[0], &_queueStatic))
    {}
    inline int32_t WAIT() const { return xQueueReceive(_queue, &_queueItem, portMAX_DELAY); }
    inline void SIGNAL(const queue_item_t& queueItem) const { xQueueOverwrite(_queue, &queueItem); }
#else
    VehicleControllerMessageQueue() = default;
    inline int32_t WAIT() const { return 0; }
    inline void SIGNAL(const queue_item_t& queueItem) const { _queueItem = queueItem; }
#endif // USE_FREERTOS
    inline const queue_item_t& getQueueItem() const { return _queueItem; }
private:
    mutable queue_item_t _queueItem {};
    enum { QUEUE_LENGTH = 1 };
    std::array<uint8_t, QUEUE_LENGTH * sizeof(_queueItem)> _queueStorageArea {};
#if defined(USE_FREERTOS)
    StaticQueue_t _queueStatic {};
    QueueHandle_t _queue {};
#endif
};
