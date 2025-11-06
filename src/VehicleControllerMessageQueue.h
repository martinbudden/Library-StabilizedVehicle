#pragma once

#include <array>
#include <cstdint>

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


class VehicleControllerMessageQueue {
public:
    struct queue_item_t {
        float throttle;
        float roll;
        float pitch;
        float yaw;
    };
public:
#if defined(FRAMEWORK_USE_FREERTOS)
    VehicleControllerMessageQueue()
        : _queue(xQueueCreateStatic(QUEUE_LENGTH, sizeof(queue_item_t), &_queueStorageArea[0], &_queueStatic))
    {}
    inline int32_t WAIT(queue_item_t& queueItem) { return xQueueReceive(_queue, &queueItem, portMAX_DELAY); }
    inline void SIGNAL(const queue_item_t& queueItem) { xQueueOverwrite(_queue, &queueItem); }
#else
    VehicleControllerMessageQueue() = default;
    inline void SIGNAL(const queue_item_t& queueItem) { _queueItem = queueItem; }
    inline const queue_item_t& getQueueItem() const { return _queueItem; }
private:
    queue_item_t _queueItem {};
#endif // FRAMEWORK_USE_FREERTOS
private:
    enum { QUEUE_LENGTH = 1 };
    std::array<uint8_t, QUEUE_LENGTH * sizeof(queue_item_t)> _queueStorageArea {};
#if defined(FRAMEWORK_USE_FREERTOS)
    StaticQueue_t _queueStatic {};
    QueueHandle_t _queue {};
#endif
};
