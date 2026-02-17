#include "VehicleControllerBase.h"
#include "VehicleControllerTask.h"

#include <TimeMicroseconds.h>

#if defined(FRAMEWORK_USE_FREERTOS)
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#else
#if defined(FRAMEWORK_ARDUINO_STM32)
#include <STM32FreeRTOS.h>
#endif
#include <FreeRTOS.h>
#include <task.h>
#endif
#endif


VehicleControllerTask::VehicleControllerTask(VehicleControllerBase& vehicleController, VehicleControllerMessageQueue& vehicleControllerMessageQueue, MotorMixerBase& motorMixer, RpmFilters* rpmFilters) :
    TaskBase(vehicleController.getTaskIntervalMicroseconds()),
    _taskIntervalMilliseconds(vehicleController.getTaskIntervalMicroseconds()/1000), // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    _vehicleController(vehicleController),
    _vehicleControllerMessageQueue(vehicleControllerMessageQueue),
    _motorMixer(motorMixer),
    _rpmFilters(rpmFilters)
{
}


/*!
loop() function for when not using FREERTOS
*/
void VehicleControllerTask::loop()
{
#if !defined(FRAMEWORK_USE_FREERTOS)
    uint32_t tickCount = timeMs();
    _tickCountDelta = tickCount - _tickCountPrevious;
    _tickCountPrevious = tickCount;

    if (_tickCountDelta >= _taskIntervalMilliseconds) { // if _taskIntervalMicroseconds has passed, then run the update
        const float delta_t = static_cast<float>(_tickCountDelta) * 0.001F;
        tickCount = timeMs();
        const VehicleControllerMessageQueue::queue_item_t queueItem = _vehicleControllerMessageQueue.getQueueItem();
        _vehicleController.outputToMixer(delta_t, tickCount, queueItem, _motorMixer, _rpmFilters);
    }
#endif
}

/*!
Task function for the VehicleController.
*/
[[noreturn]] void VehicleControllerTask::task()
{
#if defined(FRAMEWORK_USE_FREERTOS)
    VehicleControllerMessageQueue::queue_item_t queueItem {};
    while (true) {
        _vehicleControllerMessageQueue.WAIT(queueItem);

        // calculate timings for instrumentation
        const TickType_t tickCount = xTaskGetTickCount();
        _tickCountDelta = tickCount - _tickCountPrevious;
        _tickCountPrevious = tickCount;

        const float delta_t = static_cast<float>(_tickCountDelta) * 0.001F;
        _vehicleController.outputToMixer(delta_t, tickCount, queueItem, _motorMixer, _rpmFilters);
    }
#else
    while (true) {}
#endif // FRAMEWORK_USE_FREERTOS
}

/*!
Wrapper function for VehicleController::Task with the correct signature to be used in xTaskCreate.
*/
[[noreturn]] void VehicleControllerTask::Task(void* arg)
{
    const TaskBase::parameters_t* parameters = static_cast<TaskBase::parameters_t*>(arg);

    static_cast<VehicleControllerTask*>(parameters->task)->task(); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
}
