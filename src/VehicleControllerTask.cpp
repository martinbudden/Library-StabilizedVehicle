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


VehicleControllerTask::VehicleControllerTask(VehicleControllerBase& vehicleController) :
    TaskBase(vehicleController.getTaskIntervalMicroseconds()),
    _taskIntervalMilliseconds(vehicleController.getTaskIntervalMicroseconds()/1000), // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    _vehicleController(vehicleController)
{
}


/*!
loop() function for when not using FREERTOS
*/
void VehicleControllerTask::loop()
{
    uint32_t tickCount = timeMs();
    _tickCountDelta = tickCount - _tickCountPrevious;
    _tickCountPrevious = tickCount;

    if (_tickCountDelta >= _taskIntervalMilliseconds) { // if _taskIntervalMicroseconds has passed, then run the update
        const float deltaT = static_cast<float>(_tickCountDelta) * 0.001F;
        tickCount = timeMs();
        _vehicleController.outputToMixer(deltaT, tickCount, _vehicleController.getMessageQueueItem());
    }
}

/*!
Task function for the MotorPairController. Sets up and runs the task loop() function.
*/
[[noreturn]] void VehicleControllerTask::task()
{
#if defined(FRAMEWORK_USE_FREERTOS)
    while (true) {
        _vehicleController.WAIT();

        // calculate timings for instrumentation
        const TickType_t tickCount = xTaskGetTickCount();
        _tickCountDelta = tickCount - _tickCountPrevious;
        _tickCountPrevious = tickCount;

        const float deltaT = static_cast<float>(_tickCountDelta) * 0.001F;
        _vehicleController.outputToMixer(deltaT, tickCount, _vehicleController.getMessageQueueItem());
    }
#else
    while (true) {}
#endif // FRAMEWORK_USE_FREERTOS
}

/*!
Wrapper function for MotorPairController::Task with the correct signature to be used in xTaskCreate.
*/
[[noreturn]] void VehicleControllerTask::Task(void* arg)
{
    const TaskBase::parameters_t* parameters = static_cast<TaskBase::parameters_t*>(arg);

    static_cast<VehicleControllerTask*>(parameters->task)->task(); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
}
