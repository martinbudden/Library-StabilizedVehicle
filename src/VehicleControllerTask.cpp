#include "VehicleControllerBase.h"
#include "VehicleControllerTask.h"

#include <TimeMicroSeconds.h>

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


/*!
loop() function for when not using FREERTOS
*/
void VehicleControllerTask::loop()
{
    const timeUs32_t timeMicroSeconds = timeUs();
    _timeMicroSecondsDelta = timeMicroSeconds - _timeMicroSecondsPrevious;

    if (_timeMicroSecondsDelta >= _taskIntervalMicroSeconds) { // if _taskIntervalMicroSeconds has passed, then run the update
        _timeMicroSecondsPrevious = timeMicroSeconds;
        const float deltaT = static_cast<float>(_timeMicroSecondsDelta) * 0.000001F;
        const uint32_t tickCount = timeUs() / 1000;
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

        const uint32_t timeMicroSeconds = timeUs();
        _timeMicroSecondsDelta = timeMicroSeconds - _timeMicroSecondsPrevious;
        _timeMicroSecondsPrevious = timeMicroSeconds;
        const float deltaT = static_cast<float>(_timeMicroSecondsDelta) * 0.000001F;

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
