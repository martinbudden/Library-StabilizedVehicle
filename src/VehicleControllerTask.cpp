#include "VehicleControllerBase.h"
#include "VehicleControllerTask.h"

#include <TimeMicroSeconds.h>

#if defined(USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
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
        _vehicleController.loop(deltaT, tickCount);
    }
}

/*!
Task function for the MotorPairController. Sets up and runs the task loop() function.
*/
[[noreturn]] void VehicleControllerTask::task()
{
#if defined(USE_FREERTOS)
    // pdMS_TO_TICKS Converts a time in milliseconds to a time in ticks.
    const uint32_t taskIntervalTicks = pdMS_TO_TICKS(_taskIntervalMicroSeconds / 1000);
    _previousWakeTimeTicks = xTaskGetTickCount();

    while (true) {
        // delay until the end of the next taskIntervalTicks
        vTaskDelayUntil(&_previousWakeTimeTicks, taskIntervalTicks);

        // calculate _tickCountDelta to get actual deltaT value, since we may have been delayed for more than taskIntervalTicks
        const TickType_t tickCount = xTaskGetTickCount();
        _tickCountDelta = tickCount - _tickCountPrevious;
        _tickCountPrevious = tickCount;
        const uint32_t timeMicroSeconds = timeUs();
        _timeMicroSecondsDelta = timeMicroSeconds - _timeMicroSecondsPrevious;
        _timeMicroSecondsPrevious = timeMicroSeconds;

        if (_tickCountDelta > 0) { // guard against the case of this while loop executing twice on the same tick interval
            const float deltaT = static_cast<float>(pdTICKS_TO_MS(_tickCountDelta)) * 0.001F;
            _vehicleController.loop(deltaT, tickCount);
        }
    }
#else
    while (true) {}
#endif // USE_FREERTOS
}

/*!
Wrapper function for MotorPairController::Task with the correct signature to be used in xTaskCreate.
*/
[[noreturn]] void VehicleControllerTask::Task(void* arg)
{
    const TaskBase::parameters_t* parameters = static_cast<TaskBase::parameters_t*>(arg);

    static_cast<VehicleControllerTask*>(parameters->task)->task(); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
}
