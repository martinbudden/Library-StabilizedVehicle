#include "VehicleControllerBase.h"
#include "VehicleControllerTask.h"

#include <TimeMicroSeconds.h>

#if defined(FRAMEWORK_USE_FREERTOS)
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
        _vehicleController.outputToMixer(deltaT, tickCount, _vehicleController.getMessageQueueItem());
    }
}

/*!
Task function for the MotorPairController. Sets up and runs the task loop() function.
*/
[[noreturn]] void VehicleControllerTask::task()
{
#if defined(FRAMEWORK_USE_FREERTOS)
    const uint32_t taskIntervalTicks = _taskIntervalMicroSeconds < 1000 ? 1 : pdMS_TO_TICKS(_taskIntervalMicroSeconds / 1000);
    if (_taskIntervalMicroSeconds != 0) {
        // time driven scheduling
        _previousWakeTimeTicks = xTaskGetTickCount();
    }

    while (true) {
        if (_taskIntervalMicroSeconds == 0) {
            _vehicleController.WAIT();
        } else {
            // delay until the end of the next taskIntervalTicks
            vTaskDelayUntil(&_previousWakeTimeTicks, taskIntervalTicks);
            _vehicleController.PEEK(); // peek, so getMessageQueueItem returns a valid value
        }
        // store timings for instrumentation
        const TickType_t tickCount = xTaskGetTickCount();
        _tickCountDelta = tickCount - _tickCountPrevious;
        _tickCountPrevious = tickCount;
        const uint32_t timeMicroSeconds = timeUs();
        _timeMicroSecondsDelta = timeMicroSeconds - _timeMicroSecondsPrevious;
        _timeMicroSecondsPrevious = timeMicroSeconds;

        // use _tickCountDelta to get actual deltaT value, since we may have been delayed for more than taskIntervalTicks
        const float deltaT = static_cast<float>(pdTICKS_TO_MS(_tickCountDelta)) * 0.001F;
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
