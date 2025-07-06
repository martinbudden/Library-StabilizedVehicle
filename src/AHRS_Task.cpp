#include "AHRS.h"
#include "AHRS_Task.h"

#include <TimeMicroSeconds.h>

#if defined(USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif


/*!
loop() function for when not using FREERTOS
*/
void AHRS_Task::loop()
{
    const timeUs32_t timeMicroSeconds = timeUs();
    _timeMicroSecondsDelta = timeMicroSeconds - _timeMicroSecondsPrevious;

    if (_timeMicroSecondsDelta >= _taskIntervalMicroSeconds) { // if _taskIntervalMicroSeconds has passed, then run the update
        _timeMicroSecondsPrevious = timeMicroSeconds;
        _ahrs.readIMUandUpdateOrientation(timeMicroSeconds, _timeMicroSecondsDelta);
    }
}

/*!
Task function for the AHRS. Sets up and runs the task loop() function.
*/
[[noreturn]] void AHRS_Task::task()
{
#if defined(USE_FREERTOS)

#if defined(AHRS_IS_INTERRUPT_DRIVEN)
    while (true) {
        _IMU.WAIT_IMU_DATA_READY(); // wait until there is IMU data.

        const timeUs32_t timeMicroSeconds = timeUs();
        const timeUs32_t timeMicroSecondsDelta = timeMicroSeconds - _timeMicroSecondsPrevious;
        _timeMicroSecondsPrevious = timeMicroSeconds;
        if (timeMicroSecondsDelta > 0) {
            _ahrs.readIMUandUpdateOrientation(timeMicroSeconds, _timeMicroSecondsDelta);
        }
    }
#else
    // pdMS_TO_TICKS Converts a time in milliseconds to a time in ticks.
    const uint32_t taskIntervalTicks = pdMS_TO_TICKS(_taskIntervalMicroSeconds / 1000);
    assert(taskIntervalTicks > 0 && "AHRS taskIntervalTicks is zero.");
    //Serial.print("AHRS us:");
    //Serial.println(taskIntervalTicks);
    _previousWakeTimeTicks = xTaskGetTickCount();

    while (true) {
        // delay until the end of the next taskIntervalTicks
        vTaskDelayUntil(&_previousWakeTimeTicks, taskIntervalTicks);

        // calculate _tickCountDelta to get actual deltaT value, since we may have been delayed for more than taskIntervalTicks
        const TickType_t tickCount = xTaskGetTickCount();
        _tickCountDelta = tickCount - _tickCountPrevious;
        _tickCountPrevious = tickCount;
        const timeUs32_t timeMicroSeconds = timeUs();
        _timeMicroSecondsDelta = timeMicroSeconds - _timeMicroSecondsPrevious;
        _timeMicroSecondsPrevious = timeMicroSeconds;

        if (_tickCountDelta > 0) { // guard against the case of this while loop executing twice on the same tick interval
            _ahrs.readIMUandUpdateOrientation(timeMicroSeconds, _timeMicroSecondsDelta);
        }
    }
#endif // AHRS_IS_INTERRUPT_DRIVEN
#else
    while (true) {}
#endif // USE_FREERTOS
}

/*!
Wrapper function for AHRS::Task with the correct signature to be used in xTaskCreate.
*/
[[noreturn]] void AHRS_Task::Task(void* arg)
{
    const TaskBase::parameters_t* parameters = static_cast<TaskBase::parameters_t*>(arg);

    static_cast<AHRS_Task*>(parameters->task)->task(); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
}
