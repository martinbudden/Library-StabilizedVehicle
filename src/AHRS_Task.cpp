#include "AHRS.h"
#include "AHRS_Task.h"

#include <TimeMicroSeconds.h>

#if defined(FRAMEWORK_USE_FREERTOS)
#if defined(FRAMEWORK_USE_FREERTOS_SUBDIRECTORY)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#else
#include <FreeRTOS.h>
#include <task.h>
#endif
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
#if defined(FRAMEWORK_USE_FREERTOS)
    const uint32_t taskIntervalTicks = _taskIntervalMicroSeconds < 1000 ? 1 : pdMS_TO_TICKS(_taskIntervalMicroSeconds / 1000);
    if (_taskIntervalMicroSeconds != 0) {
        // time driven scheduling
        _previousWakeTimeTicks = xTaskGetTickCount();
    }

    while (true) {
        if (_taskIntervalMicroSeconds == 0) {
            // event driven scheduling
            _ahrs.getIMU().WAIT_IMU_DATA_READY(); // wait until there is IMU data.
        } else {
            // delay until the end of the next taskIntervalTicks
            vTaskDelayUntil(&_previousWakeTimeTicks, taskIntervalTicks);

            // record tickCounts for instrumentation. Not sure if this is useful anymore
            const TickType_t tickCount = xTaskGetTickCount();
            _tickCountDelta = tickCount - _tickCountPrevious;
            _tickCountPrevious = tickCount;
        }
        const timeUs32_t timeMicroSeconds = timeUs();
        _timeMicroSecondsDelta = timeMicroSeconds - _timeMicroSecondsPrevious;
        _timeMicroSecondsPrevious = timeMicroSeconds;
        if (_timeMicroSecondsDelta > 0) { // guard against the case of this while loop executing twice on the same tick interval
            _ahrs.readIMUandUpdateOrientation(timeMicroSeconds, _timeMicroSecondsDelta);
        }
    }
#else
    while (true) {}
#endif // FRAMEWORK_USE_FREERTOS
}

/*!
Wrapper function for AHRS::Task with the correct signature to be used in xTaskCreate.
*/
[[noreturn]] void AHRS_Task::Task(void* arg)
{
    const TaskBase::parameters_t* parameters = static_cast<TaskBase::parameters_t*>(arg);

    static_cast<AHRS_Task*>(parameters->task)->task(); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
}
