#include "AHRS.h"
#include "AHRS_Task.h"
#include "VehicleControllerBase.h"

#include <TimeMicroseconds.h>
#include <imu_base.h>

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
void AHRS_Task::loop()
{
    const timeUs32_t time_microseconds = timeUs();
    _tickCountDelta = time_microseconds - _timeMicrosecondsPrevious;

    if (_timeMicrosecondsDelta >= _taskIntervalMicroseconds) { // if _taskIntervalMicroseconds has passed, then run the update
        _timeMicrosecondsPrevious = time_microseconds;
        _ahrs.readIMUandUpdateOrientation(time_microseconds, _timeMicrosecondsDelta, _imuFilters, _vehicleController);
    }
}

/*!
Task function for the AHRS. Sets up and runs the task loop() function.
*/
[[noreturn]] void AHRS_Task::task()
{
#if defined(FRAMEWORK_USE_FREERTOS)
    if (_taskIntervalMicroseconds == 0) {
        // interrupt driven scheduling
        while (true) {
            _ahrs.getIMU().WAIT_IMU_DATA_READY(); // wait until there is IMU data.
            const timeUs32_t time_microseconds = timeUs();
            _timeMicrosecondsDelta = time_microseconds - _timeMicrosecondsPrevious;
            _timeMicrosecondsPrevious = time_microseconds;
            if (_timeMicrosecondsDelta > 0) { // guard against the case of this while loop executing twice on the same tick interval
                _ahrs.readIMUandUpdateOrientation(time_microseconds, _timeMicrosecondsDelta, _imuFilters, _vehicleController);
            }
        }
    } else {
        const uint32_t taskIntervalTicks = _taskIntervalMicroseconds < 1000 ? 1 : pdMS_TO_TICKS(_taskIntervalMicroseconds / 1000);
        _previousWakeTimeTicks = xTaskGetTickCount();
        while (true) {
            // delay until the end of the next taskIntervalTicks
#if (tskKERNEL_VERSION_MAJOR > 10) || ((tskKERNEL_VERSION_MAJOR == 10) && (tskKERNEL_VERSION_MINOR >= 5))
            const BaseType_t wasDelayed = xTaskDelayUntil(&_previousWakeTimeTicks, taskIntervalTicks);
            if (wasDelayed) {
                _wasDelayed = true;
            }
#else
            vTaskDelayUntil(&_previousWakeTimeTicks, taskIntervalTicks);
#endif
            // record tickCounts for instrumentation. Not sure if this is useful anymore
            const TickType_t tickCount = xTaskGetTickCount();
            _tickCountDelta = tickCount - _tickCountPrevious;
            _tickCountPrevious = tickCount;
            const timeUs32_t time_microseconds = timeUs();
            _timeMicrosecondsDelta = time_microseconds - _timeMicrosecondsPrevious;
            _timeMicrosecondsPrevious = time_microseconds;
            if (_timeMicrosecondsDelta > 0) { // guard against the case of this while loop executing twice on the same tick interval
                const ahrs_data_t& ahrsData = _ahrs.readIMUandUpdateOrientation(time_microseconds, _timeMicrosecondsDelta, _imuFilters, _vehicleController);
                _vehicleController.updateOutputsUsingPIDs(ahrsData);
            }
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
