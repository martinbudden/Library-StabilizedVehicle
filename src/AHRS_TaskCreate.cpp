#include "AHRS.h"
#include "AHRS_Task.h"

#if defined(USE_DEBUG_PRINTF_TASK_INFORMATION)
#if defined(FRAMEWORK_ARDUINO_ESP32)
#include <HardwareSerial.h>
#endif
#endif

#include <array>
#include <cstring>

#if defined(FRAMEWORK_USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#include <freertos/FreeRTOSConfig.h>
#include <freertos/task.h>
#endif



AHRS_Task* AHRS_Task::createTask(AHRS& ahrs, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds)
{
    task_info_t taskInfo {};
    return createTask(taskInfo, ahrs, priority, coreID, taskIntervalMicroSeconds);
}

AHRS_Task* AHRS_Task::createTask(task_info_t& taskInfo, AHRS& ahrs, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds)
{
    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static AHRS_Task ahrsTask(taskIntervalMicroSeconds, ahrs);
    ahrs.setTask(&ahrsTask);

    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static TaskBase::parameters_t taskParameters { // NOLINT(misc-const-correctness) false positive
        .task = &ahrsTask
    };
#if !defined(AHRS_TASK_STACK_DEPTH_BYTES)
    enum { AHRS_TASK_STACK_DEPTH_BYTES = 4096 };
#endif
    static std::array <uint8_t, AHRS_TASK_STACK_DEPTH_BYTES> stack;
    taskInfo = {
        .taskHandle = nullptr,
        .name = "AHRS_Task",
        .stackDepth = AHRS_TASK_STACK_DEPTH_BYTES,
        .stackBuffer = &stack[0],
        .priority = priority,
        .coreID = coreID,
        .taskIntervalMicroSeconds = taskIntervalMicroSeconds,
    };

#if defined(FRAMEWORK_USE_FREERTOS)
    assert(std::strlen(taskInfo.name) < configMAX_TASK_NAME_LEN);
    assert(taskInfo.priority < configMAX_PRIORITIES);

#if !defined(configCHECK_FOR_STACK_OVERFLOW)
    // fill the stack so we can do our own stack overflow detection
    stack.fill(0xA5);
#endif
    static StaticTask_t taskBuffer;
    const TaskHandle_t taskHandle = xTaskCreateStaticPinnedToCore(
        AHRS_Task::Task,
        taskInfo.name,
        taskInfo.stackDepth / sizeof(StackType_t),
        &taskParameters,
        taskInfo.priority,
        taskInfo.stackBuffer,
        &taskBuffer,
        taskInfo.coreID
    );
    taskInfo.taskHandle = taskHandle;
    assert(taskHandle != nullptr && "Unable to create AHRS task.");
#else
    (void)taskParameters;
#endif // FRAMEWORK_USE_FREERTOS

    return &ahrsTask;
}
