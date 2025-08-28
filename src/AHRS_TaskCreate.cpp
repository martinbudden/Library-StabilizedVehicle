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


AHRS_Task* AHRS_Task::createTask(task_info_t& taskInfo, AHRS& ahrs, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds)
{
    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static AHRS_Task ahrsTask(taskIntervalMicroSeconds, ahrs);
    ahrs.setTask(&ahrsTask);

#if defined(FRAMEWORK_USE_FREERTOS)
#if defined(USE_DEBUG_PRINTF_TASK_INFORMATION)
    Serial.printf("**** AHRS_Task,              core:%u, priority:%u, task interval:%ums\r\n", coreID, priority, taskIntervalMicroSeconds / 1000);
#endif
    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static TaskBase::parameters_t taskParameters { // NOLINT(misc-const-correctness) false positive
        .task = &ahrsTask
    };
#if !defined(AHRS_TASK_STACK_DEPTH_BYTES)
    enum { AHRS_TASK_STACK_DEPTH_BYTES = 4096 };
#endif
    static std::array <StackType_t, AHRS_TASK_STACK_DEPTH_BYTES> stack;
    static StaticTask_t taskBuffer;
#if !defined(configCHECK_FOR_STACK_OVERFLOW)
    // fill the stack so we can do our own stack overflow detection
    stack.fill(a5);
#endif
    taskInfo = {
        .taskHandle = nullptr,
        .name = "AHRS_Task",
        .stackDepth = AHRS_TASK_STACK_DEPTH_BYTES / sizeof(StackType_t),
        .stackBuffer = &stack[0],
        .priority = priority,
        .coreID = coreID,
    };
    assert(strlen(taskInfo.name) < configMAX_TASK_NAME_LEN && "AHRS_Task: taskname too long");
    assert(taskInfo.priority < configMAX_PRIORITIES && "AHRS_Task: priority too high");

    const TaskHandle_t taskHandle = xTaskCreateStaticPinnedToCore(
        AHRS_Task::Task,
        taskInfo.name,
        taskInfo.stackDepth,
        &taskParameters,
        taskInfo.priority,
        taskInfo.stackBuffer,
        &taskBuffer,
        taskInfo.coreID
    );
    taskInfo.taskHandle = taskHandle;
    assert(taskHandle != nullptr && "Unable to create AHRS task.");
#else
    (void)taskInfo;
    (void)priority;
    (void)coreID;
#endif // FRAMEWORK_USE_FREERTOS
    return &ahrsTask;
}

AHRS_Task* AHRS_Task::createTask(AHRS& ahrs, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds)
{
    task_info_t taskInfo {};
    return createTask(taskInfo, ahrs, priority, coreID, taskIntervalMicroSeconds);
}
