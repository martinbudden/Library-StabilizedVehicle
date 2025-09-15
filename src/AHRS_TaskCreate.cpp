#include "AHRS.h"
#include "AHRS_Task.h"

#include <array>
#include <cstring>

#if defined(FRAMEWORK_USE_FREERTOS)
#if defined(FRAMEWORK_USE_FREERTOS_SUBDIRECTORY)
#include <freertos/FreeRTOS.h>
#include <freertos/FreeRTOSConfig.h>
#include <freertos/task.h>
#else
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <task.h>
#endif
#endif


AHRS_Task* AHRS_Task::createTask(AHRS& ahrs, uint8_t priority, uint32_t core, uint32_t taskIntervalMicroSeconds)
{
    task_info_t taskInfo {};
    return createTask(taskInfo, ahrs, priority, core, taskIntervalMicroSeconds);
}

AHRS_Task* AHRS_Task::createTask(task_info_t& taskInfo, AHRS& ahrs, uint8_t priority, uint32_t core, uint32_t taskIntervalMicroSeconds)
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
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32) || defined(FRAMEWORK_TEST)
    static std::array <uint8_t, AHRS_TASK_STACK_DEPTH_BYTES> stack;
#else
    static std::array <StackType_t, AHRS_TASK_STACK_DEPTH_BYTES / sizeof(StackType_t)> stack;
#endif
    taskInfo = {
        .taskHandle = nullptr,
        .name = "AHRS_Task",
        .stackDepthBytes = AHRS_TASK_STACK_DEPTH_BYTES,
        .stackBuffer = reinterpret_cast<uint8_t*>(&stack[0]), // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        .priority = priority,
        .core = core,
        .taskIntervalMicroSeconds = taskIntervalMicroSeconds,
    };

#if defined(FRAMEWORK_USE_FREERTOS)
    assert(std::strlen(taskInfo.name) < configMAX_TASK_NAME_LEN);
    assert(taskInfo.priority < configMAX_PRIORITIES);

#if !defined(configCHECK_FOR_STACK_OVERFLOW)
    // fill the stack so we can do our own stack overflow detection
    enum { STACK_FILLER = 0xA5 };
    stack.fill(STACK_FILLER);
#endif
    static StaticTask_t taskBuffer;
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
    taskInfo.taskHandle = xTaskCreateStaticPinnedToCore(
        AHRS_Task::Task,
        taskInfo.name,
        taskInfo.stackDepthBytes / sizeof(StackType_t),
        &taskParameters,
        taskInfo.priority,
        &stack[0],
        &taskBuffer,
        taskInfo.core
    );
    assert(taskInfo.taskHandle != nullptr && "Unable to create AHRS task.");
#else
    taskInfo.taskHandle = xTaskCreateStaticAffinitySet(
        AHRS_Task::Task,
        taskInfo.name,
        taskInfo.stackDepthBytes / sizeof(StackType_t),
        &taskParameters,
        taskInfo.priority,
        &stack[0],
        &taskBuffer,
        taskInfo.core
    );
    assert(taskInfo.taskHandle != nullptr && "Unable to create AHRS task.");
#endif
#else
    (void)taskParameters;
#endif // FRAMEWORK_USE_FREERTOS

    return &ahrsTask;
}
