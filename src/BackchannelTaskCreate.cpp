#include "BackchannelBase.h"
#include "BackchannelTask.h"

#if defined(USE_DEBUG_PRINTF_TASK_INFORMATION)
#if defined(USE_ESPNOW)
#include <HardwareSerial.h>
#endif
#endif

#include <array>
#include <cstring>

#if defined(USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#include <freertos/FreeRTOSConfig.h>
#include <freertos/task.h>
#endif


BackchannelTask* BackchannelTask::createTask(task_info_t& taskInfo, BackchannelBase& backchannel, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds)
{
    static BackchannelTask backchannelTask(taskIntervalMicroSeconds, backchannel);

#if defined(USE_FREERTOS)
#if defined(USE_DEBUG_PRINTF_TASK_INFORMATION)
    Serial.printf("**** BackchannelTask,        core:%u, priority:%u, task interval:%ums\r\n\r\n", coreID, priority, taskIntervalMicroSeconds / 1000);
#endif
    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static TaskBase::parameters_t taskParameters { // NOLINT(misc-const-correctness) false positive
        .task = &backchannelTask
    };
#if !defined(BACKCHANNEL_TASK_STACK_DEPTH_BYTES)
    enum { BACKCHANNEL_TASK_STACK_DEPTH_BYTES = 4096 }; // 2048 probably sufficient when not using Serial.printf statements
#endif
    static std::array <StackType_t, BACKCHANNEL_TASK_STACK_DEPTH_BYTES> stack;
    static StaticTask_t taskBuffer;
    taskInfo = {
        .taskHandle = nullptr,
        .name = "Backchannel", // max length 16, including zero terminator
        .stackDepth = BACKCHANNEL_TASK_STACK_DEPTH_BYTES,
        .stackBuffer = &stack[0],
        .priority = priority,
        .coreID = coreID,
    };
    assert(strlen(taskInfo.name) < configMAX_TASK_NAME_LEN && "Backchannel: taskname too long");
    assert(taskInfo.priority < configMAX_PRIORITIES && "Backchannel: priority too high");

    taskInfo.taskHandle = xTaskCreateStaticPinnedToCore(
        BackchannelTask::Task,
        taskInfo.name,
        taskInfo.stackDepth,
        &taskParameters,
        taskInfo.priority,
        taskInfo.stackBuffer,
        &taskBuffer,
        taskInfo.coreID
    );
    assert(taskInfo.taskHandle != nullptr && "Unable to create BackchannelTask.");
#if !defined(configCHECK_FOR_STACK_OVERFLOW)
    // fill the stack so we can do our own stack overflow detection
    stack.fill(a5);
#endif
#else
    (void)taskInfo;
    (void)priority;
    (void)coreID;
#endif // USE_FREERTOS
    return &backchannelTask;
}

BackchannelTask* BackchannelTask::createTask(BackchannelBase& backchannel, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds)
{
    task_info_t taskInfo{};
    return createTask(taskInfo, backchannel, priority, coreID, taskIntervalMicroSeconds);
}
