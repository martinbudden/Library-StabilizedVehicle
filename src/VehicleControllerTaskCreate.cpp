#include "VehicleControllerBase.h"
#include "VehicleControllerTask.h"

#if defined(USE_ESPNOW)
#include <HardwareSerial.h>
#endif

#if defined(USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#include <freertos/FreeRTOSConfig.h>
#include <freertos/task.h>
#endif


VehicleControllerTask* VehicleControllerTask::createTask(task_info_t& taskInfo, VehicleControllerBase& vehicleController, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds)
{
    static VehicleControllerTask vehicleControllerTask(taskIntervalMicroSeconds, vehicleController);
    vehicleController.setTask(&vehicleControllerTask);

#if defined(USE_FREERTOS)
    Serial.printf("**** VehicleControllerTask,  core:%u, priority:%u, task interval:%ums\r\n", coreID, priority, taskIntervalMicroSeconds / 1000);
    static TaskBase::parameters_t taskParameters { // NOLINT(misc-const-correctness) false positive
        .task = &vehicleControllerTask
    };
    enum { TASK_STACK_DEPTH = 4096 };
    static std::array<StackType_t, TASK_STACK_DEPTH> stack;
    static StaticTask_t taskBuffer;
    taskInfo = {
        .taskHandle = nullptr,
        .name = "VehicleTask", // max length 16, including zero terminator
        .stackDepth = TASK_STACK_DEPTH,
        .stackBuffer = &stack[0],
        .priority = priority,
        .coreID = coreID,
    };
    assert(strlen(taskInfo.name) < configMAX_TASK_NAME_LEN && "VehicleControllerTask: taskname too long");
    assert(taskInfo.priority < configMAX_PRIORITIES && "VehicleControllerTask: priority too high");

    taskInfo.taskHandle = xTaskCreateStaticPinnedToCore(
        VehicleControllerTask::Task,
        taskInfo.name,
        taskInfo.stackDepth,
        &taskParameters,
        taskInfo.priority,
        taskInfo.stackBuffer,
        &taskBuffer,
        taskInfo.coreID
    );
    assert(taskInfo.taskHandle != nullptr && "Unable to create MPC_Task.");
#else
    (void)taskInfo;
    (void)priority;
    (void)coreID;
#endif // USE_FREERTOS
    return &vehicleControllerTask;
}

VehicleControllerTask* VehicleControllerTask::createTask(VehicleControllerBase& vehicleController, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds)
{
    static task_info_t taskInfo;
    return createTask(taskInfo, vehicleController, priority, coreID, taskIntervalMicroSeconds);
}
