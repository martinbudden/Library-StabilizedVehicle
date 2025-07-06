#include "VehicleControllerBase.h"
#include "VehicleControllerTask.h"

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


VehicleControllerTask* VehicleControllerTask::createTask(task_info_t& taskInfo, VehicleControllerBase& vehicleController, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds)
{
    static VehicleControllerTask vehicleControllerTask(taskIntervalMicroSeconds, vehicleController);
    vehicleController.setTask(&vehicleControllerTask);

#if defined(USE_FREERTOS)
#if defined(USE_DEBUG_PRINTF_TASK_INFORMATION)
    Serial.printf("**** VehicleControllerTask,  core:%u, priority:%u, task interval:%ums\r\n", coreID, priority, taskIntervalMicroSeconds / 1000);
#endif
    static TaskBase::parameters_t taskParameters { // NOLINT(misc-const-correctness) false positive
        .task = &vehicleControllerTask
    };
#if !defined(VEHICLE_CONTROLLER_TASK_STACK_DEPTH_BYTES)
    enum { VEHICLE_CONTROLLER_TASK_STACK_DEPTH_BYTES = 4096 };
#endif
    static std::array<StackType_t, VEHICLE_CONTROLLER_TASK_STACK_DEPTH_BYTES> stack;
    static StaticTask_t taskBuffer;
    taskInfo = {
        .taskHandle = nullptr,
        .name = "VehicleTask", // max length 16, including zero terminator
        .stackDepth = VEHICLE_CONTROLLER_TASK_STACK_DEPTH_BYTES,
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
    assert(taskInfo.taskHandle != nullptr && "Unable to create VehicleControllerTask.");
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
