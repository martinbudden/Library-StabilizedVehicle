#include "VehicleControllerBase.h"
#include "VehicleControllerTask.h"

#include <array>
#include <cassert>
#include <cstring>

#if defined(FRAMEWORK_USE_FREERTOS)
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/FreeRTOSConfig.h>
#include <freertos/task.h>
#else
#if defined(FRAMEWORK_ARDUINO_STM32)
#include <STM32FreeRTOS.h>
#endif
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <task.h>
#endif
#endif


VehicleControllerTask* VehicleControllerTask::createTask(VehicleControllerBase& vehicleController, uint8_t priority, uint32_t core)
{
    task_info_t taskInfo {};
    return createTask(taskInfo, vehicleController, priority, core);
}

VehicleControllerTask* VehicleControllerTask::createTask(task_info_t& taskInfo, VehicleControllerBase& vehicleController, uint8_t priority, uint32_t core)
{
    static VehicleControllerTask vehicleControllerTask(vehicleController);
    vehicleController.setTask(&vehicleControllerTask);

    static TaskBase::parameters_t taskParameters { // NOLINT(misc-const-correctness) false positive
        .task = &vehicleControllerTask
    };
#if !defined(VEHICLE_CONTROLLER_TASK_STACK_DEPTH_BYTES)
    enum { VEHICLE_CONTROLLER_TASK_STACK_DEPTH_BYTES = 4096 };
#endif
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32) || !defined(FRAMEWORK_USE_FREERTOS)
    static std::array<uint8_t, VEHICLE_CONTROLLER_TASK_STACK_DEPTH_BYTES> stack;
#else
    static std::array <StackType_t, VEHICLE_CONTROLLER_TASK_STACK_DEPTH_BYTES / sizeof(StackType_t)> stack;
#endif
    taskInfo = {
        .taskHandle = nullptr,
        .name = "VehicleTask", // max length 16, including zero terminator
        .stackDepthBytes = VEHICLE_CONTROLLER_TASK_STACK_DEPTH_BYTES,
        .stackBuffer = reinterpret_cast<uint8_t*>(&stack[0]), // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        .priority = priority,
        .core = core,
        .taskIntervalMicroSeconds = 0,
    };

#if defined(FRAMEWORK_USE_FREERTOS)
    assert(std::strlen(taskInfo.name) < configMAX_TASK_NAME_LEN);
    assert(taskInfo.priority < configMAX_PRIORITIES);

    static StaticTask_t taskBuffer;
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
    taskInfo.taskHandle = xTaskCreateStaticPinnedToCore(
        VehicleControllerTask::Task,
        taskInfo.name,
        taskInfo.stackDepthBytes / sizeof(StackType_t),
        &taskParameters,
        taskInfo.priority,
        &stack[0],
        &taskBuffer,
        taskInfo.core
    );
    assert(taskInfo.taskHandle != nullptr && "Unable to create VehicleControllerTask");
#elif defined(FRAMEWORK_RPI_PICO) || defined(FRAMEWORK_ARDUINO_RPI_PICO)
    taskInfo.taskHandle = xTaskCreateStaticAffinitySet(
        VehicleControllerTask::Task,
        taskInfo.name,
        taskInfo.stackDepthBytes / sizeof(StackType_t),
        &taskParameters,
        taskInfo.priority,
        &stack[0],
        &taskBuffer,
        taskInfo.core
    );
    assert(taskInfo.taskHandle != nullptr && "Unable to create VehicleControllerTask");
#else
    taskInfo.taskHandle = xTaskCreateStatic(
        VehicleControllerTask::Task,
        taskInfo.name,
        taskInfo.stackDepthBytes / sizeof(StackType_t),
        &taskParameters,
        taskInfo.priority,
        &stack[0],
        &taskBuffer
    );
    assert(taskInfo.taskHandle != nullptr && "Unable to create VehicleControllerTask");
    // vTaskCoreAffinitySet(taskInfo.taskHandle, taskInfo.core);
#endif
#else
    (void)taskParameters;
#endif // FRAMEWORK_USE_FREERTOS
    return &vehicleControllerTask;
}
