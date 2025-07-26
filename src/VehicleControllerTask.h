#pragma once

#include <TaskBase.h>

class VehicleControllerBase;

class VehicleControllerTask : public TaskBase {
public:
    explicit VehicleControllerTask(VehicleControllerBase& vehicleController) :
        TaskBase(vehicleController.getTaskIntervalMicroSeconds()),
        _vehicleController(vehicleController) {}
public:
    static VehicleControllerTask* createTask(task_info_t& taskInfo, VehicleControllerBase& vehicleController, uint8_t priority, uint8_t coreID);
    static VehicleControllerTask* createTask(VehicleControllerBase& vehicleController, uint8_t priority, uint8_t coreID);
public:
    [[noreturn]] static void Task(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    VehicleControllerBase& _vehicleController;
};
