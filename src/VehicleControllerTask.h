#pragma once

#include <TaskBase.h>

class VehicleControllerBase;
class VehicleControllerMessageQueue;
class MotorMixerBase;
class RpmFilters;


class VehicleControllerTask : public TaskBase {
public:
    VehicleControllerTask(VehicleControllerBase& vehicleController, VehicleControllerMessageQueue& vehicleControllerMessageQueue, MotorMixerBase& motorMixer, RpmFilters* rpmFilters);
public:
    static VehicleControllerTask* createTask(task_info_t& taskInfo, VehicleControllerBase& vehicleController, VehicleControllerMessageQueue& vehicleControllerMessageQueue, MotorMixerBase& motorMixer, RpmFilters* rpmFilters, uint8_t priority, uint32_t core);
    static VehicleControllerTask* createTask(VehicleControllerBase& vehicleController, VehicleControllerMessageQueue& vehicleControllerMessageQueue, MotorMixerBase& motorMixer, RpmFilters* rpmFilters, uint8_t priority, uint32_t core);
public:
    [[noreturn]] static void Task(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    uint32_t _taskIntervalMilliseconds;
    VehicleControllerBase& _vehicleController;
    VehicleControllerMessageQueue& _vehicleControllerMessageQueue;
    MotorMixerBase& _motorMixer;
    RpmFilters* _rpmFilters;
};
