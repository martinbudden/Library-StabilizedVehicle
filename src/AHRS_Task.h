#pragma once

#include <TaskBase.h>

class AHRS;
class AHRS_MessageQueue;
class IMU_FiltersBase;
class MotorMixerMessageQueue;
class VehicleControllerBase;

struct ahrs_task_parameters_t {
    AHRS& ahrs;
    AHRS_MessageQueue& ahrsMessageQueue;
    IMU_FiltersBase& imuFilters;
    VehicleControllerBase& vehicleController;
    MotorMixerMessageQueue& motor_mixer_message_queue;
};

class AHRS_Task : public TaskBase {
public:
    AHRS_Task(uint32_t taskIntervalMicroseconds, const ahrs_task_parameters_t& parameters) :
        TaskBase(taskIntervalMicroseconds),
        _task(parameters)
        {}
public:
    static AHRS_Task* createTask(task_info_t& taskInfo, const ahrs_task_parameters_t& parameters, uint8_t priority, uint32_t core, uint32_t taskIntervalMicroseconds);
    static AHRS_Task* createTask(const ahrs_task_parameters_t& parameters, uint8_t priority, uint32_t core, uint32_t taskIntervalMicroseconds);
public:
    [[noreturn]] static void Task(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    ahrs_task_parameters_t _task;
};
