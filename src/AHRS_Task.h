#pragma once

#include <task_base.h>

class AHRS;
class AHRS_MessageQueue;
class IMU_FiltersBase;
class MotorMixerMessageQueue;
class VehicleControllerBase;

struct ahrs_task_parameters_t {
    AHRS& ahrs;
    AHRS_MessageQueue& ahrs_message_queue;
    IMU_FiltersBase& imu_filters;
    VehicleControllerBase& vehicle_controller;
    MotorMixerMessageQueue& motor_mixer_message_queue;
};

class AHRS_Task : public TaskBase {
public:
    AHRS_Task(uint32_t task_interval_microseconds, const ahrs_task_parameters_t& parameters) :
        TaskBase(task_interval_microseconds),
        _task(parameters)
        {}
public:
    static AHRS_Task* create_task(task_info_t& task_info, const ahrs_task_parameters_t& parameters, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds);
    static AHRS_Task* create_task(const ahrs_task_parameters_t& parameters, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds);
public:
    [[noreturn]] static void task_static(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    ahrs_task_parameters_t _task;
};
