#pragma once

#include <task_base.h>

class Ahrs;
class AhrsMessageQueue;
class Debug;
class ImuFiltersBase;
class MotorMixerMessageQueue;
class VehicleControllerBase;

struct ahrs_parameter_group_t {
    Ahrs& ahrs;
    AhrsMessageQueue& ahrs_message_queue;
    ImuFiltersBase& imu_filters;
    VehicleControllerBase& vehicle_controller;
    MotorMixerMessageQueue& motor_mixer_message_queue;
    Debug& debug;
};

class AhrsTask : public TaskBase {
public:
    AhrsTask(uint32_t task_interval_microseconds, const ahrs_parameter_group_t& parameter_group) :
        TaskBase(task_interval_microseconds),
        _parameter_group(parameter_group)
        {}
public:
    static AhrsTask* create_task(task_info_t& task_info, const ahrs_parameter_group_t& parameter_group, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds);
    static AhrsTask* create_task(const ahrs_parameter_group_t& parameter_group, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds);
public:
    [[noreturn]] static void task_static(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    ahrs_parameter_group_t _parameter_group;
};
