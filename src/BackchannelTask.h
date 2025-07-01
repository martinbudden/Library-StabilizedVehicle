#pragma once

#include <TaskBase.h>

class BackchannelBase;

class BackchannelTask : public TaskBase {
public:
    BackchannelTask(uint32_t taskIntervalMicroSeconds, BackchannelBase& backchannel) :
        TaskBase(taskIntervalMicroSeconds),
        _backchannel(backchannel) {}
public:
    static BackchannelTask* createTask(task_info_t& taskInfo, BackchannelBase& backchannel, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);
    static BackchannelTask* createTask(BackchannelBase& backchannel, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds);
public:
    [[noreturn]] static void Task(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    BackchannelBase& _backchannel;
};
