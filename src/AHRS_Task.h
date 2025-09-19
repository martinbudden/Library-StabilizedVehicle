#pragma once

#include <TaskBase.h>

class AHRS;


class AHRS_Task : public TaskBase {
public:
    AHRS_Task(uint32_t taskIntervalMicroseconds, AHRS& ahrs) :
        TaskBase(taskIntervalMicroseconds),
        _ahrs(ahrs) {}
public:
    static AHRS_Task* createTask(task_info_t& taskInfo, AHRS& ahrs, uint8_t priority, uint32_t core, uint32_t taskIntervalMicroseconds);
    static AHRS_Task* createTask(AHRS& ahrs, uint8_t priority, uint32_t core, uint32_t taskIntervalMicroseconds);
public:
    [[noreturn]] static void Task(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    AHRS& _ahrs;
};
