# pragma once

#include "AHRS.h"
#include "VehicleControllerMessageQueue.h"
#include <TaskBase.h>


/*!
Abstract base class defining a controller for a stabilized vehicle.
*/
class VehicleControllerBase {
public:
    struct PIDF_uint16_t {
        uint16_t kp;
        uint16_t ki;
        uint16_t kd;
        uint16_t ks;
        uint16_t kk;
    };
    struct PIDF_error_t {
        float P;
        float I;
        float D;
        float S;
        float K;
    };
    enum { TYPE_NOT_SET = 0, SELF_BALANCING_ROBOT = 1, AIRCRAFT = 2 };
public:
    virtual ~VehicleControllerBase() = default;
    VehicleControllerBase(uint32_t type, uint32_t PID_Count, uint32_t taskIntervalMicroseconds) :
        _type(type), _PID_Count(PID_Count), _taskIntervalMicroseconds(taskIntervalMicroseconds)
    {
    }
public:
    inline uint32_t getType() const { return _type; };
    inline uint32_t getPID_Count() const { return _PID_Count; };
    inline void setSensorFusionFilterIsInitializing(bool sensorFusionFilterIsInitializing) { _sensorFusionFilterIsInitializing = sensorFusionFilterIsInitializing; }
    inline uint32_t getTaskIntervalMicroseconds() const { return _taskIntervalMicroseconds; }
    inline const TaskBase* getTask() const { return _task; } //!< Used to get task data for instrumentation
    inline void setTask(const TaskBase* task) { _task = task; }

    inline void SIGNAL(const VehicleControllerMessageQueue::queue_item_t& queueItem) { _messageQueue.SIGNAL(queueItem); }
#if defined(FRAMEWORK_USE_FREERTOS)
    inline void WAIT(VehicleControllerMessageQueue::queue_item_t& queueItem) { _messageQueue.WAIT(queueItem); }
#else
    inline const VehicleControllerMessageQueue::queue_item_t& getMessageQueueItem() const { return _messageQueue.getQueueItem(); }
#endif

    virtual void outputToMixer(float deltaT, uint32_t tickCount, const VehicleControllerMessageQueue::queue_item_t& queueItem) = 0;
    virtual void updateOutputsUsingPIDs(const AHRS::ahrs_data_t& ahrsData) = 0;

    // functions for telemetry/instrumentation, defaulted to do nothing
    virtual uint32_t getOutputPowerTimeMicroseconds() const { return 0; } //!< time taken to write output power to the motors
    virtual PIDF_uint16_t getPID_MSP(size_t index) const { (void)index; return PIDF_uint16_t{}; }
    virtual PIDF_error_t getPID_Error(size_t index) const { (void)index; return PIDF_error_t{}; }
    virtual float getPID_Setpoint(size_t index) const { (void)index; return 0.0F; }
protected:
    const uint32_t _type; //!< used for telemetry data
    const uint32_t _PID_Count; //!< used for telemetry data
    const uint32_t _taskIntervalMicroseconds;
    const TaskBase* _task {nullptr};
    VehicleControllerMessageQueue _messageQueue;
    bool _sensorFusionFilterIsInitializing {true};
};
