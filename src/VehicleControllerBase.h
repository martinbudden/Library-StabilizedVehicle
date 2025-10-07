# pragma once

#include "VehicleControllerMessageQueue.h"
#include <TaskBase.h>
#include <cstddef>

class AHRS;
class Quaternion;
struct xyz_t;

/*!
Abstract base class defining a controller for a stabilized vehicle.
*/
class VehicleControllerBase {
public:
    struct PIDF_uint16_t {
        uint16_t kp;
        uint16_t ki;
        uint16_t kd;
        uint16_t kf;
        uint16_t ks;
    };
    enum { TYPE_NOT_SET= 0, SELF_BALANCING_ROBOT = 1, AIRCRAFT = 2 };
public:
    VehicleControllerBase(uint32_t type, uint32_t PID_Count, uint32_t taskIntervalMicroseconds, AHRS& ahrs) :
        _type(type), _PID_Count(PID_Count), _taskIntervalMicroseconds(taskIntervalMicroseconds), _ahrs(ahrs)
    {
        _ahrs.setVehicleController(this);
    }
public:
    inline uint32_t getType() const { return _type; };
    inline uint32_t getPID_Count() const { return _PID_Count; };
    inline uint32_t getTaskIntervalMicroseconds() const { return _taskIntervalMicroseconds; }
    inline const TaskBase* getTask() const { return _task; } //!< Used to get task data for instrumentation
    inline void setTask(const TaskBase* task) { _task = task; }
    inline float getPitchAngleDegreesRaw() const { return _pitchAngleDegreesRaw; }
    inline float getRollAngleDegreesRaw() const { return _rollAngleDegreesRaw; }
    inline float getYawAngleDegreesRaw() const { return _yawAngleDegreesRaw; }
    inline const VehicleControllerMessageQueue::queue_item_t& getMessageQueueItem() const { return _messageQueue.getQueueItem(); }
    inline void SIGNAL(const VehicleControllerMessageQueue::queue_item_t& queueItem) { _messageQueue.SIGNAL(queueItem); }
    inline void WAIT() { _messageQueue.WAIT(); }
    inline void PEEK() { _messageQueue.PEEK(); }

    virtual void outputToMixer(float deltaT, uint32_t tickCount, const VehicleControllerMessageQueue::queue_item_t& queueItem) = 0;
    virtual void updateOutputsUsingPIDs(const xyz_t& gyroRPS, const xyz_t& acc, const Quaternion& orientation, float deltaT) = 0;

    virtual uint32_t getOutputPowerTimeMicroseconds() const = 0; //<! time taken to write output power to the motors, for instrumentation
    virtual PIDF_uint16_t getPID_MSP(size_t index) const = 0;
protected:
    const uint32_t _type;
    const uint32_t _PID_Count;
    uint32_t _taskIntervalMicroseconds;
    AHRS& _ahrs; //<! AHRS which uses ENU (East North Up) coordinate convention
    const TaskBase* _task {nullptr};
    VehicleControllerMessageQueue _messageQueue;
    float _pitchAngleDegreesRaw {0.0F};
    float _rollAngleDegreesRaw {0.0F};
    float _yawAngleDegreesRaw {0.0F};
};
