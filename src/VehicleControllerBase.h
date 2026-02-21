# pragma once

#include <cstddef>
#include <cstdint>

struct ahrs_data_t;

class AHRS_MessageQueue;
class MotorMixerBase;
class MotorMixerMessageQueue;
class RpmFilters;
class TaskBase;


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
    VehicleControllerBase(uint32_t type, uint32_t pid_count, uint32_t task_interval_microseconds) :
        _type(type), _pid_count(pid_count), _task_interval_microseconds(task_interval_microseconds)
    {
    }
public:
    inline uint32_t get_type() const { return _type; };
    inline uint32_t get_pid_count() const { return _pid_count; };
    inline void set_sensor_fusion_filter_is_initializing(bool sensor_fusion_filter_is_initializing) { _sensor_fusion_filter_is_initializing = sensor_fusion_filter_is_initializing; }
    inline uint32_t get_task_interval_microseconds() const { return _task_interval_microseconds; }
    inline const TaskBase* get_task() const { return _task; } //!< Used to get task data for instrumentation
    inline void set_task(const TaskBase* task) { _task = task; }

    virtual void update_outputs_using_pids(const ahrs_data_t& ahrsData, AHRS_MessageQueue& ahrs_message_queue, MotorMixerMessageQueue& motor_mixer_message_queue) = 0;

    // functions for telemetry/instrumentation, defaulted to do nothing
    virtual uint32_t get_output_power_time_microseconds() const { return 0; } //!< time taken to write output power to the motors
    virtual PIDF_uint16_t get_pid_msp(size_t index) const { (void)index; return PIDF_uint16_t{}; }
    virtual PIDF_error_t get_pid_error(size_t index) const { (void)index; return PIDF_error_t{}; }
    virtual float get_pid_setpoint(size_t index) const { (void)index; return 0.0F; }

protected:
    const uint32_t _type; //!< used for telemetry data
    const uint32_t _pid_count; //!< used for telemetry data
    const uint32_t _task_interval_microseconds;
    const TaskBase* _task {nullptr};
    bool _sensor_fusion_filter_is_initializing {true};
};
