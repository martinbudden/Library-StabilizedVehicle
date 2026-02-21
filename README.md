# Stabilized Vehicle Library ![license](https://img.shields.io/badge/license-MIT-green) ![open source](https://badgen.net/badge/open/source/blue?icon=github)

This library contains a number of classes that can be the basis for a stabilized vehicle, such as a self-balancing robot or an aircraft.

## Simplified Class Diagram

The main work of the `AhrsTask` is done in the `Ahrs::read_imu_and_update_orientation` function.<br>
This reads the IMU, filters the reading, applies sensor fusion, updates the blackbox, and then calls `VehicleControllerBase::update_outputs_using_pids`.

`update_outputs_using_pids` uses the PID controllers to calculate the new motor outputs.

The `VehicleController_Task` reads these calculated motor values and outputs them to the motors, via the `MotorMixer`.

```mermaid
classDiagram
    class TaskBase:::taskClass {
        _task_interval_microseconds
    }
    link TaskBase "https://github.com/martinbudden/Library-TaskBase/blob/main/src/task_base.h"

    TaskBase <|-- AhrsTask
    class AhrsTask:::taskClass {
        loop()
        -task() [[noreturn]]
    }
    link AhrsTask "https://github.com/martinbudden/Library-StabilizedVehicle/blob/main/src/ahrs_task.h"
    AhrsTask o-- AHRS : calls read_imu_and_update_orientation
    AhrsTask o-- IMU_Base : calls WAIT_IMU_DATA_READY


    class IMU_Base {
        <<abstract>>
        WAIT_IMU_DATA_READY()
        virtual readAccGyroRPS() acc_gyro_rps_t
    }
    link IMU_Base "https://github.com/martinbudden/Library-Sensors/blob/main/src/imu_base.h"

    class ImuFiltersBase {
        <<abstract>>
        filter() *
    }
    link ImuFiltersBase "https://github.com/martinbudden/Library-StabilizedVehicle/blob/main/src/imu_filters_base.h"

    class SensorFusionFilterBase {
        <<abstract>>
        updateOrientation() Quaternion *
    }
    link SensorFusionFilterBase "https://github.com/martinbudden/Library-SensorFusion/blob/main/src/sensor_fusion.h"

    class VehicleControllerMessageQueue {
        WAIT()
        SIGNAL()
    }
    link VehicleControllerMessageQueue "https://github.com/martinbudden/Library-StabilizedVehicle/blob/main/src/VehicleControllerMessageQueue.h"

    class VehicleControllerBase {
        <<abstract>>
        WAIT()
        SIGNAL()
        outputToMixer() *
        update_outputs_using_pids() *
    }
    link VehicleControllerBase "https://github.com/martinbudden/Library-StabilizedVehicle/blob/main/src/vehicle_controller_base.h"
    VehicleControllerBase *-- VehicleControllerMessageQueue : calls WAIT / SIGNAL

    class Ahrs {
        _acc_gyro_rps acc_gyro_rps_t
        _orientation Quaternion
        read_imu_and_update_orientation() bool
    }
    link AHRS "https://github.com/martinbudden/Library-StabilizedVehicle/blob/main/src/AHRS.h"
    AHRS o-- IMU_Base : calls readAccGyroRPS
    AHRS o-- ImuFiltersBase : calls filter
    AHRS o-- SensorFusionFilterBase : calls updateOrientation
    AHRS o-- VehicleControllerBase : calls update_outputs_using_pids / SIGNAL

    TaskBase <|-- VehicleControllerTask
    class VehicleControllerTask:::taskClasss {
        loop()
        -task() [[noreturn]]
    }
    link VehicleControllerTask "https://github.com/martinbudden/Library-StabilizedVehicle/blob/main/src/VehicleControllerTask.h"
    VehicleControllerTask o-- VehicleControllerBase : calls WAIT / outputToMixer

    classDef taskClass fill:#f96
```
