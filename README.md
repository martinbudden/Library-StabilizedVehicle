# Stabilized Vehicle Library ![license](https://img.shields.io/badge/license-MIT-green) ![open source](https://badgen.net/badge/open/source/blue?icon=github)

This library contains a number of classes that can be the basis for a stabilized vehicle, such as a self-balancing robot or an aircraft.

## Simplified Class Diagram

The main work of the `AHRS_Task` is done in the `AHRS::readIMUandUpdateOrientation` function.<br>
This reads the IMU, filters the reading, applies sensor fusion, updates the blackbox, and then calls `VehicleControllerBase::updateOutputsUsingPIDs`.

`updateOutputsUsingPIDs` uses the PID controllers to calculate the new motor outputs.

The `VehicleController_Task` reads these calculated motor values and outputs them to the motors, via the `MotorMixer`.

```mermaid
classDiagram
    class IMU_Base {
        <<abstract>>
        virtual readAccGyroRPS() accGyroRPS_t
    }
    link IMU_Base "https://github.com/martinbudden/Library-IMU/blob/main/src/IMU_Base.h"

    class IMU_FiltersBase {
        <<abstract>>
        setFilters() *
        filter() *
    }
    link IMU_FiltersBase "https://github.com/martinbudden/Library-StabilizedVehicle/blob/main/src/IMU_FiltersBase.h"

    class SensorFusionFilterBase {
        <<abstract>>
        update() Quaternion *
        getOrientation() const Quaternion
    }
    link SensorFusionFilterBase "https://github.com/martinbudden/Library-SensorFusion/blob/main/src/SensorFusion.h"

    class VehicleControllerBase {
        <<abstract>>
        loop()
        updateOutputsUsingPIDs() *
    }
    link VehicleControllerBase "https://github.com/martinbudden/Library-StabilizedVehicle/blob/main/src/VehicleControllerBase.h"

    class AHRS_MessageQueueBase {
        <<abstract>>
        append() *
    }
    link AHRS_MessageQueueBase "https://github.com/martinbudden/Library-StabilizedVehicle/blob/main/src/AHRS_MessageQueueBase.h"
    AHRS_MessageQueueBase o-- BlackboxMessageQueue : (indirect) adds to blackbox message queue

    class AHRS {
        _accGyroRPS accGyroRPS_t
        _orientation Quaternion
        readIMUandUpdateOrientation() bool
    }
    link AHRS "https://github.com/martinbudden/Library-StabilizedVehicle/blob/main/src/AHRS.h"
    AHRS o-- IMU_Base : calls readAccGyroRPS
    AHRS o-- IMU_FiltersBase : calls filter
    AHRS o-- SensorFusionFilterBase : calls update
    AHRS o-- AHRS_MessageQueueBase : calls append
    AHRS o-- VehicleControllerBase : calls updateOutputsUsingPIDs
    AHRS --o VehicleControllerBase : historical

    class TaskBase {
        _taskIntervalMicroSeconds uint32_t
    }
    link TaskBase "https://github.com/martinbudden/Library-TaskBase/blob/main/src/TaskBase.h"

    TaskBase <|-- AHRS_Task
    class AHRS_Task {
        loop()
        -task() [[noreturn]]
    }
    link AHRS_Task "https://github.com/martinbudden/Library-StabilizedVehicle/blob/main/src/AHRS_Task.h"
    AHRS_Task o-- AHRS : calls readIMUandUpdateOrientation

    TaskBase <|-- VehicleControllerTask
    class VehicleControllerTask {
        loop()
        -task() [[noreturn]]
    }
    link VehicleControllerTask "https://github.com/martinbudden/Library-StabilizedVehicle/blob/main/src/VehicleControllerTask.h"
    VehicleControllerTask o-- VehicleControllerBase : calls loop

    class MotorMixerBase {
        <<abstract>>
        outputToMotors() *
    }
    link MotorMixerBase "https://github.com/martinbudden/Library-StabilizedVehicle/blob/main/src/MotorMixerBase.h"
    VehicleControllerBase -- MotorMixerBase : (indirect) calls outputToMotors
```
