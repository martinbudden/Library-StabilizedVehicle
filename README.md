# Stabilized Vehicle Library ![license](https://img.shields.io/badge/license-MIT-green) ![open source](https://badgen.net/badge/open/source/blue?icon=github)

This library contains a number of classes that can be the basis for a stabilized vehicle, such as a self-balancing robot or an aircraft.

## StabilizedVehicle Classes

```mermaid
classDiagram
    class IMU_Base {
        virtual accGyroRPS_t readAccGyroRPS()
    }

    class IMU_FiltersBase {
        virtual void setFilters() = 0
        virtual void filter() = 0
    }

    class SensorFusionFilterBase {
        virtual Quaternion update(const xyz_t& gyroRPS, const xyz_t& acc, float deltaT) = 0
        Quaternion getOrientation() const
    }

    class VehicleControllerBase {
        virtual void updateOutputsUsingPIDs() = 0
        virtual uint32_t updateBlackbox() = 0
    }
    VehicleControllerBase o-- AHRS

    class AHRS {
        accGyroRPS_t _accGyroRPS
        Quaternion _orientation
        bool readIMUandUpdateOrientation()
    }
    AHRS *-- IMU_Base
    AHRS *-- IMU_FiltersBase
    AHRS *-- SensorFusionFilterBase
    AHRS o-- VehicleControllerBase

    class TaskBase {
        uint32_t _taskIntervalMicroSeconds
    }

    TaskBase <|-- AHRS_Task
    class AHRS_Task {
        void loop()
    }
    AHRS_Task o-- AHRS

    TaskBase <|-- VehicleControllerTask
    class VehicleControllerTask {
        void loop()
    }
    VehicleControllerTask o-- VehicleControllerBase
```

### MotorMixer

```mermaid
classDiagram
    class MotorMixerBase {
        void outputToMotors()
        float getMotorOutput(size_t motorIndex) const
        int32_t getMotorRPM(size_t motorIndex) const
        float getMotorFrequencyHz(size_t motorIndex) const
    }
```
