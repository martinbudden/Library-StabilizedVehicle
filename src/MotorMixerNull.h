#pragma once

#include "MotorMixerBase.h"

class MotorMixerNull : public MotorMixerBase {
public:
    MotorMixerNull() : MotorMixerBase(0) {}
public:
    virtual void outputToMotors(const commands_t& commands, float deltaT, uint32_t tickCount) override;
    virtual float getMotorOutput(size_t motorIndex) const override;
    virtual int32_t getMotorRPM(size_t motorIndex) const override;
};
