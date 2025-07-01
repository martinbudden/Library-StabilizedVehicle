#include "MotorMixerNull.h"

void MotorMixerNull::outputToMotors(const commands_t& commands, float deltaT, uint32_t tickCount)
{
    (void) commands;
    (void) deltaT;
    (void) tickCount;
}

float MotorMixerNull::getMotorOutput(size_t motorIndex) const
{
    (void)motorIndex;
    return 0.0F;
}

int32_t MotorMixerNull::getMotorRPM(size_t motorIndex) const
{
    (void)motorIndex;
    return 0;
}
