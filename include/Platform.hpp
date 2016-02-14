#ifndef __PLATFORM_HPP
#define __PLATFORM_HPP

#include "CtlCommon.hpp"

class Platform{
private:
    ChBodyPtr _chassis;

public:
    Platform(TrackedVehiclePtr vehicle, float maxLinear = 10.0f, float maxAngular = 0.34f, int maxRpm = 320);
    ~Platform();

    float getYaw();
    float getPitch();
    float getRoll();

    float getRotX();
    float getRotY();
    float getRotZ();
};

#endif
