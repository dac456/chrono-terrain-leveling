#ifndef __PLATFORM_HPP
#define __PLATFORM_HPP

#include "CtlCommon.hpp"
#include "LowpassFilter.hpp"

class Platform{
private:
    ChBodyPtr _chassis;

    float _rotX;
    LowpassFilter<float> _rotXFilter;

public:
    Platform(TrackedVehiclePtr vehicle, float maxLinear = 10.0f, float maxAngular = 0.34f, int maxRpm = 320);
    ~Platform();

    float getYaw();
    float getPitch();
    float getRoll();

    float getRotX();
    float getRotY();
    float getRotZ();

	void step(uint16_t dt);

    void sense(uint16_t dt);
    void act(uint16_t dt);

    virtual void senseImpl(uint16_t dt) = 0;
    virtual void actImpl(uint16_t dt) = 0;
};

#endif
