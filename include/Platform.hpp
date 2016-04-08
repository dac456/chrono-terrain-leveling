#ifndef __PLATFORM_HPP
#define __PLATFORM_HPP

#include "CtlCommon.hpp"
#include "LowpassFilter.hpp"

class Platform{
private:
    TrackedVehiclePtr _vehicle;
    ChBodyPtr _chassis;

    float _desiredLinearVelocity;
    float _desiredAngularVelocity;

    float _rotX;
    LowpassFilter<float> _rotXFilter;

    float _rotZ;
    LowpassFilter<float> _rotZFilter;

public:
    Platform(TrackedVehiclePtr vehicle, float maxLinear = 10.0f, float maxAngular = 0.34f, int maxRpm = 320);
    ~Platform();

    TrackedVehiclePtr getVehicle();
    ChBodyPtr getChassisBody();

    float getAccelYaw();
    float getAccelPitch();
    float getAccelRoll();

    float getGyroX();
    float getGyroY();
    float getGyroZ();

    bool isInclined();

    void move();

    void setDesiredLinearVelocity(float v);
    void setDesiredAngularVelocity(float v);
    float getDesiredLinearVelocity();
    float getDesiredAngularVelocity();

	void step(float dt);

    void sense(float dt);
    void act(float dt);

    virtual void senseImpl(float dt){};
    virtual void actImpl(float dt){};
};

#endif
