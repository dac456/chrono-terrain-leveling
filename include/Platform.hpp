#ifndef __PLATFORM_HPP
#define __PLATFORM_HPP

#include "CtlCommon.hpp"
#include "LowpassFilter.hpp"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_vehicle/powertrain/SimplePowertrain.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/terrain/DeformableTerrain.h"
#include "chrono_vehicle/tracked_vehicle/vehicle/TrackedVehicle.h"
#include "chrono_vehicle/chassis/RigidChassis.h"

#include "chrono_models/vehicle/m113/M113_SimplePowertrain.h"
#include "chrono_models/vehicle/m113/M113_Vehicle.h"

class Platform{
private:
    //TrackedVehiclePtr _vehicle;
    std::shared_ptr<TrackedVehicle> _vehicle;
    std::shared_ptr<vehicle::m113::M113_SimplePowertrain> _powertrain;
    //std::shared_ptr<vehicle::ChChassis> _chassis;
    ChBodyPtr _chassis;
    std::shared_ptr<vehicle::DeformableTerrain> _terrain;

    float _desiredLinearVelocity;
    float _desiredAngularVelocity;

    float _rotX;
    LowpassFilter<float> _rotXFilter;

    float _rotZ;
    LowpassFilter<float> _rotZFilter;

public:
    Platform(std::shared_ptr<TrackedVehicle> vehicle, std::shared_ptr<vehicle::m113::M113_SimplePowertrain> powertrain, std::shared_ptr<vehicle::DeformableTerrain> terrain, float maxLinear = 10.0f, float maxAngular = 0.34f, int maxRpm = 320);
    ~Platform();

    std::shared_ptr<TrackedVehicle> getVehicle();
    ChBodyPtr getChassisBody();

    float getAccelYaw();
    float getAccelPitch();
    float getAccelRoll();

    float getGyroX();
    float getGyroY();
    float getGyroZ();

    bool isInclined();

    void move(double dt);

    void warpToRelativePosition(ChVectord p);

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
