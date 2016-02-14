#include "Platform.hpp"
#include "TrackedVehicle.hpp"

Platform::Platform(TrackedVehiclePtr vehicle, float maxLinear, float maxAngular, int maxRpm)
    : _chassis(vehicle->getChassisBody())
{

}

Platform::~Platform(){

}

float Platform::getYaw(){
    ChFrameMoving<> chassisFrame = _chassis->GetFrame_COG_to_abs();
    ChQuatd orientation = chassisFrame.GetRot();

    return orientation.Q_to_NasaAngles().x;
}

float Platform::getPitch(){
    ChFrameMoving<> chassisFrame = _chassis->GetFrame_COG_to_abs();
    ChQuatd orientation = chassisFrame.GetRot();

    return orientation.Q_to_NasaAngles().z;
}

float Platform::getRoll(){
    ChFrameMoving<> chassisFrame = _chassis->GetFrame_COG_to_abs();
    ChQuatd orientation = chassisFrame.GetRot();

    return orientation.Q_to_NasaAngles().y;
}

float Platform::getRotX(){
    ChFrameMoving<> chassisFrame = _chassis->GetFrame_COG_to_abs();
    ChQuatd rotationVelocity = chassisFrame.GetRot_dt();

    return rotationVelocity.Q_to_NasaAngles().y;
}

float Platform::getRotY(){
    ChFrameMoving<> chassisFrame = _chassis->GetFrame_COG_to_abs();
    ChQuatd rotationVelocity = chassisFrame.GetRot_dt();

    return rotationVelocity.Q_to_NasaAngles().z;
}

float Platform::getRotZ(){
    ChFrameMoving<> chassisFrame = _chassis->GetFrame_COG_to_abs();
    ChQuatd rotationVelocity = chassisFrame.GetRot_dt();

    return rotationVelocity.Q_to_NasaAngles().x;
}
