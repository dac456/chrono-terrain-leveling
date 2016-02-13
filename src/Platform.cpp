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

}

float Platform::getRoll(){

}
