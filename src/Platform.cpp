#include "Platform.hpp"
#include "TrackedVehicle.hpp"

Platform::Platform(TrackedVehiclePtr vehicle, float maxLinear, float maxAngular, int maxRpm)
    : _vehicle(vehicle),
      _chassis(vehicle->getChassisBody()),
      _rotXFilter(0.98f),
      _rotZFilter(0.98f)
{

}

Platform::~Platform(){

}

/*
 Get vehicle yaw angle as would be reported by the accelerometer
*/
float Platform::getAccelYaw(){
    ChFrameMoving<> chassisFrame = _chassis->GetFrame_COG_to_abs();
    ChQuatd orientation = chassisFrame.GetRot();

    return orientation.Q_to_NasaAngles().x;
}

/*
 Get vehicle pitch angle as would be reported by the accelerometer
*/
float Platform::getAccelPitch(){
    ChFrameMoving<> chassisFrame = _chassis->GetFrame_COG_to_abs();
    ChQuatd orientation = chassisFrame.GetRot();

    return orientation.Q_to_NasaAngles().z;
}

/*
* Get vehicle roll angle as would be reported by the accelerometer
*/
float Platform::getAccelRoll(){
    ChFrameMoving<> chassisFrame = _chassis->GetFrame_COG_to_abs();
    ChQuatd orientation = chassisFrame.GetRot();

    return orientation.Q_to_NasaAngles().y;
}

/*
 Get the angular velocity about the X axis as would be reported by the gyro (dps)
*/
float Platform::getGyroX(){
    ChFrameMoving<> chassisFrame = _chassis->GetFrame_COG_to_abs();
    ChQuatd rotationVelocity = chassisFrame.GetRot_dt();

    return (rotationVelocity.Q_to_NasaAngles().y * 180.0f) / CH_C_PI;
}

/*
 Get the angular velocity about the Y axis as would be reported by the gyro (dps)
*/
float Platform::getGyroY(){
    ChFrameMoving<> chassisFrame = _chassis->GetFrame_COG_to_abs();
    ChQuatd rotationVelocity = chassisFrame.GetRot_dt();

    return (rotationVelocity.Q_to_NasaAngles().z * 180.0f) / CH_C_PI;
}

/*
 Get the angular velocity about the Z axis as would be reported by the gyro (dps)
*/
float Platform::getGyroZ(){
    ChFrameMoving<> chassisFrame = _chassis->GetFrame_COG_to_abs();
    ChQuatd rotationVelocity = chassisFrame.GetRot_dt();

    return (rotationVelocity.Q_to_NasaAngles().x * 180.0f) / CH_C_PI;
}

/*
Set motor speeds based on currently set desired linear/angular velocities
*/
void Platform::move() {
    const float r = _vehicle->getWheelRadius(); //wheel radius (m)
    const float L = _vehicle->getWheelBase(); //wheel base (m)

    float vLeft = (_desiredLinearVelocity - (L*_desiredAngularVelocity*0.5f)) / r;
    float vRight = (_desiredLinearVelocity + (L*_desiredAngularVelocity*0.5f)) / r;

    std::cout << "Move: " << vLeft << " " << vRight << std::endl;
    _vehicle->setSpeeds(vLeft, vRight);
}

void Platform::setDesiredLinearVelocity(float v) {
    _desiredLinearVelocity = v;
}

void Platform::setDesiredAngularVelocity(float v) {
    _desiredAngularVelocity = v;
}


void Platform::step(float dt) {
    sense(dt);
    act(dt);
}

void Platform::sense(float dt) {
    _rotX = _rotXFilter.getFilteredValue(this->getGyroX());
    std::cout << _rotX << std::endl;

    _rotZ = _rotZFilter.getFilteredValue(this->getGyroZ());
    std::cout << _rotZ << std::endl;

    senseImpl(dt);
}

void Platform::act(float dt){
    actImpl(dt);
}
