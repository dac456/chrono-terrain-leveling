#include "Platform.hpp"
#include "TrackedVehicle.hpp"

Platform::Platform(std::shared_ptr<TrackedVehicle> vehicle, std::shared_ptr<vehicle::m113::M113_SimplePowertrain> powertrain, std::shared_ptr<vehicle::DeformableTerrain> terrain, float maxLinear, float maxAngular, int maxRpm)
    : _vehicle(vehicle)
    , _powertrain(powertrain)
    //, _chassis(vehicle->GetChassis())
    , _chassis(vehicle->getChassisBody())
    , _terrain(terrain)
      //_vehicle("M113/vehicle/M113_vehicle->json", ChMaterialSurfaceBase::DEM)
    , _rotXFilter(0.98f)
    , _rotZFilter(0.98f)
{

}

Platform::~Platform(){

}

std::shared_ptr<TrackedVehicle> Platform::getVehicle(){
    return _vehicle;
}

ChBodyPtr Platform::getChassisBody(){
    return _chassis;
    //return _chassis->GetBody();
}

/*
 Get vehicle yaw angle as would be reported by the accelerometer
*/
float Platform::getAccelYaw(){
    //ChFrameMoving<> chassisFrame = _chassis->GetFrame_COG_to_abs();
    //ChQuatd orientation = chassisFrame.GetRot();
    ChQuatd orientation = _chassis->GetRot();

    return orientation.Q_to_NasaAngles().x; //heading
}

/*
 Get vehicle pitch angle as would be reported by the accelerometer
*/
float Platform::getAccelPitch(){
    //ChFrameMoving<> chassisFrame = _chassis->GetFrame_COG_to_abs();
    //ChQuatd orientation = chassisFrame.GetRot();
    //ChQuatd orientation = _chassis->GetRot();
    ChQuatd orientation = _chassis->GetCoord().rot;
    //ChQuatd orientation = _chassis->GetRot();

    return orientation.Q_to_NasaAngles().z; //attitude
}

/*
* Get vehicle roll angle as would be reported by the accelerometer
*/
float Platform::getAccelRoll(){
    ChFrameMoving<> chassisFrame = _chassis->GetFrame_COG_to_abs();
    ChQuatd orientation = chassisFrame.GetRot();
    //ChQuatd orientation = _chassis->GetRot();

    return orientation.Q_to_NasaAngles().y; //bank
}

/*
 Get the angular velocity about the X axis as would be reported by the gyro (dps)
*/
float Platform::getGyroX(){
    ChFrameMoving<> chassisFrame = _chassis->GetFrame_COG_to_abs();
    ChQuatd rotationVelocity = chassisFrame.GetRot_dt();
    //ChQuatd rotationVelocity = _chassis->GetBody()->GetFrame_COG_to_abs().GetRot_dt();

    return (rotationVelocity.Q_to_NasaAngles().y * 180.0f) / CH_C_PI;
}

/*
 Get the angular velocity about the Y axis as would be reported by the gyro (dps)
*/
float Platform::getGyroY(){
    ChFrameMoving<> chassisFrame = _chassis->GetFrame_COG_to_abs();
    ChQuatd rotationVelocity = chassisFrame.GetRot_dt();
    //ChQuatd rotationVelocity = _chassis->GetBody()->GetFrame_COG_to_abs().GetRot_dt();

    return (rotationVelocity.Q_to_NasaAngles().z * 180.0f) / CH_C_PI;
}

/*
 Get the angular velocity about the Z axis as would be reported by the gyro (dps)
*/
float Platform::getGyroZ(){
    ChFrameMoving<> chassisFrame = _chassis->GetFrame_COG_to_abs();
    ChQuatd rotationVelocity = chassisFrame.GetRot_dt();
    //ChQuatd rotationVelocity = _chassis->GetBody()->GetFrame_COG_to_abs().GetRot_dt();

    return (rotationVelocity.Q_to_NasaAngles().x * 180.0f) / CH_C_PI;
}

/*
Returns true if the pitch of the vehicle is above some threshold
*/
bool Platform::isInclined(){
    if(fabs((this->getAccelPitch()*180.0) / CH_C_PI) > 20.0) return true;
    else return false;
}

/*
Set motor speeds based on currently set desired linear/angular velocities
*/
void Platform::move(double dt) {
    //const float r = _vehicle->getWheelRadius(); //wheel radius (m)
    //const float L = _vehicle->getWheelBase(); //wheel base (m)
    const float r = 0.5;
    const float L = 3.1;

    float vLeft = (_desiredLinearVelocity - (L*_desiredAngularVelocity*0.5f)) / r;
    float vRight = (_desiredLinearVelocity + (L*_desiredAngularVelocity*0.5f)) / r;

    std::cout << "Move: " << vLeft << " " << vRight << std::endl;
    _vehicle->setSpeeds(vLeft, vRight);

    // Collect output data from modules (for inter-module communication)
    /*(vehicle::BodyStates shoe_states_left(_vehicle->GetNumTrackShoes(vehicle::LEFT));
    vehicle::BodyStates shoe_states_right(_vehicle->GetNumTrackShoes(vehicle::RIGHT));
    vehicle::TrackShoeForces shoe_forces_left(_vehicle->GetNumTrackShoes(vehicle::LEFT));
    vehicle::TrackShoeForces shoe_forces_right(_vehicle->GetNumTrackShoes(vehicle::RIGHT));
    double powertrain_torque = _powertrain->GetOutputTorque();
    double driveshaft_speed = _vehicle->GetDriveshaftSpeed();
    _vehicle->GetTrackShoeStates(vehicle::LEFT, shoe_states_left);
    _vehicle->GetTrackShoeStates(vehicle::RIGHT, shoe_states_right);

    // Update modules (process inputs from other modules)
    double time = _vehicle->GetSystem()->GetChTime();
    //driver.Synchronize(time);
    _powertrain->Synchronize(time, 0.75, driveshaft_speed);
    _vehicle->Synchronize(time, 0.0, 0.0, powertrain_torque, shoe_forces_left, shoe_forces_right);
    _terrain->Synchronize(time);

    // Advance simulation for one timestep for all modules
    //driver.Advance(step_size);
    _powertrain->Advance(dt);
    _vehicle->Advance(dt);
    _terrain->Advance(dt);*/
}

void Platform::warpToRelativePosition(ChVectord p) {
    //ChFrameMoving<> frame(p, QUNIT);
    //_chassis->GetBody()->ConcatenatePreTransformation(frame);
    _vehicle->warpToRelativePosition(p);
}

void Platform::setDesiredLinearVelocity(float v) {
    _desiredLinearVelocity = v;
}

void Platform::setDesiredAngularVelocity(float v) {
    _desiredAngularVelocity = v;
}

float Platform::getDesiredLinearVelocity(){
    return _desiredLinearVelocity;
}

float Platform::getDesiredAngularVelocity(){
    return _desiredAngularVelocity;
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

    std::cout << "Pitch: " << getAccelPitch() << "(" << (getAccelPitch()*180.0)/CH_C_PI << ")" << std::endl;
    std::cout << "Roll: " << getAccelRoll() << "(" << (getAccelRoll()*180.0)/CH_C_PI << ")" << std::endl;
    std::cout << "Yaw: " << getAccelYaw() << "(" << (getAccelYaw()*180.0)/CH_C_PI << ")" << std::endl;

    senseImpl(dt);
}

void Platform::act(float dt){
    actImpl(dt);
}
