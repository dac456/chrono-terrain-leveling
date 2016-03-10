#include "AlgorithmBasic.hpp"

AlgorithmBasic::AlgorithmBasic(TrackedVehiclePtr vehicle)
    : Platform(vehicle)
{


}

AlgorithmBasic::~AlgorithmBasic() {

}

void AlgorithmBasic::senseImpl(float dt) {

}

void AlgorithmBasic::actImpl(float dt) {
    /*if(isInclined()){
        setDesiredLinearVelocity(3.0f);
        setDesiredAngularVelocity(0.0f);
    }
    else{
        setDesiredLinearVelocity(2.0f);
        if(ChRandom() < 0.25) setDesiredAngularVelocity(ChRandom() * 1.57);
        else setDesiredAngularVelocity(0.0f);
    }*/
    setDesiredLinearVelocity(4.0f);
    setDesiredAngularVelocity(0.0f);

    move();
}
