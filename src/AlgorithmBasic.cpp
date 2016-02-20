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
    setDesiredLinearVelocity(0.1f);
    setDesiredAngularVelocity(0.1f);

    move();
}
