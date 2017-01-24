#include "AlgorithmRandom.hpp"

#include <random>
std::default_random_engine generator;
std::uniform_real_distribution<double> distribution(0.25, 1.0);

AlgorithmRandom::AlgorithmRandom(std::shared_ptr<TrackedVehicle> vehicle, std::shared_ptr<vehicle::m113::M113_SimplePowertrain> powertrain, std::shared_ptr<vehicle::DeformableTerrain> terrain)
    : Platform(vehicle, powertrain, terrain)
    , _ticks(0)
    , _rand(0.0)
{


}

AlgorithmRandom::~AlgorithmRandom() {

}

void AlgorithmRandom::senseImpl(float dt) {

}

void AlgorithmRandom::actImpl(float dt) {
    const size_t interval = 2000;

    if(_ticks < interval){
        setDesiredLinearVelocity(_rand * 3.0f);
        setDesiredAngularVelocity(0.0f);
    }
    else if(_ticks >= interval && _ticks < interval*2){
        setDesiredLinearVelocity(-_rand * 3.0f);
        setDesiredAngularVelocity(0.0f);
    }
    else if(_ticks >= interval*2 && _ticks < interval*3){
        setDesiredLinearVelocity(0.0f);
        setDesiredAngularVelocity(_rand * 1.57f);
    }
    else if(_ticks >= interval*3 && _ticks < interval*4){
        setDesiredLinearVelocity(0.0f);
        setDesiredAngularVelocity(-_rand * 1.57f);
    }
    else if(_ticks >= interval*4 && _ticks < interval*5){
        setDesiredLinearVelocity(_rand * 3.0f);
        setDesiredAngularVelocity(_rand * 1.57f);
    }
    else if(_ticks >= interval*5 && _ticks < interval*6){
        setDesiredLinearVelocity(-_rand * 3.0f);
        setDesiredAngularVelocity(-_rand * 1.57f);
    }
    else if(_ticks >= interval*6 && _ticks < interval*7){
        setDesiredLinearVelocity(-_rand * 3.0f);
        setDesiredAngularVelocity(_rand * 1.57f);
    }
    else if(_ticks >= interval*7 && _ticks < interval*8){
        setDesiredLinearVelocity(_rand * 3.0f);
        setDesiredAngularVelocity(-_rand * 1.57f);
    }

    if(_ticks % 250 == 0){
        _rand = distribution(generator);
    }

    _ticks++;

    move(dt);
}
