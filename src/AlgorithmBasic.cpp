#include "AlgorithmBasic.hpp"

AlgorithmBasic::AlgorithmBasic(std::shared_ptr<TrackedVehicle> vehicle, std::shared_ptr<vehicle::m113::M113_SimplePowertrain> powertrain, std::shared_ptr<vehicle::DeformableTerrain> terrain)
    : Platform(vehicle, powertrain, terrain)
    , _turning(false)
    , _waiting(true)
    , _turnDirection(1)
    , _turnTicks(0)
    , _waitTicks(0)
{


}

AlgorithmBasic::~AlgorithmBasic() {

}

void AlgorithmBasic::senseImpl(float dt) {

}

void AlgorithmBasic::actImpl(float dt) {
    if(isInclined() && !_turning){
        setDesiredLinearVelocity(3.0f);
        setDesiredAngularVelocity(0.0f);
    }
    else{
        setDesiredLinearVelocity(2.0f);
        /*setDesiredLinearVelocity(2.0f);

        double r = ChRandom();
        if(r < 0.25) setDesiredAngularVelocity(1.57);
        else if(r >= 0.25 && r < 0.5) setDesiredAngularVelocity(-1.57);
        else if(r > 0.75) setDesiredAngularVelocity(0.0f);*/
        if(!_waiting){
            if(!_turning){
                double r = ChRandom();
                if(r < 0.5) _turnDirection = -1;
                else _turnDirection = 1;

                _turning = true;
            }

            if(_turnTicks < 120){
                setDesiredAngularVelocity(1.57 * _turnDirection);

                _turnTicks++;
            }
            else{
                setDesiredAngularVelocity(0.0);
                _turnTicks = 0;
                _turning = false;
                _waiting = true;
            }
        }
        else{
            if(_waitTicks < 120){
                _waitTicks++;
            }
            else{
                _waitTicks = 0;
                _waiting = false;
            }
        }
    }

    move(dt);
}
