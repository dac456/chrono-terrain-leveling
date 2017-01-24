#ifndef __ALGORITHMRANDOM_HPP
#define __ALGORITHMRANDOM_HPP

#include "Platform.hpp"

class AlgorithmRandom : public Platform {
private:
    size_t _ticks;
    double _rand;

public:
    AlgorithmRandom(std::shared_ptr<TrackedVehicle> vehicle, std::shared_ptr<vehicle::m113::M113_SimplePowertrain> powertrain, std::shared_ptr<vehicle::DeformableTerrain> terrain);
    virtual ~AlgorithmRandom();

    void senseImpl(float dt) final;
    void actImpl(float dt) final;
};

#endif
