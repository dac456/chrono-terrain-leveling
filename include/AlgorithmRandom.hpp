#ifndef __ALGORITHMRANDOM_HPP
#define __ALGORITHMRANDOM_HPP

#include "Platform.hpp"

class AlgorithmRandom : public Platform {
private:
    size_t _ticks;
    double _rand;

public:
    AlgorithmRandom(TrackedVehiclePtr vehicle);
    virtual ~AlgorithmRandom();

    void senseImpl(float dt) final;
    void actImpl(float dt) final;
};

#endif
