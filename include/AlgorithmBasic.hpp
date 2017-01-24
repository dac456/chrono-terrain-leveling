#ifndef __ALGORITHMBASIC_HPP
#define __ALGORITHMBASIC_HPP

#include "Platform.hpp"

class AlgorithmBasic : public Platform {
private:
    bool _turning;
    bool _waiting;
    int _turnDirection;
    size_t _turnTicks;
    size_t _waitTicks;

public:
    AlgorithmBasic(std::shared_ptr<TrackedVehicle> vehicle, std::shared_ptr<vehicle::m113::M113_SimplePowertrain> powertrain, std::shared_ptr<vehicle::DeformableTerrain> terrain);
    virtual ~AlgorithmBasic();

    void senseImpl(float dt) final;
    void actImpl(float dt) final;
};

#endif
