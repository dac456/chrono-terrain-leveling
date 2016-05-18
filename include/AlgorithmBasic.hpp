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
    AlgorithmBasic(TrackedVehiclePtr vehicle);
    virtual ~AlgorithmBasic();

    void senseImpl(float dt) final;
    void actImpl(float dt) final;
};

#endif
