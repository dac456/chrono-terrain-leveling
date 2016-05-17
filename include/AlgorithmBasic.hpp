#ifndef __ALGORITHMBASIC_HPP
#define __ALGORITHMBASIC_HPP

#include "Platform.hpp"

class AlgorithmBasic : public Platform {
private:

public:
    AlgorithmBasic(TrackedVehiclePtr vehicle);
    virtual ~AlgorithmBasic();

    void senseImpl(float dt) final;
    void actImpl(float dt) final;
};

#endif
