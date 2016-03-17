#ifndef __ALGORITHMBASIC_HPP
#define __ALGORITHMBASIC_HPP

#include "Platform.hpp"

class AlgorithmBasic : public Platform {
private:

public:
    AlgorithmBasic(TrackedVehiclePtr vehicle);
    virtual ~AlgorithmBasic();

    void senseImpl(float dt) override;
    void actImpl(float dt) override;
};

#endif
