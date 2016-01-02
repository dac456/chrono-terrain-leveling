#ifndef __PARTICLESYSTEM_HPP
#define __PARTICLESYSTEM_HPP

#include "CtlCommon.hpp"

class ParticleSystem{
private:
    std::vector<ChBodyPtr> _particles;

public:
    ParticleSystem(ChSystem* system, ChVectord dimensions, double density, double particleSize);
    ~ParticleSystem();
};

#endif
