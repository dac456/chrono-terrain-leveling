#ifndef __PARTICLESYSTEM_HPP
#define __PARTICLESYSTEM_HPP

#include "CtlCommon.hpp"

class ParticleSystem{
private:
    std::vector<ChBodyPtr> _particles;

public:
    ParticleSystem(ChSystem* system, ChVectord dimensions, double particleDensity, double particleSize, bool container = true, bool visContainer = true);
    ~ParticleSystem();
};

#endif
