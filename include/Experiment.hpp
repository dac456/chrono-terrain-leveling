#ifndef __EXPERIMENT_HPP
#define __EXPERIMENT_HPP

#include "CtlCommon.hpp"

class Experiment{
private:
    std::string _name;

    PlatformPtr _platform;
    RayGridPtr _rayGrid;
    size_t _frameCount;

    double _linearVel;
    double _angularVel;

public:
    Experiment(ChSystem* system, std::string expConfigFile);
    ~Experiment();

    void step(double dt);
    void writeFrame();

};

#endif
