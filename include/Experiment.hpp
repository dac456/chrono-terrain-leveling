#ifndef __EXPERIMENT_HPP
#define __EXPERIMENT_HPP

#include "CtlCommon.hpp"

class Experiment{
private:
    std::string _name;
    po::variables_map _vm;

    PlatformPtr _platform;
    HeightMapPtr _hm;
    RayGridPtr _rayGrid;
    size_t _frameCount;

    double _linearVel;
    double _angularVel;

    bool _enteringX;
    bool _enteringZ;

public:
    Experiment(ChSystem* system, std::string expConfigFile);
    ~Experiment();

    void step(double dt);
    void writeFrame();

    PlatformPtr getPlatform();
    po::variables_map getVariables();

};

#endif
