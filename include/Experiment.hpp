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

    double _lastX;
    double _lastY;
    double _lastTheta;
    double _lastLeftSpeed;
    double _lastRightSpeed;

    bool _warped;
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
