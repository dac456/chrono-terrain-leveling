#ifndef __EXPERIMENT_HPP
#define __EXPERIMENT_HPP

#include "CtlCommon.hpp"
#include "chrono_vehicle/terrain/DeformableTerrain.h"

class Experiment{
private:
    std::string _name;
    po::variables_map _vm;
    ChSystem* _system;

    PlatformPtr _platform;
    HeightMapPtr _hm;
    RayGridPtr _rayGrid;
    std::shared_ptr<vehicle::DeformableTerrain> _terrain;
    size_t _frameCount;

    ChVectord _terrainMin;
    ChVectord _terrainMax;

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

    bool _useParticles;
    bool _useTracks;

public:
    Experiment(ChSystem* system, std::string expConfigFile);
    ~Experiment();

    void step(double dt, bool createPlatform = false);
    void writeFrame();

    PlatformPtr getPlatform();
    po::variables_map getVariables();

};

#endif
