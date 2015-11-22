#ifndef __STATICMESHFACTORY_H
#define __STATICMESHFACTORY_H

#include "CtlCommon.hpp"

class StaticMeshFactory{
private:
    ChSystem* _system;
    size_t _nextId;

public:
    StaticMeshFactory(ChSystem* system);
    ~StaticMeshFactory();
  
    StaticMeshPtr createStaticMesh(std::string name, std::string file, ChVector<double> position, double mass);
};

#endif