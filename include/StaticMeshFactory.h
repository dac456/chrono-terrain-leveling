#ifndef __STATICMESHFACTORY_H
#define __STATICMESHFACTORY_H

#include "CtlCommon.h"

class StaticMeshFactory{
private:
    ChSystem* _system;

public:
    StaticMeshFactory(ChSystem* system);
    ~StaticMeshFactory();
  
    StaticMeshPtr createStaticMesh(std::string name, ChVector<double> position, double mass);
};

#endif