#ifndef __STATICMESH_H
#define __STATICMESH_H

#include "CtlCommon.hpp"

class StaticMesh{
private:
    ChSystem* _system;

    std::string _name;
    std::string _file;

    ChVector<double> _position;

    ChBodyPtr _body;
    ChMaterialPtr _material;

public:
    StaticMesh(ChSystem* system, std::string name, std::string file, ChVector<double> position, ChMaterialPtr material = ChMaterialPtr(0));
    ~StaticMesh();

    friend class StaticMeshFactory;
};

#endif
