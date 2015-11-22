#ifndef __STATICMESH_H
#define __STATICMESH_H

#include "CtlCommon.h"

class StaticMesh{
private:
    std::string _name;
    std::string _file;
    
    ChVector<double> _position;
    double _mass;
    
    ChBodyPtr _body;
    ChMaterialPtr _material;
    
public:
    StaticMesh(std::string name, std::string file, ChVector<double> position, double mass, ChMaterialPtr material = ChMaterialPtr(0));
    ~StaticMesh();

    friend class StaticMeshFactory;
};

#endif