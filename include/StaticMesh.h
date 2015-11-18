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
    ChDEMMaterialPtr _material;
    
public:
    StaticMesh(std::string name, std::string file, ChVector<double> position, double mass, ChDEMMaterialPtr material = ChDEMMaterialPtr(0));
    ~StaticMesh();

    friend class StaticMeshFactory;
};

#endif