#include "StaticMesh.h"

StaticMesh::StaticMesh(std::string name, ChVector<double> position, double mass, ChDEMMaterialPtr material)
    : _name(name),
      _position(position),
      _mass(mass),
      _material(material),
      _body(new ChBody(new collision::ChCollisionModelParallel, ChMaterialSurfaceBase::DEM))
{
    if(_material) _body->SetMaterialSurface(_material);
    
    _body->SetMass(_mass);
    _body->SetPos(_position);
    _body->SetCollide(true);
    _body->SetBodyFixed(true);
}

StaticMesh::~StaticMesh(){
    
}