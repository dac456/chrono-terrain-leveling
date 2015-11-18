#include "StaticMesh.h"

StaticMesh::StaticMesh(std::string name, std::string file, ChVector<double> position, double mass, ChDEMMaterialPtr material)
    : _name(name),
      _file(file),
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
    
    //Build collision geometery
    //geometry::ChTriangleMeshConnected colMesh;
    //colMesh.LoadWavefrontMesh(GetChronoDataFile(_file), true, true);
    
    /*for(auto& v : colMesh.m_vertices){
        v += _position;
    }*/
    
    _body->GetCollisionModel()->ClearModel();
    //_body->GetCollisionModel()->AddTriangleMesh(colMesh, true, false, _position);
    _body->GetCollisionModel()->AddSphere(2.0, _position);
    _body->GetCollisionModel()->BuildModel();
    
    //Create Irrlicht asset
    //ChSharedPtr<ChTriangleMeshShape> meshAsset(new ChTriangleMeshShape);
    //meshAsset->SetMesh(colMesh);
    ChSharedPtr<ChObjShapeFile> meshAsset(new ChObjShapeFile);
    meshAsset->SetFilename(GetChronoDataFile(_file));
    _body->AddAsset(meshAsset);
}

StaticMesh::~StaticMesh(){
    
}