#include "StaticMesh.hpp"
#include "AssimpLoader.hpp"

StaticMesh::StaticMesh(std::string name, std::string file, ChVector<double> position, double mass, ChMaterialPtr material)
    : _name(name),
      _file(file),
      _position(position),
      _mass(mass),
      _material(material),
      _body(new ChBody(DEFAULT_BODY))

{
    if(_material) _body->SetMaterialSurface(_material);

    _body->SetMass(_mass);
    _body->SetPos(_position);
    _body->SetCollide(true);
    _body->SetBodyFixed(true);

    //Build collision geometery
    AssimpLoader ai(GetChronoDataFile(_file));
    std::shared_ptr<geometry::ChTriangleMeshConnected> colMesh = ai.toChronoTriMesh();

    for(auto& v : colMesh->m_vertices){
        v += _position;
    }

    _body->GetCollisionModel()->ClearModel();
    //_body->GetCollisionModel()->AddTriangleMesh(*colMesh.get(), true, true);
    _body->GetCollisionModel()->AddBox(20, 0.1, 20, _position);
    _body->GetCollisionModel()->BuildModel();

    //Create Irrlicht asset
    ChSharedPtr<ChTriangleMeshShape> meshAsset(new ChTriangleMeshShape);
    meshAsset->SetMesh(*colMesh);
    _body->AddAsset(meshAsset);

}

StaticMesh::~StaticMesh(){

}
