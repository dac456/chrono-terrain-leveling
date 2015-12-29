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
    _body->GetMaterialSurface()->SetFriction(1.0);

    //Build collision geometery
    AssimpLoader ai(GetChronoDataFile(_file));
    std::shared_ptr<geometry::ChTriangleMeshConnected> colMesh = ai.toChronoTriMesh();
    ChVectord meshDim = ai.getMeshDimensions();

    for(auto& v : colMesh->m_vertices){
        v += _position;
    }

    _body->GetCollisionModel()->ClearModel();
    //_body->GetCollisionModel()->AddTriangleMesh(*colMesh.get(), true, true);
    _body->GetCollisionModel()->AddBox(meshDim.x/2.0, 0.25, meshDim.z/2.0, _position);
    _body->GetCollisionModel()->BuildModel();

    //Create Irrlicht asset
    ChSharedPtr<ChTriangleMeshShape> meshAsset(new ChTriangleMeshShape);
    meshAsset->SetMesh(*colMesh);
    _body->AddAsset(meshAsset);

}

StaticMesh::~StaticMesh(){

}
