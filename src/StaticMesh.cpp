#include "StaticMesh.hpp"
#include "AssimpLoader.hpp"

StaticMesh::StaticMesh(ChSystem* system, std::string name, std::string file, ChVector<double> position, ChMaterialPtr material)
    : _system(system),
      _name(name),
      _file(file),
      _position(position),
      _material(material),
      _body(new ChBody(DEFAULT_BODY))

{
    if(_material) _body->SetMaterialSurface(_material);

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
    std::shared_ptr<ChTriangleMeshShape> meshAsset(new ChTriangleMeshShape);
    meshAsset->SetMesh(*colMesh);
    _body->AddAsset(meshAsset);

    //Add body to system
    _system->AddBody(_body);

}

StaticMesh::~StaticMesh(){

}
