#include "TrackedVehicle.h"
#include "AssimpLoader.h"

TrackedVehicle::TrackedVehicle(std::string name, std::string bodyFile, std::string wheelFile, double mass)
    : _name(name),
      _body(new ChBody(new collision::ChCollisionModelParallel, ChMaterialSurfaceBase::DVI)),
      _lfWheel(new ChBody(new collision::ChCollisionModelParallel, ChMaterialSurfaceBase::DVI)),
      _lbWheel(new ChBody(new collision::ChCollisionModelParallel, ChMaterialSurfaceBase::DVI)),
      _rfWheel(new ChBody(new collision::ChCollisionModelParallel, ChMaterialSurfaceBase::DVI)),
      _rbWheel(new ChBody(new collision::ChCollisionModelParallel, ChMaterialSurfaceBase::DVI))
{
    //Vehicle Body
    _body->SetMass(mass);
    _body->SetCollide(true);
    _body->SetBodyFixed(false);
    
    AssimpLoader aiBody(GetChronoDataFile(bodyFile));
    std::shared_ptr<geometry::ChTriangleMeshConnected> colMesh = aiBody.toChronoTriMesh();
    
    _body->GetCollisionModel()->ClearModel();
    _body->GetCollisionModel()->AddBox(1,1,1);
    _body->GetCollisionModel()->BuildModel();
    
    //Create Irrlicht asset for body
    ChSharedPtr<ChTriangleMeshShape> bodyMeshAsset(new ChTriangleMeshShape);
    bodyMeshAsset->SetMesh(*colMesh);
    _body->AddAsset(bodyMeshAsset);    
}