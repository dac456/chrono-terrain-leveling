#include "TrackedVehicle.hpp"
#include "AssimpLoader.hpp"

TrackedVehicle::TrackedVehicle(std::string name, std::string bodyFile, std::string wheelFile, double mass)
    : _name(name),
      _body(new ChBody(new collision::ChCollisionModelParallel, ChMaterialSurfaceBase::DVI)),
      _lfWheel(new ChBody(new collision::ChCollisionModelParallel, ChMaterialSurfaceBase::DVI)),
      _lbWheel(new ChBody(new collision::ChCollisionModelParallel, ChMaterialSurfaceBase::DVI)),
      _rfWheel(new ChBody(new collision::ChCollisionModelParallel, ChMaterialSurfaceBase::DVI)),
      _rbWheel(new ChBody(new collision::ChCollisionModelParallel, ChMaterialSurfaceBase::DVI))
{
    AssimpLoader aiBody(GetChronoDataFile(bodyFile));
    std::shared_ptr<geometry::ChTriangleMeshConnected> bodyMesh = aiBody.toChronoTriMesh();
    ChVectord bodyDim = aiBody.getMeshDimensions();
    ChVectord bodyCentre = aiBody.getMeshCentre();
    
    std::cout << bodyDim.x <<  " " << bodyDim.y << " " << bodyDim.z << std::endl;
    
    AssimpLoader aiWheel(GetChronoDataFile(wheelFile));
    std::shared_ptr<geometry::ChTriangleMeshConnected> wheelMesh = aiWheel.toChronoTriMesh();
    
    //Vehicle Body
    _body->SetMass(mass);
    _body->SetCollide(true);
    _body->SetBodyFixed(false);
    
    _body->GetCollisionModel()->ClearModel();
    _body->GetCollisionModel()->AddBox(bodyDim.x/2.0, bodyDim.y/2.0, bodyDim.z/2.0, bodyCentre);
    _body->GetCollisionModel()->BuildModel();  
    
    //Create Irrlicht asset for body
    ChSharedPtr<ChTriangleMeshShape> bodyMeshAsset(new ChTriangleMeshShape);
    bodyMeshAsset->SetMesh(*bodyMesh);
    _body->AddAsset(bodyMeshAsset);    
}