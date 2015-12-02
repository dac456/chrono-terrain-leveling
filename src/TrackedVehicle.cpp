#include "TrackedVehicle.hpp"
#include "AssimpLoader.hpp"

TrackedVehicle::TrackedVehicle(std::string name, std::string bodyFile, std::string wheelFile, double mass)
    : _name(name),
      _body(new ChBody(DEFAULT_BODY)),
      _lfWheel(new ChBody(DEFAULT_BODY)),
      _lbWheel(new ChBody(DEFAULT_BODY)),
      _rfWheel(new ChBody(DEFAULT_BODY)),
      _rbWheel(new ChBody(DEFAULT_BODY))
{
    AssimpLoader aiBody(GetChronoDataFile(bodyFile));
    std::shared_ptr<geometry::ChTriangleMeshConnected> bodyMesh = aiBody.toChronoTriMesh();
    ChVectord bodyDim = aiBody.getMeshDimensions();
    ChVectord bodyCentre = aiBody.getMeshCentre();

    AssimpLoader aiWheel(GetChronoDataFile(wheelFile));
    std::shared_ptr<geometry::ChTriangleMeshConnected> wheelMesh = aiWheel.toChronoTriMesh();
    ChVectord wheelDim = aiWheel.getMeshDimensions();
    ChVectord wheelCentre = aiWheel.getMeshCentre();

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



    //Left front wheel
    _lfWheel->SetMass(10.0);
    _lfWheel->SetCollide(true);
    _lfWheel->SetBodyFixed(false);

    _lfWheel->GetCollisionModel()->ClearModel();
    _lfWheel->GetCollisionModel()->AddCylinder(wheelDim.x/2.0, wheelDim.z/2.0, wheelDim.y, wheelCentre);
    _lfWheel->GetCollisionModel()->BuildModel();

    //Create link
    _lfWheelLink = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);
    _lfWheelLink->Initialize(_lfWheel, _body, ChCoordsys<>(ChVector<>(0.8, 0, 0.56), QUNIT));

    //Create Irrlicht asset for lfWheel
    ChSharedPtr<ChTriangleMeshShape> lfWheelMeshAsset(new ChTriangleMeshShape);
    lfWheelMeshAsset->SetMesh(*wheelMesh);

    ChSharedPtr<ChAssetLevel> lfWheelAsset(new ChAssetLevel);
    lfWheelAsset->GetFrame().SetRot(chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_X));
    lfWheelAsset->GetFrame().SetPos(ChVector<>(0.8, 0, 0.56));
    lfWheelAsset->AddAsset(lfWheelMeshAsset);
    _lfWheel->AddAsset(lfWheelAsset);


    //Right front wheel
    _rfWheel->SetMass(10.0);
    _rfWheel->SetCollide(true);
    _rfWheel->SetBodyFixed(false);

    _rfWheel->GetCollisionModel()->ClearModel();
    _rfWheel->GetCollisionModel()->AddCylinder(wheelDim.x/2.0, wheelDim.z/2.0, wheelDim.y);
    _rfWheel->GetCollisionModel()->BuildModel();

    //Create link
    _rfWheelLink = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);
    _rfWheelLink->Initialize(_rfWheel, _body, ChCoordsys<>(ChVector<>(-0.8, 0, 0.56), QUNIT));

    //Create Irrlicht asset for rfWheel
    ChSharedPtr<ChTriangleMeshShape> rfWheelMeshAsset(new ChTriangleMeshShape);
    rfWheelMeshAsset->SetMesh(*wheelMesh);

    ChSharedPtr<ChAssetLevel> rfWheelAsset(new ChAssetLevel);
    rfWheelAsset->GetFrame().SetRot(chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_X));
    rfWheelAsset->GetFrame().SetPos(ChVector<>(-0.8, 0, 0.56));
    rfWheelAsset->AddAsset(rfWheelMeshAsset);
    _rfWheel->AddAsset(rfWheelAsset);
}
