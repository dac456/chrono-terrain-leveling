#include "TrackedVehicle.hpp"
#include "AssimpLoader.hpp"
#include "Assembly.hpp"

TrackedVehicle::TrackedVehicle(std::string name, std::string shoeFile, AssemblyPtr assembly)
    : _name(name),
      _assembly(assembly)
{
    ChBodyPtr blWheel, brWheel, flWheel, frWheel;
    //blWheel = brWheel = flWheel = frWheel = nullptr;

    std::vector<ChBodyPtr> bodies = _assembly->getBodies();
    for(auto body : bodies){
        if(std::string(body->GetName()).find("bl_Wheel") != std::string::npos){
            blWheel = body;
        }
        if(std::string(body->GetName()).find("br_Wheel") != std::string::npos){
            brWheel = body;
        }
        if(std::string(body->GetName()).find("fl_Wheel") != std::string::npos){
            flWheel = body;
        }
        if(std::string(body->GetName()).find("fr_Wheel") != std::string::npos){
            frWheel = body;
        }
    }

    if(blWheel && brWheel && flWheel && frWheel){
        std::cout << "Assembly has all wheels" << std::endl;
        ChVectord blPos = blWheel->GetCoord().pos;
        ChVectord brPos = brWheel->GetCoord().pos;
        ChVectord flPos = flWheel->GetCoord().pos;
        ChVectord frPos = frWheel->GetCoord().pos;

        AssimpLoader aiShoe(GetChronoDataFile(shoeFile), ChVectord(1.8,1.8,1.6));
        _shoeMesh = aiShoe.toChronoTriMesh();
        ChVectord shoeDim = aiShoe.getMeshDimensions();

        ChBodyPtr firstShoeBody(new ChBody(DEFAULT_BODY));
        firstShoeBody->SetMass(5.0);
        firstShoeBody->SetCollide(true);
        firstShoeBody->SetBodyFixed(false);

        //Create Irrlicht asset for shoe
        ChSharedPtr<ChTriangleMeshShape> shoeMeshAsset(new ChTriangleMeshShape);
        shoeMeshAsset->SetMesh(*_shoeMesh);
        firstShoeBody->AddAsset(shoeMeshAsset);

        ChFrameMoving<> shoeFrame(ChVectord(brPos.x, brPos.y + 0.58, brPos.z+0.03), Q_from_AngAxis(CH_C_PI, ChVectord(1,0,0)));
        firstShoeBody->ConcatenatePreTransformation(shoeFrame);

        firstShoeBody->GetCollisionModel()->ClearModel();
        firstShoeBody->GetCollisionModel()->AddTriangleMesh(*(_shoeMesh.get()), false, false);
        //firstShoeBody->GetCollisionModel()->AddBox(shoeDim.x/2.0, shoeDim.y/2.0, shoeDim.z/2.0, ChVectord(0,0,0));

        firstShoeBody->GetCollisionModel()->SetFamily(4);
        firstShoeBody->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(4);
        firstShoeBody->GetCollisionModel()->BuildModel();

        _assembly->getSystem()->AddBody(firstShoeBody);

        double shoeDist = fabs(frPos.x - brPos.x);
        size_t numShoes = ceil(shoeDist / (shoeDim.x-0.1));

        ChBodyPtr previousShoeBody = firstShoeBody;
        for(size_t i=1; i<numShoes; i++){
            previousShoeBody = _createShoe(i, previousShoeBody, shoeDim, brPos);
        }
        for(size_t i=numShoes; i<numShoes+5; i++){
            double alpha = (CH_C_PI / ((double)(5 - 1.0))) * ((double)i);

            double lx = brPos.x + (shoeDim.x-0.1) + 0.58 * sin(alpha);
            double ly = brPos.y + 0.58 - 0.58 * cos(alpha);
            //position.Set(lx, ly, brPos.z);
            ChQuatd rotation = chrono::Q_from_AngAxis(alpha, ChVector<>(0, 0, 1));
            previousShoeBody = _createShoe(i, previousShoeBody, shoeDim, brPos, rotation);
        }
    }
}

ChBodyPtr TrackedVehicle::_createShoe(size_t idx, ChBodyPtr previousShoeBody, ChVectord shoeDim, ChVectord backWheelPos, ChQuatd shoeRotation){
    ChBodyPtr nextShoeBody(new ChBody(DEFAULT_BODY));
    nextShoeBody->SetMass(5.0);
    nextShoeBody->SetCollide(true);
    nextShoeBody->SetBodyFixed(false);

    //Create Irrlicht asset for shoe
    ChSharedPtr<ChTriangleMeshShape> shoeAsset(new ChTriangleMeshShape);
    shoeAsset->SetMesh(*_shoeMesh);
    nextShoeBody->AddAsset(shoeAsset);

    ChFrameMoving<> baseFrame(ChVectord(backWheelPos.x + ((shoeDim.x-0.1)*idx), backWheelPos.y + 0.58, backWheelPos.z+0.03), Q_from_AngAxis(CH_C_PI, ChVectord(1,0,0)));
    ChFrameMoving<> rotFrame(ChVectord(0,0,0), shoeRotation);
    ChFrameMoving<> frame = rotFrame >> baseFrame;
    nextShoeBody->ConcatenatePreTransformation(frame);

    nextShoeBody->GetCollisionModel()->ClearModel();
    nextShoeBody->GetCollisionModel()->AddTriangleMesh(*(_shoeMesh.get()), false, false);
    //nextShoeBody->GetCollisionModel()->AddBox(shoeDim.x/2.0, shoeDim.y/2.0, shoeDim.z/2.0, ChVectord(0,0,0));

    nextShoeBody->GetCollisionModel()->SetFamily(4);
    nextShoeBody->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(4);
    nextShoeBody->GetCollisionModel()->BuildModel();

    _assembly->getSystem()->AddBody(nextShoeBody);

    ChSharedPtr<ChLinkLockRevolute> shoeJoint = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);
    shoeJoint->Initialize(previousShoeBody, nextShoeBody, frame.GetCoord());

    _assembly->getSystem()->AddLink(shoeJoint);

    return nextShoeBody;
}
