#include "TrackedVehicle.hpp"
#include "AssimpLoader.hpp"
#include "Assembly.hpp"

TrackedVehicle::TrackedVehicle(std::string name, std::string shoeVisFile, std::string shoeColFile, AssemblyPtr assembly)
    : _name(name),
      _assembly(assembly)
{
    ChBodyPtr blWheel, brWheel, flWheel, frWheel;
    //blWheel = brWheel = flWheel = frWheel = nullptr;

    std::vector<ChBodyPtr> bodies = _assembly->getBodies();
    for(auto body : bodies){
        if(std::string(body->GetName()).find("bl_Wheel") != std::string::npos){
            blWheel = body;
            blWheel->GetMaterialSurface()->SetFriction(1.0);
        }
        if(std::string(body->GetName()).find("br_Wheel") != std::string::npos){
            brWheel = body;
            brWheel->GetMaterialSurface()->SetFriction(1.0);
        }
        if(std::string(body->GetName()).find("fl_Wheel") != std::string::npos){
            flWheel = body;
            flWheel->GetMaterialSurface()->SetFriction(1.0);
        }
        if(std::string(body->GetName()).find("fr_Wheel") != std::string::npos){
            frWheel = body;
            frWheel->GetMaterialSurface()->SetFriction(1.0);
        }
    }

    if(blWheel && brWheel && flWheel && frWheel){
        std::cout << "Assembly has all wheels" << std::endl;
        ChVectord blPos = blWheel->GetCoord().pos;
        ChVectord brPos = brWheel->GetCoord().pos;
        ChVectord flPos = flWheel->GetCoord().pos;
        ChVectord frPos = frWheel->GetCoord().pos;

        AssimpLoader aiShoe(GetChronoDataFile(shoeVisFile), ChVectord(1.8,1.8,1.6));
        _shoeMesh = aiShoe.toChronoTriMesh();
        ChVectord shoeDim = aiShoe.getMeshDimensions();

        AssimpLoader aiCollision(GetChronoDataFile(shoeColFile), ChVectord(1.8,1.8,1.6));
        _collisionMesh = aiCollision.toChronoTriMesh();

        std::vector<ChVectord> trackOrigin;
        trackOrigin.push_back(brPos);
        trackOrigin.push_back(blPos);

        for(ChVectord brPos : trackOrigin){
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
            firstShoeBody->GetCollisionModel()->AddTriangleMesh(*(_collisionMesh.get()), false, false);
            //firstShoeBody->GetCollisionModel()->AddBox(shoeDim.x/2.0, shoeDim.y/2.0, shoeDim.z/2.0, ChVectord(0,0,0));

            firstShoeBody->GetCollisionModel()->SetFamily(4);
            firstShoeBody->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(4);
            firstShoeBody->GetCollisionModel()->BuildModel();

            _assembly->getSystem()->AddBody(firstShoeBody);

            double shoeDist = fabs(frPos.x - brPos.x);
            size_t numShoes = ceil(shoeDist / (shoeDim.x-0.1));

            double arcLength = CH_C_PI * 0.58;
            size_t numWrap = floor(arcLength / (shoeDim.x-0.1));

            ChBodyPtr previousShoeBody = firstShoeBody;
            for(size_t i=1; i<numShoes; i++){
                ChVectord position = ChVectord(brPos.x + ((shoeDim.x-0.1)*i), brPos.y + 0.58, brPos.z+0.03);
                previousShoeBody = _createShoe(previousShoeBody, shoeDim, position);
            }
            double lx, ly;
            lx = ly = 0.0;
            for(size_t i=0; i<numWrap; i++){
                double alpha = (CH_C_PI / ((double)(numWrap-1))) * ((double)i);

                lx = brPos.x + ((shoeDim.x-0.1)*(numShoes)) + 0.58 * sin(alpha);
                ly = brPos.y + 0.58 * cos(alpha);
                //position.Set(lx, ly, brPos.z);
                ChVectord position = ChVectord(lx, ly, brPos.z+0.03);
                ChQuatd rotation = chrono::Q_from_AngAxis(alpha+0.3, ChVector<>(0, 0, 1));
                previousShoeBody = _createShoe(previousShoeBody, shoeDim, position, rotation);
            }
            for(size_t i=1; i<numShoes; i++){
                ChVectord position = ChVectord(lx - (shoeDim.x-0.1) - ((shoeDim.x-0.1)*i), ly, brPos.z+0.03);
                previousShoeBody = _createShoe(previousShoeBody, shoeDim, position, QUNIT, false);
            }
            for(size_t i=0; i<numWrap-1; i++){
                double alpha = (CH_C_PI / ((double)(numWrap - 2))) * ((double)i);

                lx = brPos.x - 0.58 * sin(alpha);
                ly = brPos.y - 0.58 * cos(alpha);
                //position.Set(lx, ly, brPos.z);
                ChVectord position = ChVectord(lx, ly, brPos.z+0.03);
                ChQuatd rotation = chrono::Q_from_AngAxis(alpha + CH_C_PI, ChVector<>(0, 0, 1));
                previousShoeBody = _createShoe(previousShoeBody, shoeDim, position, rotation, true);
            }

            ChSharedPtr<ChLinkLockRevolute> finalJoint = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);
            finalJoint->Initialize(firstShoeBody, previousShoeBody, ChCoordsys<>(ChVectord(brPos.x, brPos.y + 0.58, brPos.z+0.03), QUNIT));

            _assembly->getSystem()->AddLink(finalJoint);
        }
    }
}

ChBodyPtr TrackedVehicle::_createShoe(ChBodyPtr previousShoeBody, ChVectord shoeDim, ChVectord shoePosition, ChQuatd shoeRotation, bool top){
    ChBodyPtr nextShoeBody(new ChBody(DEFAULT_BODY));
    nextShoeBody->SetMass(5.0);
    nextShoeBody->SetCollide(true);
    nextShoeBody->SetBodyFixed(false);

    //Create Irrlicht asset for shoe
    ChSharedPtr<ChTriangleMeshShape> shoeAsset(new ChTriangleMeshShape);
    shoeAsset->SetMesh(*_shoeMesh);
    nextShoeBody->AddAsset(shoeAsset);

    ChFrameMoving<> frame;
    if(top){
        ChFrameMoving<> baseFrame(shoePosition, Q_from_AngAxis(CH_C_PI, ChVectord(1,0,0)));
        ChFrameMoving<> rotFrame(ChVectord(0,0,0), shoeRotation);
        frame = rotFrame >> baseFrame;
        nextShoeBody->ConcatenatePreTransformation(frame);
    }
    else{
        frame = ChFrameMoving<>(shoePosition, shoeRotation);
        nextShoeBody->ConcatenatePreTransformation(frame);
    }

    nextShoeBody->GetCollisionModel()->ClearModel();
    nextShoeBody->GetCollisionModel()->AddTriangleMesh(*(_collisionMesh.get()), false, false);
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
