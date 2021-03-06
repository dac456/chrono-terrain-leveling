#include "TrackedVehicle.hpp"
#include "AssimpLoader.hpp"
#include "Assembly.hpp"

TrackedVehicle::TrackedVehicle(std::string name, std::string shoeVisFile, std::string shoeColFile, AssemblyPtr assembly, double wheelRadius)
    : _name(name),
      _assembly(assembly),
      _wheelRadius(wheelRadius)
{
    ChBodyPtr blWheel, brWheel, flWheel, frWheel;

    //Check all bodies in assembly and extract the wheels
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

    //If all wheels are present, generate tracks
    if(blWheel && brWheel && flWheel && frWheel){
        std::cout << "Assembly has all wheels" << std::endl;
        ChVectord blPos = blWheel->GetCoord().pos;
        ChVectord brPos = brWheel->GetCoord().pos;
        ChVectord flPos = flWheel->GetCoord().pos;
        ChVectord frPos = frWheel->GetCoord().pos;

        AssimpLoader aiShoe(GetChronoDataFile(shoeVisFile), ChVectord(1.8,1.4,1.4));
        _shoeMesh = aiShoe.toChronoTriMesh();
        ChVectord shoeDim = aiShoe.getMeshDimensions();

        AssimpLoader aiCollision(GetChronoDataFile(shoeColFile), ChVectord(1.8,1.4,1.4));
        _collisionMesh = aiCollision.toChronoTriMesh();

        //Back wheels are origin of track on each side
        std::vector<ChVectord> trackOrigin;
        trackOrigin.push_back(brPos);
        trackOrigin.push_back(blPos);

        //Generate track loop
        for(ChVectord brPos : trackOrigin){
            ChBodyPtr firstShoeBody(new ChBody(DEFAULT_BODY));
            //firstShoeBody->SetMass(0.5);
            firstShoeBody->SetCollide(true);
            firstShoeBody->SetBodyFixed(false);

            //Create Irrlicht asset for shoe
            ChSharedPtr<ChTriangleMeshShape> shoeMeshAsset(new ChTriangleMeshShape);
            shoeMeshAsset->SetMesh(*_shoeMesh);
            firstShoeBody->AddAsset(shoeMeshAsset);

            double pz = brPos.z;

            ChFrameMoving<> shoeFrame(ChVectord(brPos.x, brPos.y + _wheelRadius, pz), Q_from_AngAxis(CH_C_PI, ChVectord(1,0,0)));
            firstShoeBody->ConcatenatePreTransformation(shoeFrame);

            //firstShoeBody->GetCollisionModel()->SetSafeMargin(0.004);
            //firstShoeBody->GetCollisionModel()->SetEnvelope(0.010);

            firstShoeBody->GetCollisionModel()->ClearModel();
            firstShoeBody->GetCollisionModel()->AddTriangleMesh(*(_collisionMesh.get()), false, false);
            //firstShoeBody->GetCollisionModel()->AddBox(shoeDim.x/2.0, shoeDim.y/2.0, shoeDim.z/2.0, ChVectord(0,0,0));

            firstShoeBody->GetCollisionModel()->SetFamily(4);
            firstShoeBody->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(4);
            firstShoeBody->GetCollisionModel()->BuildModel();

            _assembly->getSystem()->AddBody(firstShoeBody);

            double shoeDist = fabs(frPos.x - brPos.x);
            size_t numShoes = ceil(shoeDist / (shoeDim.x-0.1));

            double arcLength = CH_C_PI * _wheelRadius;
            size_t numWrap = floor(arcLength / (shoeDim.x-0.1));

            ChBodyPtr previousShoeBody = firstShoeBody;
            double px = 0.0;

            for(size_t i=0; i<=numShoes; i++){
                px = brPos.x + ((shoeDim.x-0.1)*i);
                ChVectord position = ChVectord(px, brPos.y + _wheelRadius, pz);
                previousShoeBody = _createShoe(previousShoeBody, shoeDim, position);
            }
            double lx, ly;
            double alpha = 0.0;
            lx = ly = 0.0;
            ChQuatd rotation;
            for(size_t i=0; i<numWrap; i++){
                //double alpha = (CH_C_PI / ((double)(numWrap))) * ((double)i);
                double da = CH_C_PI / double(numWrap);
                alpha += da;
                //double alpha = (CH_C_PI_2 * i) / (double)(numWrap);

                lx = px  + (_wheelRadius * sin(alpha));
                ly = brPos.y + (_wheelRadius * cos(alpha));
                //position.Set(lx, ly, brPos.z);
                ChVectord position = ChVectord(lx, ly, pz);
                rotation = chrono::Q_from_AngAxis(alpha, ChVector<>(0, 0, 1));
                previousShoeBody = _createShoe(previousShoeBody, shoeDim, position, rotation);
            }
            for(size_t i=1; i<=numShoes; i++){
                px = lx - ((shoeDim.x-0.1)*i);
                ChVectord position = ChVectord(px, ly, pz);
                previousShoeBody = _createShoe(previousShoeBody, shoeDim, position, rotation);
            }
            for(size_t i=0; i<numWrap-1; i++){
                //double alpha = (CH_C_PI / ((double)(numWrap))) * ((double)i);
                double da = CH_C_PI / double(numWrap);
                alpha += da;

                lx = px + (_wheelRadius * sin(alpha));
                ly = brPos.y + (_wheelRadius * cos(alpha));
                //position.Set(lx, ly, brPos.z);
                ChVectord position = ChVectord(lx, ly, pz);
                rotation = chrono::Q_from_AngAxis(alpha, ChVector<>(0, 0, 1));
                previousShoeBody = _createShoe(previousShoeBody, shoeDim, position, rotation);
            }

            ChSharedPtr<ChLinkLockRevolute> finalJoint = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);
            finalJoint->Initialize(previousShoeBody, firstShoeBody, ChCoordsys<>(ChVectord(brPos.x, brPos.y+_wheelRadius, pz), QUNIT));

            _assembly->getSystem()->AddLink(finalJoint);
        }
    }
}

void TrackedVehicle::setSpeeds(double left, double right){
    std::vector<ChLinkPtr> links = _assembly->getLinks();
    for(auto link : links){
        if(std::string(link->GetName()).find("fl_Wheel") != std::string::npos){
            if(ChSharedPtr<ChLinkEngine> chJoint = link.DynamicCastTo<ChLinkEngine>()){
                if(ChSharedPtr<ChFunction_Const> fn = chJoint->Get_spe_funct().DynamicCastTo<ChFunction_Const>()){
                    fn->Set_yconst(left);
                }
            }
        }
        if(std::string(link->GetName()).find("fr_Wheel") != std::string::npos){
            if(ChSharedPtr<ChLinkEngine> chJoint = link.DynamicCastTo<ChLinkEngine>()){
                if(ChSharedPtr<ChFunction_Const> fn = chJoint->Get_spe_funct().DynamicCastTo<ChFunction_Const>()){
                    fn->Set_yconst(right);
                }
            }
        }
    }
}

ChBodyPtr TrackedVehicle::_createShoe(ChBodyPtr previousShoeBody, ChVectord shoeDim, ChVectord shoePosition, ChQuatd shoeRotation){
    ChBodyPtr nextShoeBody(new ChBody(DEFAULT_BODY));
    //nextShoeBody->SetMass(0.5);
    nextShoeBody->SetCollide(true);
    nextShoeBody->SetBodyFixed(false);

    //Create Irrlicht asset for shoe
    ChSharedPtr<ChTriangleMeshShape> shoeAsset(new ChTriangleMeshShape);
    shoeAsset->SetMesh(*_shoeMesh);
    nextShoeBody->AddAsset(shoeAsset);

    ChFrameMoving<> baseFrame(shoePosition, Q_from_AngAxis(CH_C_PI, ChVectord(1,0,0)));
    ChFrameMoving<> rotFrame(ChVectord(0,0,0), shoeRotation);
    ChFrameMoving<> frame = rotFrame >> baseFrame;
    nextShoeBody->ConcatenatePreTransformation(frame);

    //nextShoeBody->GetCollisionModel()->SetSafeMargin(0.004);
    //nextShoeBody->GetCollisionModel()->SetEnvelope(0.010);

    nextShoeBody->GetCollisionModel()->ClearModel();
    nextShoeBody->GetCollisionModel()->AddTriangleMesh(*(_collisionMesh.get()), false, false);
    //nextShoeBody->GetCollisionModel()->AddBox(shoeDim.x/2.0, shoeDim.y/2.0, shoeDim.z/2.0, ChVectord(0,0,0));

    nextShoeBody->GetCollisionModel()->SetFamily(4);
    nextShoeBody->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(4);
    nextShoeBody->GetCollisionModel()->BuildModel();

    _assembly->getSystem()->AddBody(nextShoeBody);

    ChSharedPtr<ChLinkLockRevolute> shoeJoint = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);
    shoeJoint->Initialize(nextShoeBody, previousShoeBody, ChCoordsys<>(shoePosition, QUNIT));

    _assembly->getSystem()->AddLink(shoeJoint);

    return nextShoeBody;
}
