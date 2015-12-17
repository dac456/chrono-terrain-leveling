#include "Assembly.hpp"
#include "UrdfLoader.hpp"
#include "AssimpLoader.hpp"

Assembly::Assembly(ChSystem* system)
    : _system(system)
{

}

Assembly::Assembly(UrdfLoader urdfLoader, ChVectord position, ChSystem* system)
    : _system(system)
{
    ChFrameMoving<> initFrame(position, QUNIT);

    for(auto link : urdfLoader.getLinks()){
        ChBodyPtr body = ChBodyPtr(new ChBody(DEFAULT_BODY));
        body->SetName(link->name.c_str());
        body->SetBodyFixed(false);
        body->SetCollide(true);
        body->GetCollisionModel()->ClearModel();

        for(auto visual : link->visuals){
            if(visual->geometry->type != ""){
                UrdfGeometryPtr geom = visual->geometry;
                if(geom->type == "box"){
                    UrdfBoxPtr boxGeom = std::static_pointer_cast<UrdfBox>(geom);

                    ChSharedPtr<ChBoxShape> box(new ChBoxShape);
                    box->GetBoxGeometry().Pos = visual->origin.first;
                    box->GetBoxGeometry().Rot = _toChronoOrientation(visual->origin.second);
                    box->GetBoxGeometry().Size = ChVector<>(boxGeom->dim.x/2.0, boxGeom->dim.z/2.0, boxGeom->dim.y/2.0);
                    body->AddAsset(box);
                }
                else if(geom->type == "cylinder"){
                    UrdfCylinderPtr cylGeom = std::static_pointer_cast<UrdfCylinder>(geom);

                    ChSharedPtr<ChCylinderShape> cyl(new ChCylinderShape);
                    cyl->GetCylinderGeometry().p1 = ChVectord(0, cylGeom->length/2.0, 0);
                    cyl->GetCylinderGeometry().p2 = ChVectord(0, -cylGeom->length/2.0, 0);
                    cyl->GetCylinderGeometry().rad = cylGeom->radius;
                    body->AddAsset(cyl);
                }
                else if(geom->type == "mesh"){
                    UrdfMeshPtr meshGeom = std::static_pointer_cast<UrdfMesh>(geom);

                    AssimpLoader ai(meshGeom->file, ChVectord(meshGeom->scale.x, meshGeom->scale.z, meshGeom->scale.y));
                    std::shared_ptr<geometry::ChTriangleMeshConnected> chMesh = ai.toChronoTriMesh();

                    ChSharedPtr<ChTriangleMeshShape> mesh(new ChTriangleMeshShape);
                    mesh->SetMesh(*chMesh);
                    body->AddAsset(mesh);
                }
                else{
                    std::cout << "Assembly: unknown geometry type (visual)" << std::endl;
                }
            }
        }

        for(auto collision : link->collisions){
            if(collision->geometry->type != ""){
                UrdfGeometryPtr geom = collision->geometry;
                if(geom->type == "box"){
                    UrdfBoxPtr boxGeom = std::static_pointer_cast<UrdfBox>(geom);

                    //body->GetCollisionModel()->ClearModel();
                    body->GetCollisionModel()->AddBox(boxGeom->dim.x/2.0, boxGeom->dim.z/2.0, boxGeom->dim.y/2.0, collision->origin.first);
                    //body->GetCollisionModel()->BuildModel();
                }
                else if(geom->type == "cylinder"){
                    UrdfCylinderPtr cylGeom = std::static_pointer_cast<UrdfCylinder>(geom);

                    //body->GetCollisionModel()->ClearModel();
                    body->GetCollisionModel()->AddCylinder(cylGeom->radius, cylGeom->radius, cylGeom->length, collision->origin.first);
                    //body->GetCollisionModel()->BuildModel();
                }
                else if(geom->type == "mesh"){
                    UrdfMeshPtr meshGeom = std::static_pointer_cast<UrdfMesh>(geom);

                    AssimpLoader ai(meshGeom->file, ChVectord(meshGeom->scale.x, meshGeom->scale.z, meshGeom->scale.y));
                    std::shared_ptr<geometry::ChTriangleMeshConnected> chMesh = ai.toChronoTriMesh();

                    body->GetCollisionModel()->AddTriangleMesh(*(chMesh.get()), false, false);
                }
                else{
                    std::cout << "Assembly: unknown geometry type (collision)" << std::endl;
                }
            }
        }

        for(auto inertial : link->inertials){
            body->SetPos(_toChronoCoords(inertial->origin.first));
            body->SetMass(inertial->mass);
            body->SetInertiaXX(inertial->inertiaXX);
        }

        //TODO: generate unique ID
        body->GetCollisionModel()->SetFamily(3);
        body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(3);

        body->GetCollisionModel()->BuildModel();
        _bodies.push_back(body);
        _system->AddBody(body);
    }

    //TODO: Assumes first body is the root - this is a kludge and should be fixed
    if(_bodies[0]) _bodies[0]->ConcatenatePreTransformation(initFrame);

    for(auto joint : urdfLoader.getJoints()){
        if(joint->type == "revolute"){
            ChSharedPtr<ChLinkLockRevolute> chJoint = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);

            //Get parent body frame
            ChFrameMoving<> parentFrame(_system->SearchBody(joint->parent.c_str())->GetCoord());

            //Set frame of child relative to parent
            ChFrameMoving<> childFrame(_toChronoCoords(joint->origin.first), _toChronoOrientation(joint->origin.second));
            ChFrameMoving<> childFrameAbs = childFrame >> parentFrame;
            _system->SearchBody(joint->child.c_str())->ConcatenatePreTransformation(childFrameAbs);

            ChFrameMoving<> jointFrame(_toChronoCoords(joint->origin.first), QUNIT);
            ChFrameMoving<> jointFrameAbs = jointFrame >> parentFrame;

            chJoint->Initialize(_system->SearchBody(joint->parent.c_str()), _system->SearchBody(joint->child.c_str()), jointFrameAbs.GetCoord());

            _links.push_back(chJoint);
            _system->AddLink(chJoint);
        }
        else if(joint->type == "fixed"){
            ChSharedPtr<ChLinkLockLock> chJoint = ChSharedPtr<ChLinkLockLock>(new ChLinkLockLock);

            //Get parent body frame
            ChFrameMoving<> parentFrame(_system->SearchBody(joint->parent.c_str())->GetCoord());

            //Set frame of child relative to parent
            ChFrameMoving<> childFrame(_toChronoCoords(joint->origin.first), _toChronoOrientation(joint->origin.second));
            ChFrameMoving<> childFrameAbs = childFrame >> parentFrame;
            _system->SearchBody(joint->child.c_str())->ConcatenatePreTransformation(childFrameAbs);

            chJoint->Initialize(_system->SearchBody(joint->parent.c_str()), _system->SearchBody(joint->child.c_str()), childFrameAbs.GetCoord());

            _links.push_back(chJoint);
            _system->AddLink(chJoint);
        }
        else{
            std::cout << "Assembly: unhandled joint type" << std::endl;
        }
    }
}

Assembly::~Assembly(){
    _bodies.clear();
    _links.clear();
}

ChSystem* Assembly::getSystem(){
    return _system;
}

std::vector<ChBodyPtr> Assembly::getBodies(){
    return _bodies;
}

ChVectord Assembly::_toChronoCoords(ChVectord urdfCoords){
    return ChVectord(urdfCoords.x, urdfCoords.z, urdfCoords.y);
}

ChQuatd Assembly::_toChronoOrientation(ChVectord urdfOrientation){
    return chrono::Q_from_NasaAngles(ChVectord(urdfOrientation.z, urdfOrientation.x, -urdfOrientation.y));
}
