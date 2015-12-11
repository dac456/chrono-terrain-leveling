#include "Assembly.hpp"
#include "UrdfLoader.hpp"
#include "AssimpLoader.hpp"

Assembly::Assembly(ChSystem* system)
    : _system(system)
{

}

Assembly::Assembly(UrdfLoader urdfLoader, ChSystem* system)
    : _system(system)
{
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
                    box->GetBoxGeometry().Rot = chrono::Q_from_NasaAngles(ChVectord(visual->origin.second.z, visual->origin.second.x, visual->origin.second.y));
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

        body->GetCollisionModel()->BuildModel();
        _bodies.push_back(body);
        _system->AddBody(body);
    }

    for(auto joint : urdfLoader.getJoints()){
        if(joint->type == "revolute"){
            ChSharedPtr<ChLinkLockRevolute> chJoint = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);

            //Set frame of child relative to parent
            ChFrameMoving<> childFrame(_toChronoCoords(joint->origin.first), chrono::Q_from_NasaAngles(ChVectord(joint->origin.second.z, joint->origin.second.x, joint->origin.second.y)));
            _system->SearchBody(joint->child.c_str())->ConcatenatePreTransformation(childFrame);

            //Set the axis of revolution
            ChFrameMoving<> jointRot(ChVectord(0,0,0), chrono::Q_from_NasaAngles(ChVectord(joint->origin.second.z, joint->origin.second.x, joint->origin.second.y)));

            //Absolute frame for joint
            ChFrameMoving<> jointFrameAbs = jointRot >> childFrame;

            chJoint->Initialize(_system->SearchBody(joint->parent.c_str()), _system->SearchBody(joint->child.c_str()), jointFrameAbs.GetCoord());

            _links.push_back(chJoint);
            _system->AddLink(chJoint);
        }
    }
}

Assembly::~Assembly(){
    _bodies.clear();
    _links.clear();
}

ChVectord Assembly::_toChronoCoords(ChVectord urdfCoords){
    return ChVectord(urdfCoords.x, urdfCoords.z, urdfCoords.y);
}
