#include "Assembly.hpp"
#include "UrdfLoader.hpp"

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

        for(auto visual : link->visuals){
            if(visual->geometry->type != ""){
                UrdfGeometryPtr geom = visual->geometry;
                if(geom->type == "box"){
                    UrdfBoxPtr boxGeom = std::static_pointer_cast<UrdfBox>(geom);

                    ChSharedPtr<ChBoxShape> box(new ChBoxShape);
                    box->GetBoxGeometry().Pos = visual->origin.first;
                    box->GetBoxGeometry().Rot = chrono::Q_from_NasaAngles(ChVectord(visual->origin.second.z, visual->origin.second.x, visual->origin.second.y));
                    box->GetBoxGeometry().Size = ChVector<>(boxGeom->dim.x, boxGeom->dim.y, boxGeom->dim.z);
                    body->AddAsset(box);
                }
            }
        }

        for(auto collision : link->collisions){
            if(collision->geometry->type != ""){
                UrdfGeometryPtr geom = collision->geometry;
                if(geom->type == "box"){
                    UrdfBoxPtr boxGeom = std::static_pointer_cast<UrdfBox>(geom);

                    body->GetCollisionModel()->ClearModel();
                    body->GetCollisionModel()->AddBox(boxGeom->dim.x, boxGeom->dim.y, boxGeom->dim.z, collision->origin.first);
                    body->GetCollisionModel()->BuildModel();
                }
            }
        }

        for(auto inertial : link->inertials){
            body->SetPos(inertial->origin.first);
            body->SetMass(inertial->mass);
            body->SetInertiaXX(inertial->inertiaXX);
        }

        _bodies.push_back(body);
        _system->AddBody(body);
    }

    for(auto joint : urdfLoader.getJoints()){
        if(joint->type == "revolute"){
            ChSharedPtr<ChLinkLockRevolute> chJoint = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);
            chJoint->Initialize(_system->SearchBody(joint->parent.c_str()), _system->SearchBody(joint->child.c_str()), ChCoordsys<>(joint->origin.first, QUNIT));

            _links.push_back(chJoint);
            _system->AddLink(chJoint);
        }
    }
}

Assembly::~Assembly(){
    _bodies.clear();
}
