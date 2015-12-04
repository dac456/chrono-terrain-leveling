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
        body->SetBodyFixed(false);
        body->SetCollide(true);

        for(auto visual : link->visuals){
            if(visual->geometry->type != ""){
                UrdfGeometryPtr geom = visual->geometry;
                if(geom->type == "box"){
                    UrdfBoxPtr boxGeom = std::static_pointer_cast<UrdfBox>(geom);

                    ChSharedPtr<ChBoxShape> box(new ChBoxShape);
                    box->GetBoxGeometry().Pos = ChVector<>(visual->origin.first.x, visual->origin.first.y, visual->origin.first.z);
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

        _bodies.push_back(body);
    }

    for(auto& body : _bodies){
        _system->AddBody(body);
    }
}

Assembly::~Assembly(){
    _bodies.clear();
}
