#include "Assembly.hpp"
#include "UrdfLoader.hpp"

Assembly::Assembly(ChSystem* system)
    : _system(system)
{

}

Assembly::Assembly(UrdfLoader urdfLoader, ChSystem* system)
    : _system(system)
{
    for(auto& link : urdfLoader.getLinks()){
        ChBodyPtr body = ChBodyPtr(new ChBody(DEFAULT_BODY));
        body->SetBodyFixed(false);
        body->SetCollide(true);

        for(auto& visual : link.visuals){
            if(visual.geometry.type != ""){
                UrdfGeometry* geom = &visual.geometry;
                if(geom->type == "box"){
                    UrdfBox* boxGeom = static_cast<UrdfBox*>(geom);
                    std::cout << boxGeom->dim.x << std::endl;

                    ChSharedPtr<ChBoxShape> box(new ChBoxShape);
                    box->GetBoxGeometry().Pos = ChVector<>(0, 0, 0);
                    box->GetBoxGeometry().Size = ChVector<>(boxGeom->dim.x, boxGeom->dim.y, boxGeom->dim.z);
                    body->AddAsset(box);
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
