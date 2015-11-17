#include "StaticMeshFactory.h"
#include "StaticMesh.h"

StaticMeshFactory::StaticMeshFactory(ChSystem* system)
    : _system(system)
{
    
}

StaticMeshFactory::~StaticMeshFactory(){
    
}


StaticMeshPtr StaticMeshFactory::createStaticMesh(std::string name, ChVector<double> position, double mass){
    //Temporary default material
    ChDEMMaterialPtr mat(new ChMaterialSurfaceDEM);
    mat->SetYoungModulus(2e6);
    mat->SetFriction(0.4);
    mat->SetRestitution(0.4);    
    
    StaticMeshPtr newStaticMesh = std::make_shared<StaticMesh>(name, position, mass, mat);
    newStaticMesh->_body->SetIdentifier(0);
    _system->AddBody(newStaticMesh->_body);
    
    return newStaticMesh;
}