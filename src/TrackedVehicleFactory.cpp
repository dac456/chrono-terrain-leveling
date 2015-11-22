#include "TrackedVehicleFactory.hpp"
#include "TrackedVehicle.hpp"

TrackedVehicleFactory::TrackedVehicleFactory(ChSystem* system)
    : _system(system),
      _nextId(0)
{
    
}

TrackedVehicleFactory::~TrackedVehicleFactory(){
    
}

TrackedVehiclePtr TrackedVehicleFactory::createTrackedVehicle(std::string name, std::string bodyFile, std::string wheelFile, double bodyMass){
    TrackedVehiclePtr newVehicle = std::make_shared<TrackedVehicle>(name, bodyFile, wheelFile, bodyMass);
    newVehicle->_body->SetIdentifier(_nextId);
    _system->AddBody(newVehicle->_body);
    
    _nextId += 5;
    return newVehicle;
}