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
    newVehicle->_lfWheel->SetIdentifier(_nextId+1);
    
    _system->AddBody(newVehicle->_body);
    _system->AddBody(newVehicle->_lfWheel);
    _system->AddBody(newVehicle->_rfWheel);
    
    _system->AddLink(newVehicle->_lfWheelLink);
    _system->AddLink(newVehicle->_rfWheelLink);
    
    _nextId += 5;
    return newVehicle;
}