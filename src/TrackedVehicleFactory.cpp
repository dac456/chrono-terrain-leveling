#include "TrackedVehicleFactory.hpp"
#include "TrackedVehicle.hpp"

TrackedVehicleFactory::TrackedVehicleFactory(ChSystem* system)
    : _system(system),
      _nextId(0)
{

}

TrackedVehicleFactory::~TrackedVehicleFactory(){

}

TrackedVehiclePtr TrackedVehicleFactory::createTrackedVehicle(std::string name){
    return nullptr;
}
