#ifndef __TRACKEDVEHICLEFACTORY_H
#define __TRACKEDVEHICLEFACTORY_H

#include "CtlCommon.hpp"

class TrackedVehicleFactory{
private:
    ChSystem* _system;
    size_t _nextId;

public:
    TrackedVehicleFactory(ChSystem* system);
    ~TrackedVehicleFactory();

    TrackedVehiclePtr createTrackedVehicle(std::string names);

};

#endif
