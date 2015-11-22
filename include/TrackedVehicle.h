#ifndef __TRACKEDVEHICLE_H
#define __TRACKEDVEHICLE_H

#include "CtlCommon.h"

class TrackedVehicle{
private:
    std::string _name;
    ChBodyPtr _body;
    ChBodyPtr _lfWheel;
    ChBodyPtr _lbWheel;
    ChBodyPtr _rfWheel;
    ChBodyPtr _rbWheel;
    
public:
    TrackedVehicle(std::string name, std::string bodyFile, std::string wheelFile, double mass);
    
    
    friend class TrackedVehicleFactory;
};

#endif