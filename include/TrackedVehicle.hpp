#ifndef __TRACKEDVEHICLE_H
#define __TRACKEDVEHICLE_H

#include "CtlCommon.hpp"

class TrackedVehicle{
private:
    std::string _name;
    AssemblyPtr _assembly;

    std::shared_ptr<geometry::ChTriangleMeshConnected> _shoeMesh;

public:
    TrackedVehicle(std::string name, std::string shoeFile, AssemblyPtr assembly);

private:
    ChBodyPtr _createShoe(size_t idx, ChBodyPtr previousShoeBody, ChVectord shoeDim, ChVectord backWheelPos, ChQuatd shoeRotation = QUNIT);

    friend class TrackedVehicleFactory;
};

#endif
