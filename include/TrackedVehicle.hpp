#ifndef __TRACKEDVEHICLE_H
#define __TRACKEDVEHICLE_H

#include "CtlCommon.hpp"

class TrackedVehicle{
private:
    std::string _name;
    AssemblyPtr _assembly;

    std::shared_ptr<geometry::ChTriangleMeshConnected> _shoeMesh;
    std::shared_ptr<geometry::ChTriangleMeshConnected> _collisionMesh;

public:
    TrackedVehicle(std::string name, std::string shoeVisFile, std::string shoeColFile, AssemblyPtr assembly);

private:
    ChBodyPtr _createShoe(ChBodyPtr previousShoeBody, ChVectord shoeDim, ChVectord shoePosition, ChQuatd shoeRotation = QUNIT);

    friend class TrackedVehicleFactory;
};

#endif
