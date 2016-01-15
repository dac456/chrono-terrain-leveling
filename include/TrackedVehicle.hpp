#ifndef __TRACKEDVEHICLE_H
#define __TRACKEDVEHICLE_H

#include "CtlCommon.hpp"

class TrackedVehicle{
private:
    std::string _name;
    AssemblyPtr _assembly;

    SHPTR<geometry::ChTriangleMeshConnected> _shoeMesh;
    SHPTR<geometry::ChTriangleMeshConnected> _collisionMesh;

    double _wheelRadius;

public:
    TrackedVehicle(std::string name, std::string shoeVisFile, std::string shoeColFile, AssemblyPtr assembly, double wheelRadius);

    void setSpeeds(double left, double right);

private:
    ChBodyPtr _createShoe(ChBodyPtr previousShoeBody, ChVectord shoeDim, ChVectord shoePosition, ChQuatd shoeRotation = QUNIT);

    friend class TrackedVehicleFactory;
};

#endif
