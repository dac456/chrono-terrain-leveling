#ifndef __TRACKEDVEHICLE_H
#define __TRACKEDVEHICLE_H

#include "CtlCommon.hpp"

class TrackedVehicle{
private:
    std::string _name;
    AssemblyPtr _assembly;
    ChBodyPtr _chassis;

    std::vector<ChBodyPtr> _shoes;
    std::shared_ptr<geometry::ChTriangleMeshConnected> _shoeMesh;
    std::shared_ptr<geometry::ChTriangleMeshConnected> _collisionMesh;

    double _wheelRadius;
    double _wheelBase;

public:
    TrackedVehicle(std::string name, std::string shoeVisFile, std::string shoeColFile, AssemblyPtr assembly, double wheelRadius);
    ChBodyPtr getChassisBody();

    float getWheelRadius();
    float getWheelBase();

    void setSpeeds(double left, double right);

    void warpToRelativePosition(ChVectord p);

private:
    ChBodyPtr _createShoe(ChBodyPtr previousShoeBody, ChVectord shoeDim, ChVectord shoePosition, ChQuatd shoeRotation = QUNIT);

};

#endif
