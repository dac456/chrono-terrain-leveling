#ifndef __CTLCOMMON_H
#define __CTLCOMMON_H

#include <iostream>
#include <memory>
#include <string>

#include <irrlicht.h>

#include <chrono/physics/ChSystem.h>
#include <chrono_parallel/physics/ChSystemParallel.h>
#include <chrono_irrlicht/ChIrrAppInterface.h>
#include <chrono_irrlicht/ChIrrApp.h>

#include <chrono/ChConfig.h>
#include <chrono/utils/ChUtilsCreators.h>
#include <chrono/utils/ChUtilsInputOutput.h>

//Usually I'm against this, but Chrono classes are all prefixed with 'Ch' so name collision isn't likely
using namespace chrono;

typedef ChSharedPtr<ChMaterialSurface> ChMaterialPtr;
typedef ChSharedPtr<ChMaterialSurfaceDEM> ChDEMMaterialPtr;
typedef ChSharedPtr<ChBody> ChBodyPtr;

typedef ChVector<double> ChVectord;
typedef ChVector<float> ChVectorf;

//Forward declare
class AssimpLoader;
class StaticMesh;
class StaticMeshFactory;
class TrackedVehicle;
class TrackedVehicleFactory;

typedef std::shared_ptr<StaticMesh> StaticMeshPtr;
typedef std::shared_ptr<TrackedVehicle> TrackedVehiclePtr;

#endif