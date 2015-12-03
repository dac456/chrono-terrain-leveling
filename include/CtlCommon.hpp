#ifndef __CTLCOMMON_H
#define __CTLCOMMON_H

#include <iostream>
#include <memory>
#include <string>

#include <boost/filesystem.hpp>

#include <irrlicht.h>

#include <chrono/physics/ChSystem.h>
#ifdef SIM_USE_CUDA
#include <chrono_parallel/physics/ChSystemParallel.h>
#endif
#include <chrono_irrlicht/ChIrrAppInterface.h>
#include <chrono_irrlicht/ChIrrApp.h>

#include <chrono/ChConfig.h>
#include <chrono/utils/ChUtilsCreators.h>
#include <chrono/utils/ChUtilsInputOutput.h>

#ifdef SIM_USE_CUDA
    #define DEFAULT_BODY new collision::ChCollisionModelParallel, ChMaterialSurfaceBase::DVI
#else
    #define DEFAULT_BODY ChMaterialSurfaceBase::DVI
#endif

//Usually I'm against this, but Chrono classes are all prefixed with 'Ch' so name collision isn't likely
using namespace chrono;

typedef ChSharedPtr<ChMaterialSurface> ChMaterialPtr;
typedef ChSharedPtr<ChMaterialSurfaceDEM> ChDEMMaterialPtr;
typedef ChSharedPtr<ChBody> ChBodyPtr;

typedef ChVector<double> ChVectord;
typedef ChVector<float> ChVectorf;
typedef ChMatrixNM<double, 4, 4> ChMat44d;
typedef ChMatrixNM<float, 4 ,4> ChMat44f;

//Forward declare
class AssimpLoader;
class StaticMesh;
class StaticMeshFactory;
class TrackedVehicle;
class TrackedVehicleFactory;

typedef std::shared_ptr<StaticMesh> StaticMeshPtr;
typedef std::shared_ptr<TrackedVehicle> TrackedVehiclePtr;

#endif
