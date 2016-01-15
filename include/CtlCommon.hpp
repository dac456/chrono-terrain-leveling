#ifndef __CTLCOMMON_H
#define __CTLCOMMON_H

#include <iostream>
#include <sstream>
#include <memory>
#include <string>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include <chrono/physics/ChSystem.h>
#ifdef SIM_USE_CUDA
#include <chrono_parallel/physics/ChSystemParallel.h>
#endif
#include <chrono/physics/ChParticlesClones.h>

#include <chrono/assets/ChAssetLevel.h>
#ifdef SIM_USE_IRRLICHT
#include <irrlicht.h>
#include <chrono_irrlicht/ChIrrAppInterface.h>
#include <chrono_irrlicht/ChIrrApp.h>
#endif

#include <chrono/ChConfig.h>
#include <chrono/utils/ChUtilsCreators.h>
#include <chrono/utils/ChUtilsInputOutput.h>

#define CTL_MAJOR_VERSION 0
#define CTL_MINOR_VERSION 1
#define CTL_PATCH_VERSION 0

#ifdef SIM_USE_CUDA
    #define DEFAULT_BODY new collision::ChCollisionModelParallel, ChMaterialSurfaceBase::DVI
#else
    #define DEFAULT_BODY ChMaterialSurfaceBase::DVI
#endif

#define streq(x, y) (strcmp(x, y) == 0)

//Usually I'm against this, but Chrono classes are all prefixed with 'Ch' so name collision isn't likely
using namespace chrono;

typedef ChSharedPtr<ChMaterialSurface> ChMaterialPtr;
typedef ChSharedPtr<ChMaterialSurfaceDEM> ChDEMMaterialPtr;
typedef ChSharedPtr<ChBody> ChBodyPtr;
typedef ChSharedPtr<ChAssetLevel> ChAssetLevelPtr;
typedef ChSharedPtr<ChLink> ChLinkPtr;

typedef ChVector<double> ChVectord;
typedef ChVector<float> ChVectorf;
typedef ChQuaternion<double> ChQuatd;
typedef ChQuaternion<float> ChQuatf;
typedef ChMatrixNM<double, 4, 4> ChMat44d;
typedef ChMatrixNM<float, 4 ,4> ChMat44f;

//Forward declare
class Assembly;
class AssimpLoader;
class HeightMap;
class ParticleSystem;
class StaticMesh;
class TrackedVehicle;
class UrdfLoader;

#ifdef USE_STD_SHARED_PTR
    #define SHPTR std::shared_ptr
    #define MKSHR std::make_shared
    #define PTRCAST std::static_pointer_cast
#else
    #include <boost/make_shared.hpp>
    #define SHPTR boost::shared_ptr
    #define MKSHR boost::make_shared
    #define PTRCAST boost::static_pointer_cast
#endif

typedef SHPTR<Assembly> AssemblyPtr;
typedef SHPTR<HeightMap> HeightMapPtr;
typedef SHPTR<ParticleSystem> ParticleSystemPtr;
typedef SHPTR<StaticMesh> StaticMeshPtr;
typedef SHPTR<TrackedVehicle> TrackedVehiclePtr;

#endif
