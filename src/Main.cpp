#include "UrdfLoader.hpp" //tmp
#include "Assembly.hpp" //tmp
#include "StaticMesh.hpp"
#include "TrackedVehicle.hpp"
#include "ParticleSystem.hpp"

#include <chrono/core/ChFileutils.h>
#include <chrono_postprocess/ChPovRay.h>
#include <chrono_postprocess/ChPovRayAssetCustom.h>
using namespace postprocess;

double dt = 0.01; //Default timestep
size_t numThreads = 8; //Default thread count
bool renderOffline = false;
double timeout = 5.0;

void readArgs(int argc, char* argv[]){
    int current = 1;
    if(argc > 1){
        while(current < argc && argv[current] != nullptr){
            if(strcmp(argv[current],"--dt") == 0){
                if(argv[current+1] != nullptr){
                    dt = atof(argv[current+1]);
                    current++;
                }
            }
            if(strcmp(argv[current],"--nt") == 0){
                if(argv[current+1] != nullptr){
                    numThreads = atoi(argv[current+1]);
                    current++;
                }
            }
            if(strcmp(argv[current], "--offline") == 0){
                renderOffline = true;
            }
            if(strcmp(argv[current], "--timeout") == 0){
                if(argv[current+1] != nullptr){
                    timeout = atof(argv[current+1]);
                    current++;
                }
            }

            current++;
        }
    }
}

int main(int argc, char* argv[])
{
    readArgs(argc, argv);
    std::cout << "dt: " << dt << std::endl;
    std::cout << "nt: " << numThreads << std::endl;

    #ifdef SIM_USE_CUDA
        ChSystemParallelDVI system;
        system.SetParallelThreadNumber(numThreads);
        CHOMPfunctions::SetNumThreads(numThreads);
    #else
        ChSystem system(16000, 20);
    #endif

    system.Set_G_acc(ChVector<>(0, -9.81, 0));

    // Set solver parameters
    #ifdef SIM_USE_CUDA
        system.GetSettings()->solver.max_iteration_bilateral = 100;
        system.GetSettings()->solver.tolerance = 1e-3;

        system.GetSettings()->collision.narrowphase_algorithm = NARROWPHASE_HYBRID_MPR;
        system.GetSettings()->collision.bins_per_axis = I3(10, 10, 10);
    #endif

    system.SetIterLCPmaxItersSpeed(100);  // the higher, the easier to keep the constraints 'mounted'.
    system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
    //system.SetIterLCPmaxItersSpeed(70);
    //system.SetIterLCPmaxItersStab(15);
    system.SetParallelThreadNumber(numThreads);

    //Default material
    ChMaterialPtr mat(new ChMaterialSurface);
    //mat->SetYoungModulus(2e6);
    mat->SetFriction(0.4);
    mat->SetRestitution(0.4);

    //StaticMeshPtr smGround = std::make_shared<StaticMesh>(static_cast<ChSystem*>(&system), "groundplane", "groundplane.obj", ChVectord(0,0,0), mat);


    UrdfLoader urdf(GetChronoDataFile("urdf/Dagu5.urdf"));
    AssemblyPtr testAsm = std::make_shared<Assembly>(urdf, ChVectord(-10,4.5,0), static_cast<ChSystem*>(&system));

    TrackedVehiclePtr dagu = std::make_shared<TrackedVehicle>("dagu001", "shoe_view.obj", "shoe_collision.obj", testAsm, 0.5);
    dagu->setSpeeds(CH_C_PI, CH_C_PI);

    ParticleSystemPtr particles = std::make_shared<ParticleSystem>(static_cast<ChSystem*>(&system), ChVectord(30,4,30), 100.0, 0.15, true, true);

    if(renderOffline == false){
        irr::ChIrrApp app(&system, L"Terrain Leveling", irr::core::dimension2d<irr::u32>(800,600), false, true);

        app.SetStepManage(true);
        app.SetTimestep(dt);
        app.SetTryRealtime(true);

        //Convienience methods for scene setup
        app.AddTypicalLogo();
        app.AddTypicalCamera(irr::core::vector3df(14,8,14));
        app.AddTypicalSky();
        app.AddTypicalLights();

        app.AssetBindAll();
        app.AssetUpdateAll();

        while(app.GetDevice()->run()){
            app.BeginScene();
            app.DrawAll();

            irr::ChIrrTools::drawGrid(app.GetVideoDriver(), 2, 2, 30, 30,
                                      ChCoordsys<>(ChVector<>(0, 0.01, 0), Q_from_AngX(CH_C_PI_2)),
                                      irr::video::SColor(255, 60, 60, 60), true);

            app.DoStep();

            app.EndScene();
        }
    }
    else{
        ChPovRay app = ChPovRay(&system);

        app.SetTemplateFile(GetChronoDataFile("_template_POV.pov"));
        app.SetOutputScriptFile("rendering_frames.pov");
        app.SetOutputDataFilebase("my_state");
        app.SetPictureFilebase("picture");

        ChFileutils::MakeDirectory("output");
        ChFileutils::MakeDirectory("anim");

        app.SetOutputDataFilebase("output/my_state");
        app.SetPictureFilebase("anim/picture");

        app.SetLight(ChVector<>(-3, 4, 2), ChColor(0.15f, 0.15f, 0.12f), false);
        app.SetCamera(ChVectord(14,8,14), ChVectord(0,0,0), 50.0);

        // --Optional: add further POV commands, for example in this case:
        //     create an area light for soft shadows
        app.SetCustomPOVcommandsScript(
            " \
            light_source {   \
                <2, 10, -3>  \
                color rgb<1.2,1.2,1.2> \
                area_light <4, 0, 0>, <0, 0, 4>, 8, 8 \
                adaptive 1 \
                jitter\
            } \
        ");

        app.AddAll();

        app.ExportScript();

        while(system.GetChTime() < timeout){
            system.DoStepDynamics(dt);

            std::cout << "time= " << system.GetChTime() << std::endl;

            app.ExportData();
        }
    }

    return 0;
}
