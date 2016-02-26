#include "UrdfLoader.hpp" //tmp
#include "Assembly.hpp" //tmp
#include "StaticMesh.hpp"
#include "TrackedVehicle.hpp"
#include "ParticleSystem.hpp"
#include "AlgorithmBasic.hpp"
#include "HeightMap.hpp"

#include <chrono/core/ChFileutils.h>
#include <chrono_postprocess/ChPovRay.h>
#include <chrono_postprocess/ChPovRayAssetCustom.h>
using namespace postprocess;

double dt = 0.01; //Default timestep
size_t numThreads = 8; //Default thread count
bool renderOffline = false;
double timeout = 5.0;

//temp//
double tolerance = 0.1;

int max_iteration_bilateral = 100;  // 1000;
int max_iteration_normal = 0;
int max_iteration_sliding = 200;  // 2000;
int max_iteration_spinning = 0;

float contact_recovery_speed = -1;

// Periodically monitor maximum bilateral constraint violation
bool monitor_bilaterals = true;
int bilateral_frame_interval = 100;
//temp//

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

    size_t mt = CHOMPfunctions::GetNumProcs();
    std::cout << "mt: " << mt << std::endl;

    //if(numThreads > mt) numThreads = mt;

    #ifdef SIM_USE_PARALLEL
        std::cout << "Using parallel Chrono system" << std::endl;

        ChSystemParallelDVI* system = new ChSystemParallelDVI();
        system->SetParallelThreadNumber(numThreads);
        CHOMPfunctions::SetNumThreads(numThreads);
    #else
        std::cout << "Using single-threaded Chrono system" << std::endl;
        ChSystem* system = new ChSystem();
    #endif

    system->Set_G_acc(ChVector<>(0, -9.81, 0));

    // Set solver parameters
    #ifdef SIM_USE_PARALLEL
        system->GetSettings()->perform_thread_tuning = false;

        system->GetSettings()->solver.use_full_inertia_tensor = false;

        system->GetSettings()->solver.tolerance = tolerance;

        system->GetSettings()->solver.solver_mode = SLIDING;
        system->GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;
        system->GetSettings()->solver.max_iteration_normal = max_iteration_normal;
        system->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
        system->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
        system->GetSettings()->solver.alpha = 0;
        system->GetSettings()->solver.contact_recovery_speed = contact_recovery_speed;
        system->ChangeSolverType(APGD);

        system->GetSettings()->collision.narrowphase_algorithm = NARROWPHASE_HYBRID_MPR;
        system->GetSettings()->collision.collision_envelope = 0.1 * 0.15;
        system->GetSettings()->collision.bins_per_axis = I3(10, 10, 10);
    #else
        system->SetIterLCPmaxItersSpeed(100);  // the higher, the easier to keep the constraints 'mounted'.
        system->SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
        //system->SetIterLCPmaxItersSpeed(70);
        //system->SetIterLCPmaxItersStab(15);
    #endif

    //Default material
    ChMaterialPtr mat(new ChMaterialSurface);
    //mat->SetYoungModulus(2e6);
    mat->SetFriction(0.4);
    mat->SetRestitution(0.4);

    //StaticMeshPtr smGround = std::make_shared<StaticMesh>(static_cast<ChSystem*>(system), "groundplane", "groundplane.obj", ChVectord(0,0,0), mat);


    UrdfLoader urdf(GetChronoDataFile("urdf/Dagu5.urdf"));
    AssemblyPtr testAsm = std::make_shared<Assembly>(urdf, ChVectord(4.0,3.0,0), static_cast<ChSystem*>(system));

    TrackedVehiclePtr dagu = std::make_shared<TrackedVehicle>("dagu001", "shoe_view.obj", "shoe_collision.obj", testAsm, 0.5);
    AlgorithmBasicPtr daguAlg = std::make_shared<AlgorithmBasic>(dagu);

    HeightMapPtr hm = std::make_shared<HeightMap>(GetChronoDataFile("terrain3.png"));
    ParticleSystemPtr particles = std::make_shared<ParticleSystem>(static_cast<ChSystem*>(system), hm, 2.0, 100.0, 0.075, true, false);

    if(renderOffline == false){
        #ifdef SIM_USE_IRRLICHT
            irrlicht::ChIrrApp app(system, L"Terrain Leveling", irr::core::dimension2d<irr::u32>(800,600), false, true);

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

                irrlicht::ChIrrTools::drawGrid(app.GetVideoDriver(), 2, 2, 30, 30,
                                          ChCoordsys<>(ChVector<>(0, 0.01, 0), Q_from_AngX(CH_C_PI_2)),
                                          irr::video::SColor(255, 60, 60, 60), true);

                daguAlg->step(dt);
                app.DoStep();

                app.EndScene();
            }
        #else//SIM_USE_PARALLEL
            std::cout << "Not compiled for use with Irrlicht. Exiting..." << std::endl;
        #endif //SIM_USE_PARALLEL
    }
    else{
        ChPovRay app = ChPovRay(system);

        app.SetTemplateFile(GetChronoDataFile("_template_POV.pov"));
        app.SetOutputScriptFile("rendering_frames.pov");
        app.SetOutputDataFilebase("my_state");
        app.SetPictureFilebase("picture");

        ChFileutils::MakeDirectory("output");
        ChFileutils::MakeDirectory("anim");

        app.SetOutputDataFilebase("output/my_state");
        app.SetPictureFilebase("anim/picture");

        app.SetLight(ChVector<>(-3, 4, 2), ChColor(0.15f, 0.15f, 0.12f), false);
        app.SetCamera(ChVectord(18,8,18), ChVectord(0,0,0), 50.0);

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

        while(system->GetChTime() < timeout){
            std::cout << "start step" << std::endl;

            std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
            daguAlg->step(dt);
            system->DoStepDynamics(dt);
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

            auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
            std::cout << "Step took " << millis << "ms" << std::endl;

            std::cout << "time= " << system->GetChTime() << std::endl;

            app.ExportData();
        }
    }

    delete system;

    return 0;
}
