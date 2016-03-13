#include "UrdfLoader.hpp" //tmp
#include "Assembly.hpp" //tmp
#include "StaticMesh.hpp"
#include "TrackedVehicle.hpp"
#include "ParticleSystem.hpp"
#include "AlgorithmBasic.hpp"
#include "HeightMap.hpp"
#include "RayGrid.hpp"

#include <chrono/core/ChFileutils.h>
#include <chrono_postprocess/ChPovRay.h>
#include <chrono_postprocess/ChPovRayAssetCustom.h>
using namespace postprocess;

#include <boost/program_options.hpp>
namespace po = boost::program_options;

double dt = 0.01; //Default timestep
size_t numThreads = 8; //Default thread count
bool renderOffline = false;
bool rayGridOnly = false;
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

int main(int argc, char* argv[])
{
    po::options_description desc("supported options");
    desc.add_options()
        ("help", "Display program options.")
        ("dt", po::value<double>(), "Timestep size in seconds (default 1e-2)")
        ("nt", po::value<int>(), "Number of threads to use where parallel is available (default 8)")
        ("timeout", po::value<double>(), "Timestep to halt simulation in seconds (default 5.0)")
        ("offline", "Render offline")
        ("raygrid", "Output from raygrid only with --offline")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if(vm.count("help")){
        std::cout << desc << std::endl;
        return 1;
    }

    if(vm.count("dt")){
        dt = vm["dt"].as<double>();
    }

    if(vm.count("nt")){
        numThreads = vm["nt"].as<int>();
    }

    if(vm.count("timeout")){
        timeout = vm["tiemout"].as<double>();
    }

    if(vm.count("offline")){
        renderOffline = true;
    }

    if(vm.count("raygrid")){
        rayGridOnly = true;
    }

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
        ChSystem* system = new ChSystem(1000000);
        //system->ChangeCollisionSystem(new collision::ChCollisionSystemSpheres(1000000));
        system->SetParallelThreadNumber(numThreads);
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


    const double particleSize = 0.15;

    UrdfLoader urdf(GetChronoDataFile("urdf/Dagu5.urdf"));
    AssemblyPtr testAsm = std::make_shared<Assembly>(urdf, ChVectord(-30.0,2.0,0.0), static_cast<ChSystem*>(system));

    TrackedVehiclePtr dagu = std::make_shared<TrackedVehicle>("dagu001", "shoe_view.obj", "shoe_collision.obj", testAsm, 0.5);
    AlgorithmBasicPtr daguAlg = std::make_shared<AlgorithmBasic>(dagu);

    HeightMapPtr hm = std::make_shared<HeightMap>(GetChronoDataFile("mound.png"));
    ParticleSystemPtr particles = std::make_shared<ParticleSystem>(static_cast<ChSystem*>(system), hm, 1.0, 100.0, particleSize, true, false);

    //TODO: set ray grid from height map? One ray cell per pixel
    ChFileutils::MakeDirectory("raygrid");
    RayGridPtr rg = std::make_shared<RayGrid>(system, ChVectord(0.0, 4.0, 0.0), hm->getHeight()*particleSize*2.0, hm->getWidth()*particleSize*2.0, hm->getHeight()*2, hm->getWidth()*2);

    if(renderOffline == false){
        #ifdef SIM_USE_IRRLICHT
            irrlicht::ChIrrApp app(system, L"Terrain Leveling", irr::core::dimension2d<irr::u32>(800,600), false, true);

            app.SetStepManage(true);
            app.SetTimestep(dt);
            app.SetTryRealtime(true);

            //Convienience methods for scene setup
            app.AddTypicalLogo();
            app.AddTypicalCamera(irr::core::vector3df(14,18,14));
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

                rg->castRays();
                daguAlg->step(dt);

                //for(auto r : rg->getRayOrigins()){
                //    irrlicht::ChIrrTools::drawSegment(app.GetVideoDriver(), r, ChVectord(r.x, -1.0, r.z));
                //}

                app.DoStep();

                app.EndScene();
            }
        #else//SIM_USE_PARALLEL
            std::cout << "Not compiled for use with Irrlicht. Exiting..." << std::endl;
        #endif //SIM_USE_PARALLEL
    }
    else{
        if(!rayGridOnly){
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
            app.SetCamera(ChVectord(14,18,14), ChVectord(0,0,0), 50.0);

            // --Optional: add further POV commands, for example in this case:
            //     create an area light for soft shadows
            /*app.SetCustomPOVcommandsScript(
                " \
                light_source {   \
                    <2, 10, -3>  \
                    color rgb<1.2,1.2,1.2> \
                    area_light <4, 0, 0>, <0, 0, 4>, 8, 8 \
                    adaptive 1 \
                    jitter\
                } \
            ");*/
            app.SetCustomPOVcommandsScript(
                " \
                light_source { \
                    <1000,1000,-1000>, rgb <1,1,1> \
                } \
            ");

            app.AddAll();

            app.ExportScript();

            while(system->GetChTime() < timeout){
                std::cout << "start step" << std::endl;

                std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
                if(system->GetChTime() > 0.2) daguAlg->step(dt);
                if(system->GetChTime() > 0.2) rg->castRays();
                system->DoStepDynamics(dt);
                std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

                auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
                std::cout << "Step took " << millis << "ms" << std::endl;

                std::cout << "time= " << system->GetChTime() << std::endl;

                if(system->GetChTime() > 1.0) app.ExportData();
            }
        }
        else{
            while(system->GetChTime() < timeout){
                std::cout << "start step" << std::endl;

                std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
                if(system->GetChTime() > 0.2) daguAlg->step(dt);
                if(system->GetChTime() > 0.2) rg->castRays();
                system->DoStepDynamics(dt);
                std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

                auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
                std::cout << "Step took " << millis << "ms" << std::endl;

                std::cout << "time= " << system->GetChTime() << std::endl;
            }
        }
    }

    delete system;

    return 0;
}
