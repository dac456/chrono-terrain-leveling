#include "UrdfLoader.hpp" //tmp
#include "Assembly.hpp" //tmp
#include "StaticMesh.hpp"
#include "TrackedVehicle.hpp"

double dt = 0.01; //Default timestep
size_t numThreads = 8; //Default thread count

void readArgs(int argc, char* argv[]){
    int current = 1;
    if(argc > 1){
        while(current < argc){
            if(strcmp(argv[current],"--dt") == 0){
                if(argv[current+1] != nullptr){
                    dt = atof(argv[current+1]);
                    current++;
                }
                current++;
            }
            if(strcmp(argv[current],"--nt") == 0){
                if(argv[current+1] != nullptr){
                    numThreads = atoi(argv[current+1]);
                    current++;
                }
                current++;
            }
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
        ChSystem system;
    #endif

    system.Set_G_acc(ChVector<>(0, -9.81, 0));

    // Set solver parameters
    #ifdef SIM_USE_CUDA
        system.GetSettings()->solver.max_iteration_bilateral = 100;
        system.GetSettings()->solver.tolerance = 1e-3;

        system.GetSettings()->collision.narrowphase_algorithm = NARROWPHASE_HYBRID_MPR;
        system.GetSettings()->collision.bins_per_axis = I3(10, 10, 10);
    #endif

    irr::ChIrrApp app(&system, L"Terrain Leveling", irr::core::dimension2d<irr::u32>(800,600), false, true);

    //Convienience methods for scene setup
    app.AddTypicalLogo();
    app.AddTypicalCamera();
    app.AddTypicalSky();
    app.AddTypicalLights();

    system.SetIterLCPmaxItersSpeed(100);  // the higher, the easier to keep the constraints 'mounted'.
    system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);

    app.SetStepManage(true);
    app.SetTimestep(dt);
    app.SetTryRealtime(true);

    //Default material
    ChMaterialPtr mat(new ChMaterialSurface);
    //mat->SetYoungModulus(2e6);
    mat->SetFriction(0.4);
    mat->SetRestitution(0.4);

    StaticMeshPtr smGround = std::make_shared<StaticMesh>(static_cast<ChSystem*>(&system), "groundplane", "groundplane.obj", ChVectord(0,0,0), mat);

    UrdfLoader urdf(GetChronoDataFile("urdf/Dagu5.urdf"));
    AssemblyPtr testAsm = std::make_shared<Assembly>(urdf, ChVectord(0,2,0), static_cast<ChSystem*>(&system));

    TrackedVehiclePtr dagu = std::make_shared<TrackedVehicle>("dagu001", "shoe_view.obj", "shoe_collision.obj", testAsm, 0.5);

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

    return 0;
}
