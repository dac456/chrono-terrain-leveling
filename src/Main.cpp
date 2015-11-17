#include "StaticMesh.h"

double dt = 0.3; //Default timestep
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
    
    ChSystemParallelDEM system;
    CHOMPfunctions::SetNumThreads(numThreads);
    
    system.Set_G_acc(ChVector<>(0, 0, -9.81));
    
    irr::ChIrrAppInterface app(&system, L"Terrain Leveling", irr::core::dimension2d<irr::u32>(800,600), false, true);
    
    //Convienience methods for scene setup
    app.AddTypicalLogo();
    app.AddTypicalCamera();
    app.AddTypicalSky();
    app.AddTypicalLights();
    
    app.SetStepManage(true);
    app.SetTimestep(dt);
    app.SetTryRealtime(true);    
    
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