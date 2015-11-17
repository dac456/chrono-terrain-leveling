#include <iostream>

#include <irrlicht.h>

#include <chrono/physics/ChSystem.h>
#include <chrono_irrlicht/ChIrrAppInterface.h>

//Usually I'm against this, but Chrono classes are all prefixed with 'Ch' so name collision isn't likely
using namespace chrono;

double dt = 0.3; //Default timestep

void readArgs(int argc, char* argv[]){
    int current = 1;
    if(argc > 1){
        if(strcmp(argv[current],"--dt") == 0){
            if(argv[current+1] != nullptr){
                dt = atof(argv[current+1]);
            }
        }
    }
}

int main(int argc, char* argv[])
{
    readArgs(argc, argv);
    
    ChSystem system;
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