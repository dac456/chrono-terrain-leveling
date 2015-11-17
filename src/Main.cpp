#include <irrlicht.h>

#include <chrono/physics/ChSystem.h>
#include <chrono_irrlicht/ChIrrAppInterface.h>

//Usually I'm against this, but Chrono classes are all prefixed with 'Ch' so name collision isn't likely
using namespace chrono;

int main(int argc, char* argv[])
{
    ChSystem system;
    irr::ChIrrAppInterface app(&system, L"Terrain Leveling", irr::core::dimension2d<irr::u32>(800,600), false, true);
    
    //Convienience methods for scene setup
    irr::ChIrrWizard::add_typical_Logo(app.GetDevice());
    irr::ChIrrWizard::add_typical_Sky(app.GetDevice());
    irr::ChIrrWizard::add_typical_Lights(app.GetDevice());
    irr::ChIrrWizard::add_typical_Camera(app.GetDevice(), irr::core::vector3df(0, 0, -6), irr::core::vector3df(-2, 2, 0));
    
    app.SetStepManage(true);
    app.SetTimestep(0.03);
    app.SetTryRealtime(true);    
    
    while(app.GetDevice()->run()){
        app.GetVideoDriver()->beginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        
        irr::ChIrrTools::drawGrid(app.GetVideoDriver(), 2, 2, 30, 30,
                                  ChCoordsys<>(ChVector<>(0, 0.01, 0), Q_from_AngX(CH_C_PI_2)),
                                  irr::video::SColor(255, 60, 60, 60), true);
                             
        app.DoStep();
        
        app.GetVideoDriver()->endScene();
    }
    
    return 0;
}