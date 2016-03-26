#include <chrono/core/ChFileutils.h>

#include "Experiment.hpp"
#include "Config.hpp"

#include "UrdfLoader.hpp"
#include "Assembly.hpp"
#include "StaticMesh.hpp"
#include "TrackedVehicle.hpp"
#include "ParticleSystem.hpp"
#include "AlgorithmBasic.hpp"
#include "HeightMap.hpp"
#include "RayGrid.hpp"

Experiment::Experiment(ChSystem* system, std::string expConfigFile)
    : _frameCount(0)
    , _linearVel(0.0)
    , _angularVel(0.0)
{
    Config cfg(expConfigFile);
    _vm = cfg.getVariables();

    fs::create_directories(_vm["output_directory_prefix"].as<std::string>() + "framedata/");
    std::cout << fs::canonical(_vm["chrono_data_path"].as<std::string>()).string() << std::endl;
    SetChronoDataPath(fs::canonical(_vm["chrono_data_path"].as<std::string>() + '/').string() + '/');

    _name = _vm["experiment.name"].as<std::string>();

    double particleSize = _vm["map.particle_radius"].as<double>();

    ChVectord startPos = ChVectord(
        _vm["vehicle.x"].as<double>(),
        _vm["vehicle.y"].as<double>(),
        _vm["vehicle.z"].as<double>()
    );
    ChQuatd startOrient;
    startOrient.Q_from_NasaAngles(ChVectord(
        _vm["vehicle.h"].as<double>(),
        _vm["vehicle.r"].as<double>(),
        _vm["vehicle.p"].as<double>()
    ));

    UrdfLoader urdf(GetChronoDataFile("urdf/Dagu5.urdf"));
    AssemblyPtr testAsm = std::make_shared<Assembly>(urdf, startPos, startOrient, static_cast<ChSystem*>(system));

    TrackedVehiclePtr dagu = std::make_shared<TrackedVehicle>("dagu001", "shoe_view.obj", "shoe_collision.obj", testAsm, 0.5);
    _platform = std::make_shared<Platform>(dagu);

    //TODO: eventually want to load simple algorithms, such as driving forward and backward repeatedly
    _linearVel = _vm["experiment.linear"].as<double>();
    _angularVel = _vm["experiment.angular"].as<double>();
    _platform->setDesiredLinearVelocity(_linearVel);
    _platform->setDesiredAngularVelocity(_angularVel);

    HeightMapPtr hm = std::make_shared<HeightMap>(GetChronoDataFile(_vm["map.filename"].as<std::string>()));
    ParticleSystemPtr particles = std::make_shared<ParticleSystem>(static_cast<ChSystem*>(system), hm, _vm["map.scale"].as<double>(), 50.0, particleSize, true, false);

    double rgRes = _vm["raygrid.resolution"].as<double>();
    _rayGrid = std::make_shared<RayGrid>(system, _vm, ChVectord(0.0, 4.0, 0.0), hm->getHeight()*particleSize*2.0, hm->getWidth()*particleSize*2.0, hm->getHeight()*rgRes, hm->getWidth()*rgRes);
}

Experiment::~Experiment(){}

void Experiment::step(double dt){
    _platform->move();
    _rayGrid->castRays();

    writeFrame();
}

void Experiment::writeFrame(){
    std::stringstream ssf;
    ssf << _vm["output_directory_prefix"].as<std::string>() << "framedata/frame" << _frameCount << ".dat";

    std::ofstream fout(ssf.str());

    fout << "dppx = " << _rayGrid->getDistancePerPixel().first << std::endl;
    fout << "dppy = " << _rayGrid->getDistancePerPixel().second << std::endl;

    fout << "rw = " << _rayGrid->getWidth() << std::endl;
    fout << "rl = " << _rayGrid->getLength() << std::endl;

    std::pair<int,int> transformedPos = _rayGrid->transformRealPositionToGrid(_platform->getChassisBody()->GetPos());
    fout << "vx = " << transformedPos.first << std::endl;
    fout << "vy = " << transformedPos.second << std::endl;
    fout << "vtheta = " << _platform->getAccelYaw() << std::endl;

    fout << "vdl = " << _linearVel << std::endl;
    fout << "vda = " << _angularVel << std::endl;

    ChVectord aabbMin, aabbMax;
    _platform->getChassisBody()->GetTotalAABB(aabbMin, aabbMax);

    std::pair<int,int> gx1 = _rayGrid->transformRealPositionToGrid(_platform->getChassisBody()->GetPos() + ChVectord(0.0,0.0,aabbMax.z));
    std::pair<int,int> gx2 = _rayGrid->transformRealPositionToGrid(_platform->getChassisBody()->GetPos() + ChVectord(0.0,0.0,aabbMin.z));

    fout << "gx1x = " << gx1.first << std::endl;
    fout << "gx1y = " << gx1.second << std::endl;
    fout << "gx2x = " << gx2.first << std::endl;
    fout << "gx2y = " << gx2.second << std::endl;

    std::pair<int,int> gy1 = _rayGrid->transformRealPositionToGrid(_platform->getChassisBody()->GetPos() + ChVectord(aabbMax.x,0.0,0.0));
    std::pair<int,int> gy2 = _rayGrid->transformRealPositionToGrid(_platform->getChassisBody()->GetPos() + ChVectord(aabbMin.x,0.0,0.0));

    fout << "gy1x = " << gy1.first << std::endl;
    fout << "gy1y = " << gy1.second << std::endl;
    fout << "gy2x = " << gy2.first << std::endl;
    fout << "gy2y = " << gy2.second << std::endl;

    /*std::pair<int,int> gx1Pos = _rayGrid->transformRealPositionToGrid(_platform->getChassisBody()->GetPos() + ChVectord(0.0,0.0,aabbMax.z));
    double gx1 = _rayGrid->getRayResults()[gx1Pos.first + (gx1Pos.second*_rayGrid->getNumDivWidth())];
    std::pair<int,int> gx2Pos = _rayGrid->transformRealPositionToGrid(_platform->getChassisBody()->GetPos() + ChVectord(0.0,0.0,aabbMin.z));
    double gx2 = _rayGrid->getRayResults()[gx2Pos.first + (gx2Pos.second*_rayGrid->getNumDivWidth())];

    fout << "gx1 = " << gx1 << std::endl;
    fout << "gx2 = " << gx2 << std::endl;
    fout << "lgx1 = " << _rayGrid->getLastRayResults()[gx1Pos.first + (gx1Pos.second*_rayGrid->getNumDivWidth())] << std::endl;
    fout << "lgx2 = " << _rayGrid->getLastRayResults()[gx2Pos.first + (gx2Pos.second*_rayGrid->getNumDivWidth())] << std::endl;

    std::pair<int,int> gy1Pos = _rayGrid->transformRealPositionToGrid(_platform->getChassisBody()->GetPos() + ChVectord(aabbMax.x,0.0,0.0));
    double gy1 = _rayGrid->getRayResults()[gy1Pos.first + (gy1Pos.second*_rayGrid->getNumDivWidth())];
    std::pair<int,int> gy2Pos = _rayGrid->transformRealPositionToGrid(_platform->getChassisBody()->GetPos() + ChVectord(aabbMin.x,0.0,0.0));
    double gy2 = _rayGrid->getRayResults()[gy2Pos.first + (gy2Pos.second*_rayGrid->getNumDivWidth())];

    fout << "gy1 = " << gy1 << std::endl;
    fout << "gy2 = " << gy2 << std::endl;
    fout << "lgy1 = " << _rayGrid->getLastRayResults()[gy1Pos.first + (gy1Pos.second*_rayGrid->getNumDivWidth())] << std::endl;
    fout << "lgy2 = " << _rayGrid->getLastRayResults()[gy2Pos.first + (gy2Pos.second*_rayGrid->getNumDivWidth())] << std::endl;*/

    fout.close();
    _frameCount++;
}

PlatformPtr Experiment::getPlatform(){
    return _platform;
}

po::variables_map Experiment::getVariables(){
    return _vm;
}
