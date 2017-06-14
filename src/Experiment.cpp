#include <chrono/core/ChFileutils.h>

//#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

#include "Experiment.hpp"
#include "Config.hpp"

#include "UrdfLoader.hpp"
#include "Assembly.hpp"
#include "StaticMesh.hpp"
#include "TrackedVehicle.hpp"
#include "ParticleSystem.hpp"
#include "AlgorithmBasic.hpp"
#include "AlgorithmRandom.hpp"
#include "HeightMap.hpp"
#include "RayGrid.hpp"

Experiment::Experiment(ChSystem* system, std::string expConfigFile)
    : _system(system)
    , _frameCount(0)
    , _linearVel(0.0)
    , _angularVel(0.0)
    , _lastLeftSpeed(0.0)
    , _lastRightSpeed(0.0)
    , _warped(false)
    , _enteringX(false)
    , _enteringZ(false)
    , _useParticles(false)
    , _useTracks(false)
{
    Config cfg(expConfigFile);
    _vm = cfg.getVariables();

    fs::create_directories(_vm["output_directory_prefix"].as<std::string>() + "framedata/");
    std::cout << "Chrono data path: " << fs::canonical(_vm["chrono_data_path"].as<std::string>()).string() << std::endl;
    SetChronoDataPath(fs::canonical(_vm["chrono_data_path"].as<std::string>()).string() + '/');
    vehicle::SetDataPath(fs::canonical(_vm["chrono_data_path"].as<std::string>()).string() + "/vehicle/");

    _name = _vm["experiment.name"].as<std::string>();
    _useTracks = _vm["vehicle.tracks"].as<bool>();

    double particleSize = _vm["map.particle_radius"].as<double>();

    if(_vm["map.model"].as<std::string>() == "particle") {
        _useParticles = true;
    } else if(_vm["map.model"].as<std::string>() == "scm") {
        _useParticles = false;
    } else {
        std::cout << "Unknown terrain model '" << _vm["map.model"].as<std::string>() << "'" << std::endl;
    }

#if 1
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

    _lastX = startPos.x;
    _lastY = startPos.z;
    _lastTheta = 0.0;


    UrdfLoader urdf(GetChronoDataFile("urdf/" + _vm["vehicle.model"].as<std::string>() + ".urdf"));
    AssemblyPtr assembly = std::make_shared<Assembly>(urdf, startPos, startOrient, static_cast<ChSystem*>(system));

    TrackedVehiclePtr vehicle = std::make_shared<TrackedVehicle>("dagu001", "shoe_view.obj", "shoe_collision.obj", assembly, 0.5, _useTracks);

    if(!_vm.count("experiment.algorithm")){
        _platform = std::make_shared<Platform>(vehicle, nullptr, nullptr);
        _linearVel = _vm["experiment.linear"].as<double>();
        _angularVel = _vm["experiment.angular"].as<double>();
        _platform->setDesiredLinearVelocity(_linearVel);
        _platform->setDesiredAngularVelocity(_angularVel);
    }
    else{
        if(_vm["experiment.algorithm"].as<std::string>() == "basic"){
            _platform = std::make_shared<AlgorithmBasic>(vehicle, nullptr, nullptr);
        }
        else if(_vm["experiment.algorithm"].as<std::string>() == "random"){
            _platform = std::make_shared<AlgorithmRandom>(vehicle, nullptr, nullptr);
        }
        else{
            std::cout << "Error: Unknown algorithm selected." << std::endl;
        }
    }
#endif

    _hm = std::make_shared<HeightMap>(GetChronoDataFile(_vm["map.filename"].as<std::string>()), _vm["map.xy_scale"].as<double>());
    if(_useParticles) {
        ParticleSystemPtr particles = std::make_shared<ParticleSystem>(static_cast<ChSystem *>(system), _hm,
                                                                       _vm["map.scale"].as<double>(), _vm["map.particle_density"].as<double>(),
                                                                       particleSize, true, false);
    } else {
        _terrain = std::make_shared<vehicle::DeformableTerrain>(system);

       /* _terrain->SetSoilParametersSCM(2e7,   // Bekker Kphi
                                     0,     // Bekker Kc
                                     1.1,   // Bekker n exponent
                                     0,     // Mohr cohesive limit (Pa)
                                     20,    // Mohr friction limit (degrees)
                                     0.01,  // Janosi shear coefficient (m)
                                     2e8,   // Elastic stiffness (Pa/m), before plastic yeld
                                     3e4    // Damping (Pa s/m), proportional to negative vertical speed (optional)
                                     );

        _terrain->SetAutomaticRefinement(true);
        _terrain->SetAutomaticRefinementResolution(0.04);*/
        //_terrain->SetPlane(ChCoordsys<>(ChVector<>(0,0,0), Q_from_AngX(CH_C_PI)));
        _terrain->SetPlotType(vehicle::DeformableTerrain::PLOT_LEVEL, 0.0, 5.0);
        //_terrain->SetBulldozingFlow(true);
        _terrain->Initialize(GetChronoDataFile(_vm["map.filename"].as<std::string>()), "terrain1", _hm->getWidth(), _hm->getHeight(), 0, 5);
        std::vector<ChVectord> verts = _terrain->GetMesh()->GetMesh().getCoordsVertices();

        ChVectord min(10e10, 10e10, 10e10);
        ChVectord max(-10e10, -10e10, -10e10);

        for(auto v : verts) {
            if(v.x < min.x) min.x = v.x;
            if(v.x > max.x) max.x = v.x;
            if(v.y < min.y) min.y = v.y;
            if(v.y > max.y) max.y = v.y;
            if(v.z < min.z) min.z = v.z;
            if(v.z > max.z) max.z = v.z;
        }
        _terrainMin = min;
        _terrainMax = max;

        std::cout << "Terrain min: " << min.x << " " << min.y << " " << min.z << std::endl;
        std::cout << "Terrain max: " << max.x << " " << max.y << " " << max.z << std::endl;
    }

    double rgRes = _vm["raygrid.resolution"].as<double>();
    _rayGrid = std::make_shared<RayGrid>(system, _vm, ChVectord(0.0, _vm["map.scale"].as<double>(), 0.0), _hm->getHeight()*particleSize*2.0, _hm->getWidth()*particleSize*2.0, _hm->getHeight()*rgRes, _hm->getWidth()*rgRes);
}

Experiment::~Experiment(){}

void Experiment::step(double dt, bool createPlatform) {
    if (createPlatform) {
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

        _lastX = startPos.x;
        _lastY = startPos.z;
        _lastTheta = 0.0;


        UrdfLoader urdf(GetChronoDataFile("urdf/" + _vm["vehicle.model"].as<std::string>() + ".urdf"));
        AssemblyPtr assembly = std::make_shared<Assembly>(urdf, startPos, startOrient,
                                                          static_cast<ChSystem *>(_system));

        TrackedVehiclePtr vehicle = std::make_shared<TrackedVehicle>("dagu001", "shoe_view.obj", "shoe_collision.obj",
                                                                     assembly, 0.5, _useTracks);
        //std::shared_ptr<vehicle::m113::M113_Vehicle> vehicle = std::make_shared<vehicle::m113::M113_Vehicle>(_system, vehicle::GetDataFile(std::string("M113/vehicle/M113_Vehicle.json")));
        /*std::shared_ptr<vehicle::m113::M113_Vehicle> vehicle = std::make_shared<vehicle::m113::M113_Vehicle>(false, vehicle::TrackShoeType::SINGLE_PIN, _system);
        vehicle->Initialize(ChCoordsys<>(startPos, startOrient));

        // Set visualization type for vehicle components.
        vehicle->SetChassisVisualizationType(vehicle::VisualizationType::PRIMITIVES);
        vehicle->SetSprocketVisualizationType(vehicle::VisualizationType::PRIMITIVES);
        vehicle->SetIdlerVisualizationType(vehicle::VisualizationType::PRIMITIVES);
        vehicle->SetRoadWheelAssemblyVisualizationType(vehicle::VisualizationType::PRIMITIVES);
        vehicle->SetTrackShoeVisualizationType(vehicle::VisualizationType::PRIMITIVES);

        // Solver settings.
        vehicle->GetSystem()->SetMaxItersSolverSpeed(50);
        vehicle->GetSystem()->SetMaxItersSolverStab(50);

        // Create and initialize the powertrain system
        //std::shared_ptr<vehicle::m113::M113_SimplePowertrain> powertrain = std::make_shared<vehicle::m113::M113_SimplePowertrain>(vehicle::GetDataFile("M113/powertrain/M113_SimplePowertrain.json"));
        std::shared_ptr<vehicle::m113::M113_SimplePowertrain> powertrain = std::make_shared<vehicle::m113::M113_SimplePowertrain>();
        powertrain->Initialize(vehicle->GetChassisBody(), vehicle->GetDriveshaft());*/

        if (!_vm.count("experiment.algorithm")) {
            _platform = std::make_shared<Platform>(vehicle, nullptr/*powertrain*/, _terrain);
            _linearVel = _vm["experiment.linear"].as<double>();
            _angularVel = _vm["experiment.angular"].as<double>();
            _platform->setDesiredLinearVelocity(_linearVel);
            _platform->setDesiredAngularVelocity(_angularVel);
        } else {
            if (_vm["experiment.algorithm"].as<std::string>() == "basic") {
                _platform = std::make_shared<AlgorithmBasic>(vehicle, nullptr/*powertrain*/, _terrain);
            } else if (_vm["experiment.algorithm"].as<std::string>() == "random") {
                _platform = std::make_shared<AlgorithmRandom>(vehicle, nullptr/*powertrain*/, _terrain);
            } else {
                std::cout << "Error: Unknown algorithm selected." << std::endl;
            }
        }
    }


    (_vm.count("experiment.algorithm") > 0) ? _platform->step(dt) : _platform->move(dt);
    if (_useParticles) {
        //_rayGrid->castRays();
    } else {
        _terrain->Synchronize(_system->GetChTime());
        _terrain->Advance(dt);
    }

    writeFrame();
    if(_warped) _warped = false;

    ChVectord aabbMin, aabbMax;
    _platform->getChassisBody()->GetTotalAABB(aabbMin, aabbMax);

    double vehicleLength = aabbMax.x - aabbMin.x;
    double vehicleWidth = aabbMax.z - aabbMin.z;
    //double vehicleLength = 2.0;
    double mapLength, mapWidth;
    if(_useParticles) {
        mapLength = _rayGrid->getLength() * 0.5;
        mapWidth = _rayGrid->getWidth() * 0.5;
    } else {
        mapLength = _hm->getHeight() * 0.5;
        mapWidth = _hm->getWidth() * 0.5;
    }

    ChVectord pos = _platform->getChassisBody()->GetPos();
    std::cout << pos.x << " " << mapLength << " " << vehicleLength << std::endl;
    if(pos.x > (mapLength-vehicleLength) && pos.x < mapLength && !_enteringX){
        _platform->warpToRelativePosition(ChVectord(-pos.x*2.0 + 0.5, 1.0, 0));
        _enteringX = true;
        _warped = true;
    }
    else if(pos.x < (vehicleLength-mapLength) && pos.x > -mapLength && !_enteringX){
        _platform->warpToRelativePosition(ChVectord(-pos.x*2.0 - 0.5, 1.0, 0));
        _enteringX = true;
        _warped = true;
    }

    if(_enteringX && !(pos.x > (mapLength-vehicleLength) && pos.x < mapLength) && !(pos.x < (vehicleLength-mapLength) && pos.x > -mapLength)){
        _enteringX = false;
    }

    if(pos.z > (mapWidth-(vehicleLength)) && pos.z < mapWidth && !_enteringZ){
        _platform->warpToRelativePosition(ChVectord(0, 1.0, -pos.z*2.0 + 0.5));
        _platform->setDesiredAngularVelocity(-_platform->getDesiredAngularVelocity());
        _enteringZ = true;
        _warped = true;
    }
    else if(pos.z < ((vehicleLength)-mapWidth) && pos.z > -mapWidth && !_enteringZ){
        _platform->warpToRelativePosition(ChVectord(0, 1.0, -pos.z*2.0 - 0.5));
        _platform->setDesiredAngularVelocity(-_platform->getDesiredAngularVelocity());
        _enteringZ = true;
        _warped = true;
    }

    if(_enteringZ && !(pos.z > (mapWidth-vehicleLength) && pos.z < mapWidth) && !(pos.z < (vehicleLength-mapWidth) && pos.z > -mapWidth)){
        _enteringZ = false;
    }
}

void Experiment::writeFrame(){
    std::stringstream ssf;
    ssf << _vm["output_directory_prefix"].as<std::string>() << "framedata/frame" << _frameCount << ".dat";

    std::ofstream fout(ssf.str());

    fout << "dppx = " << _rayGrid->getDistancePerPixel().first << std::endl;
    fout << "dppy = " << _rayGrid->getDistancePerPixel().second << std::endl;

    fout << "rw = " << _rayGrid->getWidth() << std::endl;
    fout << "rl = " << _rayGrid->getLength() << std::endl;

    (_warped == true) ? fout << "warped = Yes" << std::endl : fout << "warped = No" << std::endl;

    fout << "vxr = " << _platform->getChassisBody()->GetPos().x << std::endl;
    fout << "vyr = " << _platform->getChassisBody()->GetPos().z << std::endl;
    fout << "vdxr = " << _platform->getChassisBody()->GetPos().x - _lastX << std::endl;
    fout << "vdyr = " << _platform->getChassisBody()->GetPos().z - _lastY << std::endl;
    if(_useParticles) {
        std::pair<int,int> transformedPos = _rayGrid->transformRealPositionToGrid(_platform->getChassisBody()->GetPos());
        fout << "vx = " << transformedPos.first << std::endl;
        fout << "vy = "  << transformedPos.second << std::endl;
    } else {
        ChVectord pos = _platform->getChassisBody()->GetPos();
        double xs = (pos.x + _terrainMax.x) / (_terrainMax.x - _terrainMin.x);
        double zs = (pos.z + _terrainMax.z) / (_terrainMax.z - _terrainMin.z);
        fout << "vx = " << std::round(xs * _hm->getImageWidth()) << std::endl;
        fout << "vy = " << std::round(zs * _hm->getImageHeight()) << std::endl;
    }
    fout << "vtheta = " << _platform->getAccelYaw() << std::endl;
    fout << "vdtheta = " << _platform->getAccelYaw() - _lastTheta << std::endl;
    fout << "vpitch = " << _platform->getAccelPitch() << std::endl;
    fout << "vroll = " << _platform->getAccelRoll() << std::endl;
    _lastX = _platform->getChassisBody()->GetPos().x;
    _lastY = _platform->getChassisBody()->GetPos().z;
    _lastTheta = _platform->getAccelYaw();

    fout << "vdl = " << _platform->getDesiredLinearVelocity() << std::endl;
    fout << "vda = " << _platform->getDesiredAngularVelocity() << std::endl;

    double leftSpeed, rightSpeed;
    _platform->getVehicle()->getSpeeds(leftSpeed, rightSpeed);
    fout << "vleft = " << leftSpeed << std::endl;
    fout << "vright = " << rightSpeed << std::endl;
    fout << "vleftlast = " << _lastLeftSpeed << std::endl;
    fout << "vrightlast = " << _lastRightSpeed << std::endl;
    _lastLeftSpeed = leftSpeed;
    _lastRightSpeed = rightSpeed;

    /*ChVectord aabbMin, aabbMax;
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
    fout << "gy2y = " << gy2.second << std::endl;*/

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

    // Write heightmap
    if(!_useParticles) {
        size_t halfWidth = _hm->getWidth()*0.5;
        size_t halfHeight = _hm->getHeight()*0.5;

        std::vector<ChVectord> verts = _terrain->GetMesh()->GetMesh().getCoordsVertices();
        std::vector<uint8_t> pixels;
        pixels.resize(verts.size());
        size_t side = static_cast<size_t>(sqrt(verts.size()));

        uint8_t* img = new uint8_t[side*side];

        for(int i = 0; i < side; i++) {
            for(int j = 0; j < side; j++) {
                size_t idx = i + (j*side);
                double h = verts[idx].y/5.0;
                img[idx] = static_cast<uint8_t>(h*255.0);
            }
        }

        std::stringstream ssFrame;
        ssFrame << _vm["output_directory_prefix"].as<std::string>() << "raygrid/frame" << _frameCount << ".tga";
        stbi_write_tga(ssFrame.str().c_str(), side, side, 1, &img[0]);

        delete img;
    }

    _frameCount++;
}

PlatformPtr Experiment::getPlatform(){
    return _platform;
}

po::variables_map Experiment::getVariables(){
    return _vm;
}
