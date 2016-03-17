#include "RayGrid.hpp"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

#include <chrono/collision/ChCCollisionSystem.h>

RayGrid::RayGrid(ChSystem* system, ChVectord centre, double width, double length, size_t numDivWidth, size_t numDivLength)
    : _system(system)
    , _frameCount(0)
    , _centre(centre)
    , _width(width)
    , _length(length)
    , _numDivWidth(numDivWidth)
    , _numDivLength(numDivLength)
{
    _grid = new double[numDivLength*numDivWidth];
    _lastGrid = new double[numDivLength*numDivWidth];
    _gridOut = new unsigned char[numDivLength*numDivWidth];
    for(size_t i=0; i<(numDivLength*numDivWidth); i++){
        _grid[i] = 0.0;
        _lastGrid[i] = 0.0;
        _gridOut[i] = 0;
    }

    double widthStep = width / numDivWidth;
    double lengthStep = length / numDivLength;
    double halfWidth = width * 0.5;
    double halfLength = length * 0.5;

    for(double i=-halfLength + (lengthStep*0.5); i<halfLength; i+=lengthStep){
        for(double j=-halfWidth + (widthStep*0.5); j<halfWidth; j+=widthStep){
            ChVectord p(centre.x + (i + (lengthStep*0.5)), centre.y, centre.z + (j + (widthStep*0.5)));
            _origins.push_back(p);
        }
    }

    std::cout << _origins.size() << " " << (_numDivWidth*_numDivLength) << std::endl;
}

RayGrid::~RayGrid(){
    delete[] _gridOut;
    delete[] _lastGrid;
    delete[] _grid;
}

double RayGrid::getWidth(){
    return _width;
}

double RayGrid::getLength(){
    return _length;
}

size_t RayGrid::getNumDivWidth(){
    return _numDivWidth;
}

size_t RayGrid::getNumDivLength(){
    return _numDivLength;
}

std::pair<int,int> RayGrid::transformRealPositionToGrid(ChVectord p){
    std::pair<double,double> dpp = getDistancePerPixel();

    double halfWidth = _width * 0.5;
    double halfLength = _length * 0.5;

    int px = ((p.z+halfWidth)/_width)*_numDivWidth;
    int py = ((p.x+halfLength)/_length)*_numDivLength;

    return std::make_pair(px, py);
}

std::pair<double,double> RayGrid::getDistancePerPixel(){
    double dx = _width / _numDivWidth;
    double dy = _length / _numDivLength;

    return std::make_pair(dx, dy);
}

void RayGrid::castRays(){
    for(size_t i=0; i<_numDivLength; i++){
        for(size_t j=0; j<_numDivWidth; j++){
            size_t idx = j + (i*_numDivWidth);
            _lastGrid[idx] = _grid[idx];

            collision::ChCollisionSystem::ChRayhitResult result;
            _system->GetCollisionSystem()->RayHit(_origins[idx], ChVectord(_origins[idx].x, 0.0, _origins[idx].z), result);
            if(result.hit){
                _grid[idx] = 1.0 - result.dist_factor;
                _gridOut[idx] = static_cast<unsigned char>(255.0*_grid[idx]);
            }
            else{
                _grid[idx] = 0.0;
                _gridOut[idx] = 0;
            }
        }
    }

    std::stringstream ssf;
    ssf << "./raygrid/frame" << _frameCount << ".tga";
    stbi_write_tga(ssf.str().c_str(), _numDivWidth, _numDivLength, 1, &_gridOut[0]);

    _frameCount++;

}

std::vector<ChVectord> RayGrid::getRayOrigins(){
    return _origins;
}

double* RayGrid::getRayResults(){
    return _grid;
}

double* RayGrid::getLastRayResults(){
    return _lastGrid;
}
