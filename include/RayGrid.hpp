#ifndef __RAYGRID_HPP
#define __RAYGRID_HPP

#include "CtlCommon.hpp"

class RayGrid{
private:
    ChSystem* _system;
    size_t _frameCount;

    ChVectord _centre;
    double _width;
    double _length;
    size_t _numDivWidth;
    size_t _numDivLength;

    //double** _grid;
    double* _grid;
    double* _lastGrid;
    unsigned char* _gridOut;
    std::vector<ChVectord> _origins;

public:
    RayGrid(ChSystem* system, ChVectord centre, double width, double length, size_t numDivWidth, size_t numDivLength);
    ~RayGrid();

    double getWidth();
    double getLength();
    size_t getNumDivWidth();
    size_t getNumDivLength();

    void castRays();
    std::pair<int, int> transformRealPositionToGrid(ChVectord p);

    std::pair<double,double> getDistancePerPixel();
    std::vector<ChVectord> getRayOrigins();

    double* getRayResults();
    double* getLastRayResults();
};

#endif
