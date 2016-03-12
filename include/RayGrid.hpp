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
    unsigned char* _gridOut;
    std::vector<ChVectord> _origins;

public:
    RayGrid(ChSystem* system, ChVectord centre, double width, double length, size_t numDivWidth, size_t numDivLength);
    ~RayGrid();

    void castRays();

    std::vector<ChVectord> getRayOrigins();
};

#endif
