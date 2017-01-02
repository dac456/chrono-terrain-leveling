#ifndef __HEIGHTMAP_HPP
#define __HEIGHTMAP_HPP

#include "CtlCommon.hpp"

class HeightMap{
private:
    float* _heights;
    int _width;
    int _height;

public:
    HeightMap(std::string filename, double scale = 1.0);
    ~HeightMap();

    float* getHeights();

    int getWidth();
    int getHeight();
};

#endif
