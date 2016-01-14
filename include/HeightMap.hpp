#ifndef __HEIGHTMAP_HPP
#define __HEIGHTMAP_HPP

#include "CtlCommon.hpp"

class HeightMap{
private:
    float* _heights;
    int _width;
    int _height;

public:
    HeightMap(std::string filename);
    ~HeightMap();

    float* getHeights();

    int getWidth();
    int getHeight();
};

#endif
