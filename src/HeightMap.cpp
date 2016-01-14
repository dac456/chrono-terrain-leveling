#include "HeightMap.hpp"

#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"

HeightMap::HeightMap(std::string filename)
    : _heights(nullptr),
      _width(-1),
      _height(-1)
{
    int x, y, n;
    unsigned char* data = stbi_load(filename.c_str(), &x, &y, &n, 1);
    if(data){
        _heights = new float[x*y];
        _width = x;
        _height = y;

        for(int i=0; i<y; i++){
            for(int j=0; j<x; j++){
                _heights[j + (x*i)] = static_cast<float>(data[j + (x*i)]) / 255.0f;
            }
        }
    }
}

HeightMap::~HeightMap(){
    if(_heights){
        delete[] _heights;
    }
}

float* HeightMap::getHeights(){
    return _heights;
}

int HeightMap::getWidth(){
    return _width;
}

int HeightMap::getHeight(){
    return _height;
}
