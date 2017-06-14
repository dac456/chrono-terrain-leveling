#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"

#include "HeightMap.hpp"

HeightMap::HeightMap(std::string filename, double scale)
    : _heights(nullptr),
      _width(-1),
      _height(-1)
{
    int w, h, n;
    unsigned char* data = stbi_load(filename.c_str(), &w, &h, &n, 1);
    if(data){
        _imgWidth = w;
        _imgHeight = h;
        /*_heights = new float[x*y];
        _width = x;
        _height = y;

        for(int i=0; i<y; i++){
            for(int j=0; j<x; j++){
                _heights[j + (x*i)] = static_cast<float>(data[j + (x*i)]) / 255.0f;
            }
        }*/


        int h2 = h * scale;
        int w2 = w * scale;
        int x_ratio = static_cast<int>((w << 16) / w2) + 1;
        int y_ratio = static_cast<int>((h << 16) / h2) + 1;
        float* temp = new float[h2 * w2];

        for(int y = 0; y < h2; y++) {
            for(int x = 0; x < w2; x++) {
                int x2 = ((x * x_ratio) >> 16) ;
                int y2 = ((y * y_ratio) >> 16) ;
                temp[(y * w2) + x] = static_cast<float>(data[(y2 * w) + x2] / 255.0f) ;
            }
        }

        _heights = temp;
        _width = w2;
        _height = h2;
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

int HeightMap::getImageWidth() {
    return _imgWidth;
}

int HeightMap::getImageHeight() {
    return _imgHeight;
}
