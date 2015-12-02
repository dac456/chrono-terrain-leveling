#include "UrdfLoader.hpp"

UrdfLoader::UrdfLoader(std::string file)
    : _file(file)
{
    //TODO:check if file exists
    _load();
    
}

UrdfLoader::~UrdfLoader(){
    
}

void UrdfLoader::_load(){
    std::string buffer;
    
    std::ifstream fin(_file, std::ios::in);
    fin >> buffer;
    fin.close();
    
    rapidxml::xml_document<> doc;
    doc.parse<0>(buffer.c_str());
}