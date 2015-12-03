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
    doc.parse<0>(const_cast<char*>(buffer.c_str()));
    
    if(doc.first_node()->name() == "robot"){
        _loadNode(doc.first_node())
    }
}

void UrdfLoader::_loadNode(rapidxm::xml_node* node){
    rapidxml::xml_node* current = node;
    
    while(current){
        
    }
}