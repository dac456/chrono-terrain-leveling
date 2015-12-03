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
        _loadNode(doc.first_node());
    }
}

void UrdfLoader::_loadNode(rapidxml::xml_node<>* node){
    rapidxml::xml_node<>* current = node;
    
    while(current){
        if(current->name() == "robot"){
            current = current->first_node();
        }
        else if(current->name() == "link"){
            UrdfLink link;
            link.name = current->first_attribute("name")->value();
            
            _loadLink(current, &link);
            _links.push_back(link);
        }
        
        current = current->next_sibling();
    }
}

void UrdfLoader::_loadLink(rapidxml::xml_node<>* node, UrdfLink* link){
    rapidxml::xml_node<>* current = node->first_node();
    
    while(current){
        if(current->name() == "visual"){
            _loadVisual(current, link);
        }
        else if(current->name() == "collision"){
            
        }
        
        current = current->next_sibling();
    }
}

void UrdfLoader::_loadVisual(rapidxml::xml_node<>* node, UrdfLink* link){
    rapidxml::xml_node<>* current = node;
    
    UrdfVisual visual;
    if(current->first_attribute("name")){
        visual.name = current->first_attribute("name")->value();
    }
    
    current = current->first_node();
    while(current){
        if(current->name() == "geometry"){
            
        }
        
        current = current->next_sibling();
    }
    
    link->visual.push_back(visual);
}