#include "UrdfLoader.hpp"

#define URDFDEBUG(x) std::cout << "URDF Loader: " << x << std::endl

UrdfLoader::UrdfLoader(std::string file)
    : _file(file)
{
    if(fs::exists(file)) _load();
}

UrdfLoader::~UrdfLoader(){

}

std::vector<UrdfLink> UrdfLoader::getLinks(){
    return _links;
}

void UrdfLoader::_load(){
    std::stringstream buffer;

    std::ifstream fin(_file, std::ios::in);
    std::string s;
    while(std::getline(fin, s)) buffer << s << std::endl;
    fin.close();

    rapidxml::xml_document<> doc;
    doc.parse<0>(const_cast<char*>(buffer.str().c_str()));

    if(strcmp(doc.first_node()->name(), "robot") == 0){
        _loadRobot(doc.first_node()->first_node());
    }
}

void UrdfLoader::_loadRobot(rapidxml::xml_node<>* node){
    rapidxml::xml_node<>* current = node;

    while(current){
        URDFDEBUG(current->name());
        if(streq(current->name(), "link")){
            UrdfLink link;
            link.name = current->first_attribute("name")->value();

            _loadLink(current, &link);
            _links.push_back(link);
        }
        if(streq(current->name(), "material")){
            _loadMaterial(current);
        }

        current = current->next_sibling();
    }
}

void UrdfLoader::_loadLink(rapidxml::xml_node<>* node, UrdfLink* link){
    rapidxml::xml_node<>* current = node->first_node();

    while(current){
        if(streq(current->name(), "link")){
            UrdfLink child;
            child.name = current->first_attribute("name")->value();

            _loadLink(current, &child);
            link->links.push_back(child);
        }
        if(streq(current->name(), "visual")){
            _loadVisual(current, link);
        }
        if(streq(current->name(), "collision")){

        }

        current = current->next_sibling();
    }
}

void UrdfLoader::_loadMaterial(rapidxml::xml_node<>* node){
    UrdfMaterial mat;
    //TODO
    URDFDEBUG("material");
    _materials.push_back(mat);
}

void UrdfLoader::_loadVisual(rapidxml::xml_node<>* node, UrdfLink* link){
    rapidxml::xml_node<>* current = node;

    UrdfVisual visual;
    if(current->first_attribute("name")){
        visual.name = current->first_attribute("name")->value();
    }

    current = current->first_node();
    while(current){
        if(streq(current->name(), "origin")){
            URDFDEBUG("origin");
        }
        if(streq(current->name(), "geometry")){
            rapidxml::xml_node<>* geo = current->first_node();

            if(streq(geo->name(), "box")){
                URDFDEBUG("box");
                std::vector<std::string> vec = _split(geo->first_attribute("size")->value(), ' ');

                //TODO: these values aren't converted properly - check atof and also polymorphism stuff
                UrdfBox boxGeom;
                boxGeom.dim = ChVectord(atof(vec[0].c_str()), atof(vec[1].c_str()), atof(vec[2].c_str()));

                visual.geometry = boxGeom;
            }
        }

        current = current->next_sibling();
    }

    link->visuals.push_back(visual);
}

std::vector<std::string> UrdfLoader::_split(std::string str, const char delim){
    std::vector<std::string> ret;

    while(str.find_first_of(delim) != std::string::npos){
        std::string tok = str.substr(0, str.find_first_of(delim)+1);
        ret.push_back(tok);
        str = str.erase(0, str.find_first_of(delim)+1);
    }
    ret.push_back(str);

    return ret;
}
