#ifndef __URDFLOADER_HPP
#define __URDFLOADER_HPP

#include "CtlCommon.hpp"

#include "RapidXML/rapidxml.hpp"

struct UrdfGeometry{
    std::string type;
};

struct UrdfBox : public UrdfGeometry{
    UrdfBox(){ type = "box"; }
    ChVectord dim;
};

struct UrdfMaterial{
    std::string name;
};

struct UrdfVisual{
    std::string name;
    std::pair<ChVectord,ChVectord> origin;

    UrdfGeometry geometry;
    UrdfMaterial material;
};

struct UrdfLink{
    std::string name;
    std::vector<UrdfVisual> visuals;
    std::vector<UrdfLink> links;
};

class UrdfLoader{
private:
    std::string _file;

    std::vector<UrdfLink> _links;
    std::vector<UrdfMaterial> _materials;

public:
    UrdfLoader(std::string file);
    ~UrdfLoader();

private:
    void _load();
    void _loadRobot(rapidxml::xml_node<>* node);
    void _loadLink(rapidxml::xml_node<>* node, UrdfLink* link);
    void _loadMaterial(rapidxml::xml_node<>* node);

    void _loadVisual(rapidxml::xml_node<>* node, UrdfLink* link);

    std::vector<std::string> _split(std::string str, const char delim);

};

#endif
