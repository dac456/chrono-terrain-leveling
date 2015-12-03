#ifndef __URDFLOADER_HPP
#define __URDFLOADER_HPP

#include "CtlCommon.hpp"

#include "RapidXML/rapidxml.hpp"

struct UrdfVisual{
    std::string name;
    std::pair<ChVectord,ChVectord> origin;
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

public:
    UrdfLoader(std::string file);
    ~UrdfLoader();

private:
    void _load();
    void _loadNode(rapidxml::xml_node<>* node);
    void _loadLink(rapidxml::xml_node<>* node, UrdfLink* link);

    void _loadVisual(rapidxml::xml_node<>* node, UrdfLink* link);

};

#endif
