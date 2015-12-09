#ifndef __URDFLOADER_HPP
#define __URDFLOADER_HPP

#include "CtlCommon.hpp"

#include "RapidXML/rapidxml.hpp"

struct UrdfGeometry{
    std::string type;
};
typedef std::shared_ptr<UrdfGeometry> UrdfGeometryPtr;

struct UrdfBox : public UrdfGeometry{
    UrdfBox() : UrdfGeometry() { type = "box"; }
    ChVectord dim;
};
typedef std::shared_ptr<UrdfBox> UrdfBoxPtr;

struct UrdfMaterial{
    std::string name;
};
typedef std::shared_ptr<UrdfMaterial> UrdfMaterialPtr;

struct UrdfInertial{
    std::pair<ChVectord,ChVectord> origin;

    double mass;
    ChVectord inertiaXX;
};
typedef std::shared_ptr<UrdfInertial> UrdfInertialPtr;

struct UrdfVisual{
    std::string name;
    std::pair<ChVectord,ChVectord> origin;

    UrdfGeometryPtr geometry;
    UrdfMaterialPtr material;
};
typedef std::shared_ptr<UrdfVisual> UrdfVisualPtr;

struct UrdfCollision{
    std::string name;
    std::pair<ChVectord,ChVectord> origin;

    UrdfGeometryPtr geometry;
};
typedef std::shared_ptr<UrdfCollision> UrdfCollisionPtr;

struct UrdfLink{
    std::string name;

    std::vector<UrdfVisualPtr> visuals;
    std::vector<UrdfCollisionPtr> collisions;
    std::vector<UrdfInertialPtr> inertials;

    std::vector<std::shared_ptr<UrdfLink>> links;
};
typedef std::shared_ptr<UrdfLink> UrdfLinkPtr;

struct UrdfJoint{
    std::string name;
    std::pair<ChVectord,ChVectord> origin;

    std::string parent;
    std::string child;

    std::string type;
};
typedef std::shared_ptr<UrdfJoint> UrdfJointPtr;

class UrdfLoader{
private:
    std::string _file;

    std::vector<UrdfLinkPtr> _links; //TODO: use map instead?
    std::vector<UrdfJointPtr> _joints;
    std::vector<UrdfMaterialPtr> _materials;

public:
    UrdfLoader(std::string file);
    ~UrdfLoader();

    std::vector<UrdfLinkPtr> getLinks();
    UrdfLinkPtr getLink(std::string name);

    std::vector<UrdfJointPtr> getJoints();

private:
    void _load();
    void _loadRobot(rapidxml::xml_node<>* node);
    void _loadLink(rapidxml::xml_node<>* node, UrdfLinkPtr link);
    void _loadJoint(rapidxml::xml_node<>* node);
    void _loadMaterial(rapidxml::xml_node<>* node);

    void _loadVisual(rapidxml::xml_node<>* node, UrdfLinkPtr link);
    void _loadCollision(rapidxml::xml_node<>* node, UrdfLinkPtr link);
    void _loadInertial(rapidxml::xml_node<>* node, UrdfLinkPtr link);

    UrdfGeometryPtr _loadGeometry(rapidxml::xml_node<>* node);
    std::vector<std::string> _split(std::string str, const char delim);

};

#endif
