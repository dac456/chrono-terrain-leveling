#include "UrdfLoader.hpp"

#define URDFDEBUG(x) std::cout << "URDF Loader: " << x << std::endl

UrdfLoader::UrdfLoader(std::string file)
    : _file(file)
{
    if(fs::exists(file)) _load();
    else URDFDEBUG("file not found.");
}

UrdfLoader::~UrdfLoader(){
    _links.clear();
    _joints.clear();
}

std::vector<UrdfLinkPtr> UrdfLoader::getLinks(){
    return _links;
}

UrdfLinkPtr UrdfLoader::getLink(std::string name){
    for(auto link : _links){
        if(link->name == name){
            return link;
        }
    }

    return nullptr;
}

std::vector<UrdfJointPtr> UrdfLoader::getJoints(){
    return _joints;
}

void UrdfLoader::_load(){
    std::stringstream buffer;

    std::ifstream fin(_file, std::ios::in);
    buffer << fin.rdbuf();
    fin.close();

    rapidxml::xml_document<> doc;
    doc.parse<0>(const_cast<char*>(buffer.str().c_str()));

    if(streq(doc.first_node()->name(), "robot")){
        _loadRobot(doc.first_node()->first_node());
    }

    URDFDEBUG("Loaded " + _file);
}

void UrdfLoader::_loadRobot(rapidxml::xml_node<>* node){
    rapidxml::xml_node<>* current = node;

    while(current){
        if(streq(current->name(), "link")){
            UrdfLinkPtr link = std::make_shared<UrdfLink>();
            link->name = current->first_attribute("name")->value();

            _loadLink(current, link);
            _links.push_back(link);
        }
        if(streq(current->name(), "joint")){
            _loadJoint(current);
        }
        if(streq(current->name(), "material")){
            _loadMaterial(current);
        }

        current = current->next_sibling();
    }
}

void UrdfLoader::_loadLink(rapidxml::xml_node<>* node, UrdfLinkPtr link){
    rapidxml::xml_node<>* current = node->first_node();

    while(current){
        if(streq(current->name(), "link")){
            UrdfLinkPtr child = std::make_shared<UrdfLink>();
            child->name = current->first_attribute("name")->value();

            _loadLink(current, child);
            link->links.push_back(child);
        }
        if(streq(current->name(), "visual")){
            _loadVisual(current, link);
        }
        if(streq(current->name(), "collision")){
            _loadCollision(current, link);
        }
        if(streq(current->name(), "inertial")){
            _loadInertial(current, link);
        }

        current = current->next_sibling();
    }
}

void UrdfLoader::_loadJoint(rapidxml::xml_node<>* node){
    UrdfJointPtr joint = std::make_shared<UrdfJoint>();
    joint->name = node->first_attribute("name")->value();
    joint->type = node->first_attribute("type")->value();

    rapidxml::xml_node<>* current = node->first_node();

    while(current){
        if(streq(current->name(), "origin")){
            std::vector<std::string> xyz = _split(current->first_attribute("xyz")->value(), ' ');
            std::vector<std::string> rpy = _split(current->first_attribute("rpy")->value(), ' ');

            joint->origin.first = ChVectord(atof(xyz[0].c_str()), atof(xyz[1].c_str()), atof(xyz[2].c_str()));
            joint->origin.second = ChVectord(atof(rpy[0].c_str()), atof(rpy[1].c_str()), atof(rpy[2].c_str()));
        }
        if(streq(current->name(), "parent")){
            joint->parent = current->first_attribute("link")->value();
            if(joint->parent == ""){
                URDFDEBUG("link referenced by joint not found");
            }
        }
        if(streq(current->name(), "child")){
            joint->child = current->first_attribute("link")->value();
            if(joint->child == ""){
                URDFDEBUG("link referenced by joint not found");
            }
        }
        if(streq(current->name(), "limit")){
            if(joint->type == "revolute"){
                if(atof(current->first_attribute("velocity")->value()) != 0.0){
                    joint->type = "engine";
                }
            }
        }

        current = current->next_sibling();
    }

    _joints.push_back(joint);
}

void UrdfLoader::_loadMaterial(rapidxml::xml_node<>* node){
    UrdfMaterialPtr mat = std::make_shared<UrdfMaterial>();
    //TODO
    URDFDEBUG("material");
    _materials.push_back(mat);
}

void UrdfLoader::_loadVisual(rapidxml::xml_node<>* node, UrdfLinkPtr link){
    rapidxml::xml_node<>* current = node;

    UrdfVisualPtr visual = std::make_shared<UrdfVisual>();
    if(current->first_attribute("name")){
        visual->name = current->first_attribute("name")->value();
    }

    current = current->first_node();
    while(current){
        if(streq(current->name(), "origin")){
            std::vector<std::string> xyz = _split(current->first_attribute("xyz")->value(), ' ');
            std::vector<std::string> rpy = _split(current->first_attribute("rpy")->value(), ' ');

            visual->origin.first = ChVectord(atof(xyz[0].c_str()), atof(xyz[1].c_str()), atof(xyz[2].c_str()));
            visual->origin.second = ChVectord(atof(rpy[0].c_str()), atof(rpy[1].c_str()), atof(rpy[2].c_str()));
        }
        if(streq(current->name(), "geometry")){
            visual->geometry = _loadGeometry(current);
        }

        current = current->next_sibling();
    }

    link->visuals.push_back(visual);
}

void UrdfLoader::_loadCollision(rapidxml::xml_node<>* node, UrdfLinkPtr link){
    rapidxml::xml_node<>* current = node;

    UrdfCollisionPtr collision = std::make_shared<UrdfCollision>();
    if(current->first_attribute("name")){
        collision->name = current->first_attribute("name")->value();
    }

    current = current->first_node();
    while(current){
        if(streq(current->name(), "origin")){
            std::vector<std::string> xyz = _split(current->first_attribute("xyz")->value(), ' ');
            std::vector<std::string> rpy = _split(current->first_attribute("rpy")->value(), ' ');

            collision->origin.first = ChVectord(atof(xyz[0].c_str()), atof(xyz[1].c_str()), atof(xyz[2].c_str()));
            collision->origin.second = ChVectord(atof(rpy[0].c_str()), atof(rpy[1].c_str()), atof(rpy[2].c_str()));
        }
        if(streq(current->name(), "geometry")){
            collision->geometry = _loadGeometry(current);
        }

        current = current->next_sibling();
    }

    link->collisions.push_back(collision);
}

void UrdfLoader::_loadInertial(rapidxml::xml_node<>* node, UrdfLinkPtr link){
    rapidxml::xml_node<>* current = node->first_node();

    UrdfInertialPtr inertia = std::make_shared<UrdfInertial>();

    while(current){
        if(streq(current->name(), "origin")){
            std::vector<std::string> xyz = _split(current->first_attribute("xyz")->value(), ' ');
            std::vector<std::string> rpy = _split(current->first_attribute("rpy")->value(), ' ');

            inertia->origin.first = ChVectord(atof(xyz[0].c_str()), atof(xyz[1].c_str()), atof(xyz[2].c_str()));
            inertia->origin.second = ChVectord(atof(rpy[0].c_str()), atof(rpy[1].c_str()), atof(rpy[2].c_str()));
        }
        if(streq(current->name(), "mass")){
            inertia->mass = atof(current->first_attribute("value")->value());
        }
        if(streq(current->name(), "inertia")){
            double ixx = atof(current->first_attribute("ixx")->value());
            double iyy = atof(current->first_attribute("iyy")->value());
            double izz = atof(current->first_attribute("izz")->value());

            inertia->inertiaXX = ChVectord(ixx, iyy, izz);
        }

        current = current->next_sibling();
    }

    link->inertials.push_back(inertia);
}

UrdfGeometryPtr UrdfLoader::_loadGeometry(rapidxml::xml_node<>* node){
    rapidxml::xml_node<>* geo = node->first_node();
    UrdfGeometryPtr out = nullptr;

    if(streq(geo->name(), "box")){
        std::vector<std::string> vec = _split(geo->first_attribute("size")->value(), ' ');

        UrdfBoxPtr boxGeom = std::make_shared<UrdfBox>();
        boxGeom->dim = ChVectord(atof(vec[0].c_str()), atof(vec[1].c_str()), atof(vec[2].c_str()));

        out = boxGeom;
    }
    else if(streq(geo->name(), "cylinder")){
        double r = atof(geo->first_attribute("radius")->value());
        double l = atof(geo->first_attribute("length")->value());

        UrdfCylinderPtr cylGeom = std::make_shared<UrdfCylinder>();
        cylGeom->radius = r;
        cylGeom->length = l;

        out = cylGeom;
    }
    else if(streq(geo->name(), "mesh")){
        std::string file = geo->first_attribute("filename")->value();
        std::vector<std::string> vec = _split(geo->first_attribute("scale")->value(), ' ');

        UrdfMeshPtr meshGeom = std::make_shared<UrdfMesh>();
        meshGeom->file = GetChronoDataFile(std::string("urdf/") + file);
        meshGeom->scale = ChVectord(atof(vec[0].c_str()), atof(vec[1].c_str()), atof(vec[2].c_str()));

        out = meshGeom;
    }
    else{
        URDFDEBUG("unknown geometry type");
    }

    return out;
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
