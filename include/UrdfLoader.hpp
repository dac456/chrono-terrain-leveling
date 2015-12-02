#ifndef __URDFLOADER_HPP
#define __URDFLOADER_HPP

#include "CtlCommon.hpp"

#include "RapidXML/rapidxml.hpp"

class UrdfLoader{
private:
    std::string _file;

public:
    UrdfLoader(std::string file);
    ~UrdfLoader();
    
private:
    void _load();

};

#endif