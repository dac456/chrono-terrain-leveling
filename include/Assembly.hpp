#ifndef __ASSEMBLY_HPP
#define __ASSEMBLY_HPP

#include "CtlCommon.hpp"

class Assembly{
private:
    ChSystem* _system;
    std::vector<ChBodyPtr> _bodies;

public:
    Assembly(ChSystem* system);
    Assembly(UrdfLoader urdfLoader, ChSystem* system);
    ~Assembly();

};

#endif
