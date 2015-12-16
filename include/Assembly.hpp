#ifndef __ASSEMBLY_HPP
#define __ASSEMBLY_HPP

#include "CtlCommon.hpp"

class Assembly{
private:
    ChSystem* _system;
    std::vector<ChBodyPtr> _bodies;
    std::vector<ChLinkPtr> _links;

public:
    Assembly(ChSystem* system);
    Assembly(UrdfLoader urdfLoader, ChVectord position, ChSystem* system);
    ~Assembly();

    ChSystem* getSystem();
    std::vector<ChBodyPtr> getBodies();

private:
    ChVectord _toChronoCoords(ChVectord urdfCoords);
    ChQuatd _toChronoOrientation(ChVectord urdfOrientation);

};

#endif
