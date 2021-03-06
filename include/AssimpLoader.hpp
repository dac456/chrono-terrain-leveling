#ifndef __ASSIMPLOADER_HPP
#define __ASSIMPLOADER_HPP

#include "CtlCommon.hpp"

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

class AssimpLoader{
private:
    const aiScene* _scene;

    std::vector<ChVector<double>> _vertices;
    std::vector<ChVector<double>> _normals;
    std::vector<ChVector<int>> _indices;

    std::map<std::string, ChMat44d> _bones; //(name, offsetmat) pair

public:
    AssimpLoader(std::string path, ChVectord scale = ChVectord(1,1,1));
    ~AssimpLoader();

    ChVectord getMeshDimensions();
    ChVectord getMeshCentre();

    std::shared_ptr<geometry::ChTriangleMeshConnected> toChronoTriMesh();

};

#endif
