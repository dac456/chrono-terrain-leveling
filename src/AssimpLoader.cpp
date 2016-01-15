#include "AssimpLoader.hpp"

AssimpLoader::AssimpLoader(std::string path, ChVectord scale){
    Assimp::Importer imp;
    _scene = imp.ReadFile(path, aiProcess_CalcTangentSpace | aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType);

    if(!_scene){
        std::cout << imp.GetErrorString() << std::endl;
    }
    else{
        for(size_t i=0; i<_scene->mNumMeshes; i++){
            aiMesh* mesh = _scene->mMeshes[i];

            //Load triangle indices
            for(size_t j=0; j<mesh->mNumFaces; j++){
                aiFace face = mesh->mFaces[j];

                std::vector<int> faceIndices;
                for(size_t k=0; k<face.mNumIndices; k++){
                    int idx = face.mIndices[k];
                    faceIndices.push_back(idx);
                }

                //ChTriangleMeshConnected stores indices per triangle
                //Assume each face has only three vertices...
                _indices.push_back(ChVector<int>(faceIndices[0], faceIndices[1], faceIndices[2]));
            }

            //Load mesh vertices and normals
            for(size_t j=0; j<mesh->mNumVertices; j++){
                aiVector3t<double> v = mesh->mVertices[j];
                _vertices.push_back(ChVector<double>(v.x, v.y, v.z) * scale);

                aiVector3t<double> n = mesh->mNormals[j];
                _normals.push_back(ChVector<double>(n.x, n.y, n.z) * scale);
            }

            //Load mesh bones
            for(size_t j=0; j<mesh->mNumBones; j++){
                aiBone* bone = mesh->mBones[j];
                std::cout << "BONE: " << bone->mName.C_Str() << std::endl;
            }

        }
    }
}

AssimpLoader::~AssimpLoader(){
    _vertices.clear();
    _normals.clear();
    _indices.clear();
}

ChVectord AssimpLoader::getMeshDimensions(){
    ChVectord min(0,0,0);
    ChVectord max(0,0,0);

    for(auto& v : _vertices){
        if(v.x < min.x) min.x = v.x;
        if(v.y < min.y) min.y = v.y;
        if(v.z < min.z) min.z = v.z;

        if(v.x > max.x) max.x = v.x;
        if(v.y > max.y) max.y = v.y;
        if(v.z > max.z) max.z = v.z;
    }

    return ChVectord(max.x-min.x, max.y-min.y, max.z-min.z);
}

ChVectord AssimpLoader::getMeshCentre(){
    ChVectord min(0,0,0);
    ChVectord max(0,0,0);

    for(auto& v : _vertices){
        if(v.x < min.x) min.x = v.x;
        if(v.y < min.y) min.y = v.y;
        if(v.z < min.z) min.z = v.z;

        if(v.x > max.x) max.x = v.x;
        if(v.y > max.y) max.y = v.y;
        if(v.z > max.z) max.z = v.z;
    }

    return ChVectord((max.x+min.x)/2.0, (max.y+min.y)/2.0, (max.z+min.z)/2.0);
}

SHPTR<geometry::ChTriangleMeshConnected> AssimpLoader::toChronoTriMesh(){
    SHPTR<geometry::ChTriangleMeshConnected> trimesh = MKSHR<geometry::ChTriangleMeshConnected>();
    trimesh->m_vertices = _vertices;
    trimesh->m_normals = _normals;
    trimesh->m_face_v_indices = _indices;

    return trimesh;
}
