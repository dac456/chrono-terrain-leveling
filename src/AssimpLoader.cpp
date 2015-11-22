#include "AssimpLoader.h"

AssimpLoader::AssimpLoader(std::string path){
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
                _vertices.push_back(ChVector<double>(v.x, v.y, v.z));
                
                aiVector3t<double> n = mesh->mNormals[j];
                _normals.push_back(ChVector<double>(n.x, n.y, n.z));
            }
          
        }
    }
}

AssimpLoader::~AssimpLoader(){
    _vertices.clear();
    _normals.clear();
    _indices.clear();
}

std::shared_ptr<geometry::ChTriangleMeshConnected> AssimpLoader::toChronoTriMesh(){
    std::shared_ptr<geometry::ChTriangleMeshConnected> trimesh = std::make_shared<geometry::ChTriangleMeshConnected>();
    trimesh->m_vertices = _vertices; 
    trimesh->m_normals = _normals;
    trimesh->m_face_v_indices = _indices;
    
    return trimesh;
}