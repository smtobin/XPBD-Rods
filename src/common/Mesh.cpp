#include "common/Mesh.hpp"

#include <assimp/Importer.hpp>
#include <assimp/Exporter.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

Mesh::Mesh(const std::vector<Vec3r>& vertices, const std::vector<Vec3i>& faces)
    : _vertices(vertices), _faces(faces)
{

}

Mesh Mesh::loadFromFile(const std::string& filename)
{
    Assimp::Importer importer;
    importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS, 
        aiComponent_NORMALS | 
        aiComponent_TANGENTS_AND_BITANGENTS |
        aiComponent_COLORS |
        aiComponent_TEXCOORDS
    );

    // And have it read the given file with some example postprocessing
    // Usually - if speed is not the most important aspect for you - you'll
    // probably to request more postprocessing than we do in this example.
    const aiScene* scene = importer.ReadFile( filename,
        aiProcess_Triangulate            |
        aiProcess_JoinIdenticalVertices  |
        aiProcess_RemoveComponent        |
        aiProcess_SortByPType);

    // If the import failed, report it
    if (scene == nullptr)
    {
        std::cerr << "\tAssimp::Importer could not open " << filename << std::endl;
        std::cerr << "\tEnsure that the file is in a format that assimp can handle." << std::endl;
        assert(0);
    }

    const aiMesh* ai_mesh = scene->mMeshes[0];

    // Extract vertices
    std::vector<Vec3r> verts(ai_mesh->mNumVertices);
    for (unsigned i = 0; i < ai_mesh->mNumVertices; i++)
    {
        verts[i][0] = ai_mesh->mVertices[i].x;
        verts[i][1] = ai_mesh->mVertices[i].y;
        verts[i][2] = ai_mesh->mVertices[i].z;
    }

    // Extract faces
    std::vector<Vec3i> faces(ai_mesh->mNumFaces);
    for (unsigned i = 0; i < ai_mesh->mNumFaces; i++)
    {
        faces[i][0] = ai_mesh->mFaces[i].mIndices[0];
        faces[i][1] = ai_mesh->mFaces[i].mIndices[1];
        faces[i][2] = ai_mesh->mFaces[i].mIndices[2];
    }

    Mesh mesh(verts, faces);
    return mesh;
}