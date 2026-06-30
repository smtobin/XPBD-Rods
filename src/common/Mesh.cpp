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

    std::cout << "Loaded " << filename << " from file:\n  " << verts.size() << " vertices, " << faces.size() << " faces" << std::endl;

    Mesh mesh(verts, faces);
    return mesh;
}

Vec3r Mesh::massCenter() const
{
    Real total_volume = 0;
    Vec3r weighted_volume(0,0,0);
    for (const auto& f : _faces)
    {
        // each triangle in the mesh + origin forms a tetrahedron
        // v0=origin, v1=f[0], v2=f[1], v3=f[2]
        const Vec3r v0(0,0,0);
        const Vec3r v1 = vertex(f[0]);
        const Vec3r v2 = vertex(f[1]);
        const Vec3r v3 = vertex(f[2]);

        // tet basis matrix
        Mat3r A;
        A.col(0) = (v1 - v0);
        A.col(1) = (v2 - v0);
        A.col(2) = (v3 - v0);

        // find signed volume of tet
        const Real volume = A.determinant() / 6.0;

        // calculate the center of mass of this tetrahedron - just average of 4 vertices
        const Vec3r tet_cm = 0.25*(v0 + v1 + v2 + v3);
        // update overall center of mass using a weighted average
        // if (total_volume + volume > 0)
        //     center_of_mass = (center_of_mass*total_volume + tet_cm*volume) / (total_volume + volume);
        weighted_volume += tet_cm * volume;

        // update overall volume
        total_volume += volume;
    }

    return weighted_volume / total_volume;
}

void Mesh::moveDelta(const Vec3r& delta)
{
    for (auto& v : _vertices)
        v += delta;
}

void Mesh::applyRotation(const Mat3r& R)
{
    for (auto& v : _vertices)
        v = R*v;
}

void Mesh::resize(Real scale_x, Real scale_y, Real scale_z)
{
    for (auto& v : _vertices)
    {
        v[0] *= scale_x;
        v[1] *= scale_y;
        v[2] *= scale_z;
    }
}

void Mesh::resize(Real scale)
{
    Mesh::resize(scale, scale, scale);
}