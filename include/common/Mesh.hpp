#pragma once

#include "common/common.hpp"

/** Basic mesh class.
 * Stores an array of vertices and faces.
 */
class Mesh
{
public:
    Mesh(const std::vector<Vec3r>& vertices, const std::vector<Vec3i>& faces);

    /** Loads from a file with Assimp. */
    static Mesh loadFromFile(const std::string& filename);

    const std::vector<Vec3r>& vertices() const { return _vertices; }
    const std::vector<Vec3i>& faces() const { return _faces; }

    const Vec3r& vertex(int vertex_ind) const { return _vertices[vertex_ind]; }
    const Vec3i& face(int face_ind) const { return _faces[face_ind]; }

private:
    std::vector<Vec3r> _vertices;
    std::vector<Vec3i> _faces;

};