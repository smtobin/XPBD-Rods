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

    /** Computes the current center of mass for the mesh.
     * Uses the same algorithm as massProperties(), but without calculating the moment of inertia.
     */
    Vec3r massCenter() const;

    /** Moves each vertex in the mesh by the same amount. */
    void moveDelta(const Vec3r& delta);

    /** Applies a rotation to each vertex (specified as a rotation matrix) */
    void applyRotation(const Mat3r& rot_mat);

    /** Resizes the mesh (multiplies everything by the scale factors) */
    void resize(Real scale_x, Real scale_y, Real scale_z);
    void resize(Real scale);

private:
    std::vector<Vec3r> _vertices;
    std::vector<Vec3i> _faces;

};