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

private:
    std::vector<Vec3r> _vertices;
    std::vector<Vec3i> _faces;

};