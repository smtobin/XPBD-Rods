#include "collision/sdf/MeshSDF.hpp"
#include "common/Mesh.hpp"
#include "common/math.hpp"

#include <map>
#include <iostream>
#include <iomanip>
#include <fstream>

// for fast file parsing
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <errno.h>

namespace Collision
{

MeshSDF::MeshSDF(const SimObject::OrientedParticle* particle, const std::vector<Vec3r>& verts, const std::vector<Vec3i>& tris, int grid_size, int padding, bool with_gradient)
    : SDF(),
    _particle(particle),
    _N(grid_size), _with_gradient(with_gradient)
{
    _makeSDF(verts, tris, padding, with_gradient);
}

MeshSDF::MeshSDF(const SimObject::OrientedParticle* particle, const std::string& filename)
    : _particle(particle),
      _N(0), _cell_size(),
     _grid_bbox_min(Vec3r::Zero()), _grid_bbox_max(Vec3r::Zero()),
     _mesh_bbox_min(Vec3r::Zero()), _mesh_bbox_max(Vec3r::Zero()),
     _with_gradient(false)
{
    _loadSDFFromFile(filename);
}

Real MeshSDF::evaluate(const Vec3r& p) const
{
    // convert to local frame
    Vec3r p_local = _particle->orientation.transpose() * (p - _particle->position);

    const Vec3r& ijk = _gridIJKFromPoint(p_local);
    
    const int i0 = std::floor(ijk[0]);    const int i1 = i0+1;
    const int j0 = std::floor(ijk[1]);    const int j1 = j0+1;
    const int k0 = std::floor(ijk[2]);    const int k1 = k0+1;

    // the decimal part of the (i,j,k) coordinates - becomes the interpolation parameters
    const Real id = ijk[0] - i0;
    const Real jd = ijk[1] - j0;
    const Real kd = ijk[2] - k0;

    // check to make sure we are inside the grid boundaries
    if (i0 >= 0 && i0 < _N-1 && j0 >= 0 && j0 < _N-1 && k0 >= 0 && k0 < _N-1)
    {
        return _interpolateDistanceGrid(i0, j0, k0, i1, j1, k1, id, jd, kd);
    }

    // we are outside the bounds of the grid - so clamp to the grid border
    const int clamped_i0 = std::clamp(i0, 0, _N-1);     const int clamped_i1 = std::clamp(i1, 0, _N-1);
    const int clamped_j0 = std::clamp(j0, 0, _N-1);     const int clamped_j1 = std::clamp(j1, 0, _N-1);
    const int clamped_k0 = std::clamp(k0, 0, _N-1);     const int clamped_k1 = std::clamp(k1, 0, _N-1);

    // get the distance and gradient from the border point
    const Real dist_from_border = _interpolateDistanceGrid(clamped_i0, clamped_j0, clamped_k0, clamped_i1, clamped_j1, clamped_k1, id, jd, kd);
    const Vec3r border_point = _gridPointFromIJK(   (clamped_i0 == i0) ? ijk[0] : (Real)clamped_i0,
                                                    (clamped_j0 == j0) ? ijk[1] : (Real)clamped_j0,
                                                    (clamped_k0 == k0) ? ijk[2] : (Real)clamped_k0  );

    // if we don't have the SDF gradient, we can't get the closest point on the mesh
    // so the best we can do is an estimate ==> (Distance From p to SDF Grid border) + (Distance from SDF Grid border to mesh)
    if (!_with_gradient)
    {
        return dist_from_border + (p_local - border_point).norm();    // THIS IS NOT EXACT! But without the gradient (I think) this is the best we can do
    }

    // if we have the gradient, we can be more exact by getting the closest point on the mesh
    const Vec3r grad_from_border = _interpolateGradientGrid(clamped_i0, clamped_j0, clamped_k0, clamped_i1, clamped_j1, clamped_k1, id, jd, kd);
    
    // find the closest point on the mesh from the grid border
    const Vec3r closest_point = border_point - dist_from_border * grad_from_border;

    // now that we have the closest point on the mesh, we can find the distance
    return (p_local - closest_point).norm();
}

Vec3r MeshSDF::gradient(const Vec3r& p) const
{
    // convert to local frame
    Vec3r p_local = _particle->orientation.transpose() * (p - _particle->position);

    assert(_with_gradient);
    const Vec3r& ijk = _gridIJKFromPoint(p_local);
    const int i0 = std::floor(ijk[0]);    const int i1 = i0+1;
    const int j0 = std::floor(ijk[1]);    const int j1 = j0+1;
    const int k0 = std::floor(ijk[2]);    const int k1 = k0+1;

    // the decimal part of the (i,j,k) coordinates - becomes the interpolation parameters
    const Real id = ijk[0] - i0;
    const Real jd = ijk[1] - j0;
    const Real kd = ijk[2] - k0;

    // check to make sure we are inside the grid boundaries
    if (i0 >= 0 && i0 < _N-1 && j0 >= 0 && j0 < _N-1 && k0 >= 0 && k0 < _N-1)
    {
        return _interpolateGradientGrid(i0, j0, k0, i1, j1, k1, id, jd, kd);
    }

    // we are outside the bounds of the grid - so clamp to the grid border
    const int clamped_i0 = std::clamp(i0, 0, _N-1);     const int clamped_i1 = std::clamp(i1, 0, _N-1);
    const int clamped_j0 = std::clamp(j0, 0, _N-1);     const int clamped_j1 = std::clamp(j1, 0, _N-1);
    const int clamped_k0 = std::clamp(k0, 0, _N-1);     const int clamped_k1 = std::clamp(k1, 0, _N-1);

    // get the distance and gradient from the border point
    const Real dist_from_border = _interpolateDistanceGrid(clamped_i0, clamped_j0, clamped_k0, clamped_i1, clamped_j1, clamped_k1, id, jd, kd);
    const Vec3r grad_from_border = _interpolateGradientGrid(clamped_i0, clamped_j0, clamped_k0, clamped_i1, clamped_j1, clamped_k1, id, jd, kd);
    const Vec3r border_point = _gridPointFromIJK(   (clamped_i0 == i0) ? ijk[0] : (Real)clamped_i0,
                                                    (clamped_j0 == j0) ? ijk[1] : (Real)clamped_j0,
                                                    (clamped_k0 == k0) ? ijk[2] : (Real)clamped_k0  );
    // find the closest point on the mesh from the grid border
    const Vec3r closest_point = border_point - dist_from_border * grad_from_border;

    Vec3r grad_local = (p_local - closest_point).normalized();
    return _particle->orientation * grad_local;
}

void MeshSDF::writeToFile(const std::string& filename) const
{
    std::cout << "Writing SDF to " << filename << "..." << std::endl;

    std::vector<std::byte> buf;
    serialize(buf);

    std::ofstream outfile(filename, std::ios::binary);
    if (!outfile.is_open())
    {
        std::cerr << "Error opening file " << filename << "!" << std::endl;
        assert(0);
    }

    outfile.write(reinterpret_cast<const char*>(buf.data()), buf.size());
    outfile.close();

    std::cout << "Done!" << std::endl;

}

void MeshSDF::_loadSDFFromFile(const std::string& filename)
{
    std::ifstream file(filename, std::ios::binary);
    if (!file)
    {
        std::cerr << "Error opening file " << filename << "!" << std::endl;
        assert(0);
    }
    
    // get file size
    file.seekg(0, std::ios::end);
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);
    
    // read into buffer
    std::vector<std::byte> buf(size);
    file.read(reinterpret_cast<char*>(buf.data()), size);
    const std::byte* cursor = buf.data();
    deserialize(cursor);
    
    std::cout << "Finished reading SDF from file..." << std::endl;
    std::cout << "\tGrid size: " << _N << " x " << _N << " x " << _N << std::endl;
    std::cout << "\tCell size: " << _cell_size[0] << ", " << _cell_size[1] << ", " << _cell_size[2] << std::endl;
    std::cout << "\tGrid Bounding box: (" << _grid_bbox_min[0] << ", " << _grid_bbox_min[1] << ", " << _grid_bbox_min[2] << ") to (" << _grid_bbox_max[0] << ", " << _grid_bbox_max[1] << ", " << _grid_bbox_max[2] << ")" << std::endl;
    std::cout << "\tMesh Bounding box: (" << _mesh_bbox_min[0] << ", " << _mesh_bbox_min[1] << ", " << _mesh_bbox_min[2] << ") to (" << _mesh_bbox_max[0] << ", " << _mesh_bbox_max[1] << ", " << _mesh_bbox_max[2] << ")" << std::endl;
    std::cout << "\tWith gradients: " << (_with_gradient ? "True" : "False") << std::endl;
}

void MeshSDF::_makeSDF(const std::vector<Vec3r>& verts, const std::vector<Vec3i>& tris, int padding, bool with_gradient)
{
    // set up the grids based on the input parameters
    _distance_grid.resize(_N, _N, _N);

    // compute the bounding box around the vertices
    Vec3r vertex_mins = verts[0];
    Vec3r vertex_maxs = verts[0];
    for (const auto& vert : verts)
    {
        vertex_mins = vertex_mins.cwiseMin(vert);
        vertex_maxs = vertex_maxs.cwiseMax(vert);
    }

    // compute the cell size for the grid in absolute units, accomodating for padding cells on all sides
    _cell_size = (vertex_maxs - vertex_mins) / (_N - padding*2);

    // compute the SDF bounding box limits
    _grid_bbox_min = vertex_mins - _cell_size * padding;
    _grid_bbox_max = vertex_maxs + _cell_size * padding;

    // store the mesh bounding box limits
    _mesh_bbox_min = vertex_mins;
    _mesh_bbox_max = vertex_maxs;

    _mesh_mass_center = Mesh::massCenter(verts, tris);

    // initialize distances with really large value
    _distance_grid.assign(std::numeric_limits<Real>::max());


    Array3i closest_tri(_N, _N, _N, -1);    // keeps track of index of closest triangle to each grid point
    Array3i intersection_count(_N, _N, _N, 0); // intersection_count(i,j,k) is # of tri intersections in (i-1,i]x{j}x{k}


    // tracks the number of edge/vertex intersections occur while doing intersection testing
    // by tracking when we intersect with an edge or vertex, we can be sure that we only count an intersection once,
    // leading to accurate inside/outside information
    //
    // the key is the grid index of the grid point (mapped from 3D to 1D)
    // the value is a vector that stores the indices of vertices (or the first vertex in the edge for an edge intersection) directly intersected
    std::map<int, std::vector<unsigned>> direct_hits;

    // no idea what this does, but it was in the original implementation
    const int exact_band = 1;

    for(unsigned ti = 0; ti < tris.size(); ti++){
        // extract triangle vertices
        const int p = tris[ti][0];
        const int q = tris[ti][1];
        const int r = tris[ti][2];

        const Vec3r& vp = verts[p];
        const Vec3r& vq = verts[q];
        const Vec3r& vr = verts[r];

        // coordinates in grid to high precision
        Real fip=((Real)vp[0]-_grid_bbox_min[0])/_cell_size[0], fjp=((Real)vp[1]-_grid_bbox_min[1])/_cell_size[1], fkp=((Real)vp[2]-_grid_bbox_min[2])/_cell_size[2];
        Real fiq=((Real)vq[0]-_grid_bbox_min[0])/_cell_size[0], fjq=((Real)vq[1]-_grid_bbox_min[1])/_cell_size[1], fkq=((Real)vq[2]-_grid_bbox_min[2])/_cell_size[2];
        Real fir=((Real)vr[0]-_grid_bbox_min[0])/_cell_size[0], fjr=((Real)vr[1]-_grid_bbox_min[1])/_cell_size[1], fkr=((Real)vr[2]-_grid_bbox_min[2])/_cell_size[2];

        // do distances nearby
        int i0=std::clamp(int(std::min({fip,fiq,fir}))-exact_band, 0, _N-1), i1=std::clamp(int(std::max({fip,fiq,fir}))+exact_band+1, 0, _N-1);
        int j0=std::clamp(int(std::min({fjp,fjq,fjr}))-exact_band, 0, _N-1), j1=std::clamp(int(std::max({fjp,fjq,fjr}))+exact_band+1, 0, _N-1);
        int k0=std::clamp(int(std::min({fkp,fkq,fkr}))-exact_band, 0, _N-1), k1=std::clamp(int(std::max({fkp,fkq,fkr}))+exact_band+1, 0, _N-1);

        for (int k = k0; k <= k1; k++)  for (int j = j0; j <= j1; j++)  for (int i = i0; i <= i1; i++)
        {
            const Vec3r grid_point(i*_cell_size[0]+_grid_bbox_min[0], j*_cell_size[1]+_grid_bbox_min[1], k*_cell_size[2]+_grid_bbox_min[2]);
            Real dist = Math::pointTriangleDistance(grid_point, vp, vq, vr);
            if(dist < _distance_grid(i,j,k)){
                _distance_grid(i,j,k) = dist;
                closest_tri(i,j,k) = ti;
            }
        }
        // and do intersection counts
        j0=std::clamp((int)std::ceil(std::min({fjp,fjq,fjr})), 0, _N-1);
        j1=std::clamp((int)std::floor(std::max({fjp,fjq,fjr})), 0, _N-1);
        k0=std::clamp((int)std::ceil(std::min({fkp,fkq,fkr})), 0, _N-1);
        k1=std::clamp((int)std::floor(std::max({fkp,fkq,fkr})), 0, _N-1);
        for (int k = k0; k <= k1; k++)  for (int j = j0; j <= j1; j++)
        {
            // check if this grid point is inside the triangle projected in the YZ-plane
            Real a, b, c;
            bool in_triangle = Math::pointInTriangle2D(j, k, fjp, fkp, fjq, fkq, fjr, fkr, a, b, c);
            
            if (in_triangle)
            {
                Real fi=a*fip+b*fiq+c*fir; // intersection i coordinate
                int i_interval=std::max(0, int(std::ceil(fi))); // intersection is in (i_interval-1,i_interval]

                // SPECIAL CASE: the ray directly intersects a vertex or edge of the triangle
                //  - since vertices and edges are shared by multiple triangles, we must keep track of the vertices/edges we hit directly so that
                //    we only count 1 intersection
                if ( (a-GEOMETRY_EPS) <= 0 || (b-GEOMETRY_EPS) <= 0 || (c-GEOMETRY_EPS) <= 0)
                {
                    unsigned direct_hit = -1;
                    if ( (a-GEOMETRY_EPS) <= 0 && (b-GEOMETRY_EPS) <= 0)      direct_hit = r; // intersects vertex R
                    else if ( (a-GEOMETRY_EPS) <= 0 && (c-GEOMETRY_EPS) <= 0) direct_hit = q; // intersects vertex Q
                    else if ( (b-GEOMETRY_EPS) <= 0 && (c-GEOMETRY_EPS) <= 0) direct_hit = p; // intersects vertex P
                    else if ( (a-GEOMETRY_EPS) <= 0) direct_hit = std::min(q,r); // intersects edge QR
                    else if ( (b-GEOMETRY_EPS) <= 0) direct_hit = std::min(p,r); // intersects edge PR
                    else if ( (c-GEOMETRY_EPS) <= 0) direct_hit = std::min(p,q); // intersects edge PQ
                    else    assert(0);  // shouldn't ever get to here

                    // convert (i,j,k) to a unique integer key for the map
                    int key = i_interval*_N*_N + j*_N + k;
                    
                    // if key does not exist in map, create an empty vector there
                    if (direct_hits.count(key) == 0)
                    {
                        direct_hits[key] = std::vector<unsigned>();
                    }

                    std::vector<unsigned>& vec = direct_hits[key];
                    // if we've already counted an intersection for this edge or vertex, skip the intersection counting
                    if (std::find(vec.begin(), vec.end(), direct_hit) != vec.end())
                        continue;
                    // otherwise register that we've hit this edge or vertex, and go on to count the intersection
                    else
                        vec.push_back(direct_hit);
                }

                // count the intersection
                if(i_interval < 0) ++intersection_count(0, j, k); // we enlarge the first interval to include everything to the -x direction
                else if(i_interval < _N) ++intersection_count(i_interval,j,k);
                // we ignore intersections that are beyond the +x side of the grid
            }
        }
    }

    // and now we fill in the rest of the distances with fast sweeping
    for(unsigned int pass=0; pass<2; ++pass){
        _sweep(verts, tris, closest_tri, +1, +1, +1);
        _sweep(verts, tris, closest_tri, -1, -1, -1);
        _sweep(verts, tris, closest_tri, +1, +1, -1);
        _sweep(verts, tris, closest_tri, -1, -1, +1);
        _sweep(verts, tris, closest_tri, +1, -1, +1);
        _sweep(verts, tris, closest_tri, -1, +1, -1);
        _sweep(verts, tris, closest_tri, +1, -1, -1);
        _sweep(verts, tris, closest_tri, -1, +1, +1);
    }

    // then figure out signs (inside/outside) from intersection counts
    for (int k=0; k<_N; ++k)    for (int j=0; j<_N; ++j)
    {
        int total_count=0;
        for(int i=0; i<_N; ++i){
            total_count+=intersection_count(i,j,k);
            if(total_count%2==1){ // if parity of intersections so far is odd,
                _distance_grid(i,j,k) = -_distance_grid(i,j,k); // we are inside the mesh
            }
        }
    }

    // evaluate gradients if required
    if (with_gradient)
    {
        _gradient_grid.resize(_N, _N, _N);
        // do gradients from closest tris
        for (int k=0; k<_N; ++k) for (int j=0; j<_N; ++j) for (int i=0; i<_N; ++i)
        {
            const int ti = closest_tri(i,j,k);
            // extract triangle vertices
            const int p = tris[ti][0];
            const int q = tris[ti][1];
            const int r = tris[ti][2];

            const Vec3r& vp = verts[p];
            const Vec3r& vq = verts[q];
            const Vec3r& vr = verts[r];

            const Vec3r grid_point = _gridPointFromIJK(i,j,k);
            const Vec3r grad = Math::pointTriangleDirection(grid_point, vp, vq, vr);

            if (_distance_grid.at(i,j,k) < 0)   _gradient_grid.at(i,j,k) = -grad;
            else                                _gradient_grid.at(i,j,k) = grad;
            
        }
    }

    std::cout << "Finished creating SDF from mesh..." << std::endl;
    std::cout << "\tGrid size: " << _N << " x " << _N << " x " << _N << std::endl;
    std::cout << "\tCell size: " << _cell_size[0] << ", " << _cell_size[1] << ", " << _cell_size[2] << std::endl;
    std::cout << "\tBounding box: (" << _grid_bbox_min[0] << ", " << _grid_bbox_min[1] << ", " << _grid_bbox_min[2] << ") to (" << _grid_bbox_max[0] << ", " << _grid_bbox_max[1] << ", " << _grid_bbox_max[2] << ")" << std::endl;
    std::cout << "\tWith gradients: " << (_with_gradient ? "True" : "False") << std::endl;
    
}

void MeshSDF::_checkNeighbor(const std::vector<Vec3r>& verts, const std::vector<Vec3i>& tris, 
                        Array3i& closest_tri,
                        const Vec3r& grid_point,
                        int i0, int j0, int k0, int i1, int j1, int k1)
{
    const int ti = closest_tri(i1,j1,k1);
    if (ti >= 0)
    {
        // extract triangle vertices
        const int p = tris[ti][0];
        const int q = tris[ti][1];
        const int r = tris[ti][2];

        const Vec3r& vp = verts[p];
        const Vec3r& vq = verts[q];
        const Vec3r& vr = verts[r];
        
        Real dist = Math::pointTriangleDistance(grid_point, vp, vq, vr);
        if (dist < _distance_grid(i0,j0,k0))
        {
            _distance_grid(i0,j0,k0) = dist;
            closest_tri(i0,j0,k0) = ti;
        }
    }
}

void MeshSDF::_sweep(const std::vector<Vec3r>& verts, const std::vector<Vec3i>& tris,
                Array3i& closest_tri,
                int di, int dj, int dk)
{
    int i0, i1;
    if(di>0){ i0=1; i1=_N; }
    else{ i0=_N-2; i1=-1; }
    int j0, j1;
    if(dj>0){ j0=1; j1=_N; }
    else{ j0=_N-2; j1=-1; }
    int k0, k1;
    if(dk>0){ k0=1; k1=_N; }
    else{ k0=_N-2; k1=-1; }

    for(int k=k0; k!=k1; k+=dk) for(int j=j0; j!=j1; j+=dj) for(int i=i0; i!=i1; i+=di){
      const Vec3r grid_point = _gridPointFromIJK(i,j,k);
      _checkNeighbor(verts, tris, closest_tri, grid_point, i, j, k, i-di, j,    k   );
      _checkNeighbor(verts, tris, closest_tri, grid_point, i, j, k, i,    j-dj, k   );
      _checkNeighbor(verts, tris, closest_tri, grid_point, i, j, k, i-di, j-dj, k   );
      _checkNeighbor(verts, tris, closest_tri, grid_point, i, j, k, i,    j,    k-dk);
      _checkNeighbor(verts, tris, closest_tri, grid_point, i, j, k, i-di, j,    k-dk);
      _checkNeighbor(verts, tris, closest_tri, grid_point, i, j, k, i,    j-dj, k-dk);
      _checkNeighbor(verts, tris, closest_tri, grid_point, i, j, k, i-di, j-dj, k-dk);
   }

}

Vec3r MeshSDF::_gridPointFromIJK(int i, int j, int k) const
{
    return Vec3r(i*_cell_size[0]+_grid_bbox_min[0], j*_cell_size[1]+_grid_bbox_min[1], k*_cell_size[2]+_grid_bbox_min[2]);
}

Vec3r MeshSDF::_gridPointFromIJK(Real i, Real j, Real k) const
{
    return Vec3r(i*_cell_size[0]+_grid_bbox_min[0], j*_cell_size[1]+_grid_bbox_min[1], k*_cell_size[2]+_grid_bbox_min[2]);
}

Vec3r MeshSDF::_gridIJKFromPoint(const Vec3r& p) const
{
    return (p - _grid_bbox_min).array() / _cell_size.array();
}

Real MeshSDF::_interpolateDistanceGrid(int i0, int j0, int k0, int i1, int j1, int k1, Real id, Real jd, Real kd) const
{
    // trilinear interpolation
    const Real c000 = _distance_grid.at(i0,j0,k0);    const Real c100 = _distance_grid.at(i1,j0,k0);
    const Real c001 = _distance_grid.at(i0,j0,k1);    const Real c101 = _distance_grid.at(i1,j0,k1);
    const Real c010 = _distance_grid.at(i0,j1,k0);    const Real c110 = _distance_grid.at(i1,j1,k0);
    const Real c011 = _distance_grid.at(i0,j1,k1);    const Real c111 = _distance_grid.at(i1,j1,k1);
    const Real c00 = c000*(1-id) + c100*id;
    const Real c01 = c001*(1-id) + c101*id;
    const Real c10 = c010*(1-id) + c110*id;
    const Real c11 = c011*(1-id) + c111*id;
    const Real c0 = c00*(1-jd) + c10*jd;
    const Real c1 = c01*(1-jd) + c11*jd;
    const Real c = c0*(1-kd) + c1*kd;

    return c;
}

Vec3r MeshSDF::_interpolateGradientGrid(int i0, int j0, int k0, int i1, int j1, int k1, Real id, Real jd, Real kd) const
{
    const Vec3r& c000 = _gradient_grid.at(i0,j0,k0);    const Vec3r& c100 = _gradient_grid.at(i1,j0,k0);
    const Vec3r& c001 = _gradient_grid.at(i0,j0,k1);    const Vec3r& c101 = _gradient_grid.at(i1,j0,k1);
    const Vec3r& c010 = _gradient_grid.at(i0,j1,k0);    const Vec3r& c110 = _gradient_grid.at(i1,j1,k0);
    const Vec3r& c011 = _gradient_grid.at(i0,j1,k1);    const Vec3r& c111 = _gradient_grid.at(i1,j1,k1);
    return Math::vectorTriSlerp(c000, c100, c010, c110, c001, c101, c011, c111, id, jd, kd);
}

void MeshSDF::serialize(std::vector<std::byte>& buf) const
{
    pack(buf, _N);
    pack(buf, _cell_size);
    pack(buf, _grid_bbox_min);
    pack(buf, _grid_bbox_max);
    pack(buf, _mesh_bbox_min);
    pack(buf, _mesh_bbox_max);
    pack(buf, _mesh_mass_center);
    pack(buf, _with_gradient);
    pack(buf, _distance_grid);
    pack(buf, _gradient_grid);
}

void MeshSDF::deserialize(const std::byte*& buf)
{
    unpack(buf, _N);
    unpack(buf, _cell_size);
    unpack(buf, _grid_bbox_min);
    unpack(buf, _grid_bbox_max);
    unpack(buf, _mesh_bbox_min);
    unpack(buf, _mesh_bbox_max);
    unpack(buf, _mesh_mass_center);
    unpack(buf, _with_gradient);
    unpack(buf, _distance_grid);
    unpack(buf, _gradient_grid);
}

} // namespace Collision