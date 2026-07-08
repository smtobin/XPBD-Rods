#include "graphics/HigherOrderRodGraphicsObject.hpp"
#include "graphics/VTKUtils.hpp"

#include <vtkTriangle.h>
#include <vtkPolygon.h>
#include <vtkQuad.h>
#include <vtkCellArray.h>
#include <vtkFloatArray.h>
#include <vtkProperty.h>

#include <vtkTexture.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataTangents.h>
#include <vtkPolyDataMapper.h>
#include <vtkPNGReader.h>
#include <vtkCleanPolyData.h>
#include <vtkImageData.h>
#include <vtkPolyLine.h>
#include <vtkTubeFilter.h>
#include <vtkLookupTable.h>
#include <vtkCellData.h>

#include <filesystem>
#include <optional>

namespace Graphics
{

template<typename ElementType>
HigherOrderRodGraphicsObject<ElementType>::HigherOrderRodGraphicsObject(const SimObject::XPBDRod_<ElementType>* rod, const Config::ObjectRenderConfig& render_config)
    : GraphicsObject(render_config), _rod(rod), _render_config(render_config), 
    _color_elements(render_config.colorElements()),
    _num_samples(render_config.numCenterlineSamples()), _draw_centerline(render_config.drawCenterline()), _draw_end_caps(render_config.drawEndCaps())
{

    // if we are coloring the elements individually, the number of centerline samples is a multiple of the elements + 1 so that some samples are at element boundaries
    if (_color_elements)
    {
        _num_samples = _rod->elements().size() * _sample_points_per_element + 1;
    }
    
    _vtk_poly_data = vtkSmartPointer<vtkPolyData>::New();
    _vtk_poly_data_normals = vtkSmartPointer<vtkPolyDataNormals>::New();
    _vtk_poly_data_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();

    _generateInitialPolyData();

    vtkNew<vtkPolyDataMapper> data_mapper;
    if (render_config.smoothNormals())
    {
        // smooth normals
        vtkNew<vtkPolyDataNormals> normal_generator;
        normal_generator->SetInputData(_vtk_poly_data);
        normal_generator->SetFeatureAngle(30.0);
        normal_generator->SplittingOff();
        normal_generator->ConsistencyOn();
        normal_generator->ComputePointNormalsOn();
        normal_generator->ComputeCellNormalsOff();
        normal_generator->Update();

        data_mapper->SetInputConnection(normal_generator->GetOutputPort());
    }
    else
    {   
        data_mapper->SetInputData(_vtk_poly_data);
    }

    // set up the lookup table only if color-by-elements is enabled
    if (_color_elements)
    {
        int numElements = (int)_rod->elements().size();
        vtkNew<vtkLookupTable> lut;
        lut->SetNumberOfTableValues(numElements);
        lut->Build();

        // Alternating two colors so adjacent elements are visually distinct
        // set up the alternate color
        Vec3r alt_color;
        for (int i = 0; i < 3; i++)
        {
            alt_color[i] = (_render_config.color()[i] > 0.5) ? _render_config.color()[i] - 0.3 : _render_config.color()[i] + 0.3;
            if (_render_config.color()[i] == 0.0)
                alt_color[i] = 1.0;
        }
            
        for (int i = 0; i < numElements; i++)
        {
            if (i % 2 == 0)
                lut->SetTableValue(i, _render_config.color()[0], _render_config.color()[1], _render_config.color()[2], 1.0);
            else
            {
                
                lut->SetTableValue(i, alt_color[0], alt_color[1], alt_color[2], 1.0);
            }
                
        }

        data_mapper->SetScalarRange(0, numElements - 1);
        data_mapper->SetLookupTable(lut);
        data_mapper->SetScalarModeToUseCellData();
        data_mapper->ScalarVisibilityOn();
    }

    _vtk_actor = vtkSmartPointer<vtkActor>::New();
    _vtk_actor->SetMapper(data_mapper);

    VTKUtils::setupActorFromRenderConfig(_vtk_actor.Get(), render_config);

    // create centerline if enabled
    if (_draw_centerline)
    {
        vtkNew<vtkPoints> centerline_points;
        const std::vector<ElementType>& elements = _rod->elements();
        const std::vector<SimObject::OrientedParticle>& nodes = _rod->nodes();
        for (int si = 0; si < _num_samples; si++)
        {
            // position along rod in [0, 1]
            Real s = (Real)si / (_num_samples - 1);
            // the element index that s corresponds to
            int elem_ind = std::clamp(static_cast<int>(s * elements.size()), 0, static_cast<int>(elements.size()-1));
            // the "local" s within the element
            Real s_hat = s * elements.size() - (Real)elem_ind;

            Vec3r position = elements[elem_ind].position(s_hat);

            centerline_points->InsertNextPoint(position[0], position[1], position[2]);
            
        }

        vtkNew<vtkPolyLine> line;
        line->GetPointIds()->SetNumberOfIds(_num_samples);
        for (int i = 0; i < _num_samples; i++)
        {
            line->GetPointIds()->SetId(i, i);
        }

        vtkNew<vtkCellArray> cells;
        cells->InsertNextCell(line);

        _vtk_centerline_poly_data = vtkSmartPointer<vtkPolyData>::New();
        _vtk_centerline_poly_data->SetPoints(centerline_points);
        _vtk_centerline_poly_data->SetLines(cells);

        vtkNew<vtkTubeFilter> tube;
        tube->SetInputData(_vtk_centerline_poly_data);

        tube->SetRadius(_rod->radius()/10);
        tube->SetNumberOfSides(12);
        tube->CappingOn();

        vtkNew<vtkPolyDataMapper> mapper;
        mapper->SetInputConnection(tube->GetOutputPort());

        _vtk_centerline_actor = vtkSmartPointer<vtkActor>::New();
        _vtk_centerline_actor->SetMapper(mapper);
        VTKUtils::setupActorFromRenderConfig(_vtk_centerline_actor.Get(), render_config);
        _vtk_centerline_actor->GetProperty()->SetOpacity(1.0); // opacity of centerline is always 1
    }

    // Add this step for smooth normals
    // vtkNew<vtkPolyDataNormals> normalGenerator;
    // normalGenerator->SetInputData(_vtk_poly_data);
    // normalGenerator->SetFeatureAngle(30.0); // Force smooth normals
    // normalGenerator->SplittingOff();
    // normalGenerator->ConsistencyOn();
    // normalGenerator->AutoOrientNormalsOn();
    // normalGenerator->ComputePointNormalsOn();
    // normalGenerator->ComputeCellNormalsOff();
    // normalGenerator->FlipNormalsOff();
    // normalGenerator->Update();

    // vtkNew<vtkPolyDataTangents> tangents;
    // tangents->SetInputConnection(normalGenerator->GetOutputPort());
    // tangents->Update();

    // vtkNew<vtkPolyDataMapper> mapper;
    // mapper->SetInputConnection(tangents->GetOutputPort());
    // _vtk_actor->SetMapper(mapper);

}

template<typename ElementType>
void HigherOrderRodGraphicsObject<ElementType>::update()
{
    _updatePolyData();

    const std::vector<ElementType>& elements = _rod->elements();
    if (_draw_centerline)
    {
        vtkPoints* points = _vtk_centerline_poly_data->GetPoints();
        for (int si = 0; si < _num_samples; si++)
        {
            // position along rod in [0, 1]
            Real s = (Real)si / (_num_samples - 1);
            // the element index that s corresponds to
            int elem_ind = std::clamp(static_cast<int>(s * elements.size()), 0, static_cast<int>(elements.size()-1));
            // the "local" s within the element
            Real s_hat = s * elements.size() - (Real)elem_ind;

            Vec3r position = elements[elem_ind].position(s_hat);
            points->SetPoint( si, position[0], position[1], position[2] );
        }
        points->Modified();
    }

}

template<typename ElementType>
void HigherOrderRodGraphicsObject<ElementType>::_generateInitialPolyData()
{
    const std::vector<ElementType>& elements = _rod->elements();

    const std::vector<SimObject::OrientedParticle>& nodes = _rod->nodes();

    vtkNew<vtkPoints> points;

    // base cap
    Mat3r R_beg = nodes.front().orientation;
    Vec3r p_beg = nodes.front().position;
    if (_draw_end_caps)
    {
        for (int i = _cap_resolution-1; i >= 0; i--)
        {
            Real phi = (M_PI / 2.0) * i / _cap_resolution;

            Real ring_radius = _rod->radius() * std::cos(phi);
            Real z = _rod->radius() * std::sin(phi);

            for (int j = 0; j < _tubular_resolution; j++)
            {
                Real theta = 2.0 * M_PI * j / _tubular_resolution;

                Vec3r p(
                    ring_radius * std::cos(theta),
                    ring_radius * std::sin(theta),
                    -z);

                Vec3r transformed = R_beg * p + p_beg;
                points->InsertNextPoint(transformed.data());
            }
        }
    }

    // create cross-section points for a circular cross-section
    _cross_section_points.resize(_tubular_resolution);
    for (int i = 0; i < _tubular_resolution; i++)
    {
        const Real angle = i * 2 * M_PI / _tubular_resolution;
        _cross_section_points[i] = Vec3r( _rod->radius() * std::cos(angle), _rod->radius() * std::sin(angle), 0);
    }

    // insert cross-section vertices
    for (int si = 0; si < _num_samples; si++)
    {
        // position along rod in [0, 1]
        Real s = (Real)si / (_num_samples - 1);
        // the element index that s corresponds to
        int elem_ind = std::clamp(static_cast<int>(s * elements.size()), 0, static_cast<int>(elements.size()-1));
        // the "local" s within the element
        Real s_hat = s * elements.size() - (Real)elem_ind;

        Vec3r position = elements[elem_ind].position(s_hat);
        Mat3r orientation = elements[elem_ind].orientation(s_hat);
        // for (unsigned pi = 0; pi < cross_section_points.size(); pi++)
        for (const auto& p : _cross_section_points)
        {
            // transform each point in local cross section to global frame
            const Vec3r transformed = orientation * p + position;
            points->InsertNextPoint( transformed[0], transformed[1], transformed[2] );
        }
    }

    // end cap
    Mat3r R_end = nodes.back().orientation;
    Vec3r p_end = nodes.back().position;
    if (_draw_end_caps)
    {
        for (int i = 0; i < _cap_resolution; i++)
        {
            Real phi = (M_PI / 2.0) * i / _cap_resolution;

            Real ring_radius = _rod->radius() * std::cos(phi);
            Real z = _rod->radius() * std::sin(phi);

            for (int j = 0; j < _tubular_resolution; j++)
            {
                Real theta = 2.0 * M_PI * j / _tubular_resolution;

                Vec3r p(
                    ring_radius * std::cos(theta),
                    ring_radius * std::sin(theta),
                    z);

                Vec3r transformed = R_end * p + p_end;
                points->InsertNextPoint(transformed.data());
            }
        }
    }


    int total_num_rings = _num_samples;
    if (_draw_end_caps)
        total_num_rings += 2*_cap_resolution;

    // create faces inbetween nodes
    vtkNew<vtkCellArray> faces;
    for (int si = 0; si < total_num_rings-1; si++)
    {
        for (int pi = 0; pi < _tubular_resolution; pi++)
        {
            // vertices for the quad face
            const int v1 = si*_tubular_resolution + pi;
            const int v2 = (pi != _tubular_resolution-1) ? v1 + 1 : si*_tubular_resolution;
            const int v3 = v2 + _tubular_resolution;
            const int v4 = v1 + _tubular_resolution;

            vtkNew<vtkTriangle> tri1;
            tri1->GetPointIds()->SetId(0, v1);
            tri1->GetPointIds()->SetId(1, v2);
            tri1->GetPointIds()->SetId(2, v3);

            vtkNew<vtkTriangle> tri2;
            tri2->GetPointIds()->SetId(0, v1);
            tri2->GetPointIds()->SetId(1, v3);
            tri2->GetPointIds()->SetId(2, v4);
            faces->InsertNextCell(tri1);
            faces->InsertNextCell(tri2);
        }
    }

    // end cap points
    if (_draw_end_caps)
    {
        Vec3r base_pt = R_beg * Vec3r(0, 0, -_rod->radius()) + p_beg;
        Vec3r end_pt = R_end * Vec3r(0, 0, _rod->radius()) + p_end;
        points->InsertNextPoint(base_pt.data());
        points->InsertNextPoint(end_pt.data());
    }
    else
    {
        points->InsertNextPoint( nodes[0].position[0], nodes[0].position[1], nodes[0].position[2] );
        points->InsertNextPoint( nodes.back().position[0], nodes.back().position[1], nodes.back().position[2] );
    }

    // end cap faces
    // base faces
    for (int pi = 0; pi < _tubular_resolution; pi++)
    {
        const int v1 = pi;
        const int v2 = total_num_rings*_tubular_resolution;
        const int v3 = (pi != _tubular_resolution-1) ? v1 + 1 : 0;

        vtkNew<vtkTriangle> tri;
        tri->GetPointIds()->SetId(0, v1);
        tri->GetPointIds()->SetId(1, v2);
        tri->GetPointIds()->SetId(2, v3);
        faces->InsertNextCell(tri);
    }

    // tip faces
    for (int pi = 0; pi < _tubular_resolution; pi++)
    {
        const int v1 = (total_num_rings-1)*_tubular_resolution + pi;
        const int v2 = (pi != _tubular_resolution-1) ? v1 + 1 : (total_num_rings-1)*_tubular_resolution;
        const int v3 = total_num_rings*_tubular_resolution + 1;
        

        vtkNew<vtkTriangle> tri;
        tri->GetPointIds()->SetId(0, v1);
        tri->GetPointIds()->SetId(1, v2);
        tri->GetPointIds()->SetId(2, v3);
        faces->InsertNextCell(tri);
    }

    // vtkNew<vtkPolygon> base_face, end_face;
    // base_face->GetPointIds()->SetNumberOfIds(cross_section_points.size());
    // end_face->GetPointIds()->SetNumberOfIds(cross_section_points.size());
    // for (unsigned i = 0; i < cross_section_points.size(); i++)
    // {
    //     base_face->GetPointIds()->SetId(i, i);
    //     end_face->GetPointIds()->SetId(i, (nodes.size()-1)*cross_section_points.size() + i);
    // }
    // faces->InsertNextCell(base_face);
    // faces->InsertNextCell(end_face);

    // vtkNew<vtkPolyData> rod_poly_data;
    _vtk_poly_data->SetPoints(points);
    _vtk_poly_data->SetPolys(faces);

    // set per-cell coloring if we are coloring by elements
    if (_color_elements)
    {
        vtkNew<vtkIntArray> elementIds;
        elementIds->SetName("ElementIds");
        elementIds->SetNumberOfComponents(1);

        // Tube faces — each quad (2 tris) belongs to a sample interval.
        // Map sample interval -> element index.
        for (int si = 0; si < _num_samples - 1; si++)
        {
            Real s = (Real)si / (_num_samples - 1);
            int elem_ind = std::clamp(static_cast<int>(s * elements.size()),
                                    0, static_cast<int>(elements.size() - 1));

            for (int pi = 0; pi < _tubular_resolution; pi++)
            {
                elementIds->InsertNextValue(elem_ind); // tri1
                elementIds->InsertNextValue(elem_ind); // tri2
            }
        }

        // End cap tris — assign to element 0 and last element respectively
        for (int pi = 0; pi < _tubular_resolution; pi++)
            elementIds->InsertNextValue(0);

        for (int pi = 0; pi < _tubular_resolution; pi++)
            elementIds->InsertNextValue((int)elements.size() - 1);

        _vtk_poly_data->GetCellData()->SetScalars(elementIds);
    }

    // Create and set texture coordinates
    vtkNew<vtkFloatArray> textureCoords;
    textureCoords->SetNumberOfComponents(2);
    textureCoords->SetNumberOfTuples(nodes.size()*_tubular_resolution);
    textureCoords->SetName("TextureCoordinates");
    for (unsigned ni = 0; ni < nodes.size(); ni++)
    {
        float x_coord = (ni % 2 == 0) ? 0.0f : 0.5f;
        for (int pi = 0; pi < _tubular_resolution; pi++)
        {
            float y_coord = static_cast<float>(pi) / _tubular_resolution;
            textureCoords->SetTuple2(ni*_tubular_resolution + pi, x_coord, y_coord);
        }
    }
    _vtk_poly_data->GetPointData()->SetTCoords(textureCoords);
}

template<typename ElementType>
void HigherOrderRodGraphicsObject<ElementType>::_updatePolyData()
{
    // TODO: use vtkPoints::setData and vtkFloatArray? supposed to be faster

    const std::vector<SimObject::OrientedParticle>& nodes = _rod->nodes();
    const std::vector<ElementType>& elements = _rod->elements();

    vtkPoints* points = _vtk_poly_data->GetPoints();

    // base cap
    Mat3r R_beg = nodes.front().orientation;
    Vec3r p_beg = nodes.front().position;
    if (_draw_end_caps)
    {
        for (int i = _cap_resolution-1; i >= 0; i--)
        {
            Real phi = (M_PI / 2.0) * i / _cap_resolution;

            Real ring_radius = _rod->radius() * std::cos(phi);
            Real z = _rod->radius() * std::sin(phi);

            for (int j = 0; j < _tubular_resolution; j++)
            {
                Real theta = 2.0 * M_PI * j / _tubular_resolution;

                Vec3r p(
                    ring_radius * std::cos(theta),
                    ring_radius * std::sin(theta),
                    -z);

                Vec3r transformed = R_beg * p + p_beg;
                int offset = (_cap_resolution - 1 - i) * _tubular_resolution;
                points->SetPoint(offset + j, transformed.data());
            }
        }
    }

    int cs_start_index = _draw_end_caps ? _cap_resolution*_tubular_resolution : 0;
    for (int si = 0; si < _num_samples; si++)
    {
        // position along rod in [0, 1]
        Real s = (Real)si / (_num_samples - 1);
        // the element index that s corresponds to
        int elem_ind = std::clamp(static_cast<int>(s * elements.size()), 0, static_cast<int>(elements.size()-1));
        // the "local" s within the element
        Real s_hat = s * elements.size() - (Real)elem_ind;

        Vec3r position = elements[elem_ind].position(s_hat);

        // std::cout << "Centerline position " << si << ": " << position.transpose() << std::endl;

        Mat3r orientation = elements[elem_ind].orientation(s_hat);
        // for (unsigned pi = 0; pi < cross_section_points.size(); pi++)
        for (int pi = 0; pi < _tubular_resolution; pi++)
        {
            // transform each point in local cross section to global frame
            const Vec3r transformed = orientation * _cross_section_points[pi] + position;
            points->SetPoint( cs_start_index + si*_tubular_resolution + pi, transformed.data() );
        }
    }

    // end cap
    Mat3r R_end = nodes.back().orientation;
    Vec3r p_end = nodes.back().position;
    if (_draw_end_caps)
    {
        for (int i = 0; i < _cap_resolution; i++)
        {
            Real phi = (M_PI / 2.0) * i / _cap_resolution;

            Real ring_radius = _rod->radius() * std::cos(phi);
            Real z = _rod->radius() * std::sin(phi);

            for (int j = 0; j < _tubular_resolution; j++)
            {
                Real theta = 2.0 * M_PI * j / _tubular_resolution;

                Vec3r p(
                    ring_radius * std::cos(theta),
                    ring_radius * std::sin(theta),
                    z);

                Vec3r transformed = R_end * p + p_end;
                points->SetPoint((_cap_resolution+_num_samples)*_tubular_resolution + i*_tubular_resolution + j, transformed.data());
            }
        }
    }

    if (_draw_end_caps)
    {
        Vec3r base_pt = R_beg * Vec3r(0, 0, -_rod->radius()) + p_beg;
        Vec3r end_pt = R_end * Vec3r(0, 0, _rod->radius()) + p_end;
        points->SetPoint((2*_cap_resolution + _num_samples)*_tubular_resolution, base_pt.data());
        points->SetPoint((2*_cap_resolution + _num_samples)*_tubular_resolution+1, end_pt.data());
    }
    else
    {
        points->SetPoint(_num_samples*_tubular_resolution, nodes[0].position[0], nodes[0].position[1], nodes[0].position[2]);
        points->SetPoint(_num_samples*_tubular_resolution+1, nodes.back().position[0], nodes.back().position[1], nodes.back().position[2]);
    }

    points->Modified();
}

template class HigherOrderRodGraphicsObject<SimObject::RodElement<0>>;
template class HigherOrderRodGraphicsObject<SimObject::RodElement<1>>;
template class HigherOrderRodGraphicsObject<SimObject::RodElement<2>>;
template class HigherOrderRodGraphicsObject<SimObject::RodElement<3>>;
template class HigherOrderRodGraphicsObject<SimObject::CubicHermiteRodElement>;

} // namespace Graphics