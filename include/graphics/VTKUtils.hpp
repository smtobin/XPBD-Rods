#pragma once

#include "config/ObjectRenderConfig.hpp"

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

#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
#include <vtkPointData.h>

#include <vtkActor.h>
#include <vtkNew.h>
#include <vtkSmartPointer.h>

#include <filesystem>
#include <optional>

namespace Graphics
{

struct VTKUtils
{
    static void setupActorFromRenderConfig(vtkActor* actor, const Config::ObjectRenderConfig& render_config)
    {
        // select rendering type
        Config::ObjectRenderConfig::RenderType render_type = render_config.renderType();
        if (render_type == Config::ObjectRenderConfig::RenderType::FLAT)
        {
            actor->GetProperty()->SetInterpolationToFlat();
        }
        else if (render_type == Config::ObjectRenderConfig::RenderType::PHONG)
        {
            actor->GetProperty()->SetInterpolationToPhong();
        }
        else if (render_type == Config::ObjectRenderConfig::RenderType::PBR)
        {
            actor->GetProperty()->SetInterpolationToPBR();
        }

        // set ORM (occulsion, roughness, metallicity) texture from given filename
        std::optional<std::string> orm_file = render_config.ormTextureFilename();
        if (orm_file.has_value())
        {
            // make sure the file exists
            if (!std::filesystem::exists(orm_file.value()))
            {
                std::cout << "ORM texture file \"" << orm_file.value() << "\" does not exist!" << std::endl;
                assert(0);
            }

            // make sure the file is a PNG
            if (std::filesystem::path(orm_file.value()).extension().compare(".png") != 0)
            {
                std::cout << "ORM texture file \"" << orm_file.value() << "\" does not have .png file extension!" << std::endl;
                assert(0);
            }

            // read the texture from PNG
            vtkNew<vtkPNGReader> orm_reader;
            orm_reader->SetFileName(orm_file.value().c_str());
            vtkNew<vtkTexture> material;
            material->InterpolateOn();
            material->SetInputConnection(orm_reader->GetOutputPort());
            actor->GetProperty()->SetORMTexture(material);
        }

        // set base color texture from given filename
        std::optional<std::string> base_color_file = render_config.baseColorTextureFilename();
        if (base_color_file.has_value())
        {
            // make sure the file exists
            if (!std::filesystem::exists(base_color_file.value()))
            {
                std::cout << "Base color texture file \"" << base_color_file.value() << "\" does not exist!" << std::endl;
                assert(0);
            }

            // make sure the file is a PNG
            if (std::filesystem::path(base_color_file.value()).extension().compare(".png") != 0)
            {
                std::cout << "Base color texture file \"" << base_color_file.value() << "\" does not have .png file extension!" << std::endl;
                assert(0);
            }

            // read the texture from PNG
            vtkNew<vtkPNGReader> color_reader;
            color_reader->SetFileName(base_color_file.value().c_str());
            vtkNew<vtkTexture> color;
            color->UseSRGBColorSpaceOn();
            color->InterpolateOn();
            color->SetInputConnection(color_reader->GetOutputPort());
            actor->GetProperty()->SetBaseColorTexture(color);
        }

        // set normals texture from given filename
        std::optional<std::string> normals_file = render_config.normalsTextureFilename();
        if (normals_file.has_value())
        {
            // make sure the file exists
            if (!std::filesystem::exists(normals_file.value()))
            {
                std::cout << "Normals texture file \"" << normals_file.value() << "\" does not exist!" << std::endl;
                assert(0);
            }

            // make sure the file is a PNG
            if (std::filesystem::path(normals_file.value()).extension().compare(".png") != 0)
            {
                std::cout << "Normals texture file \"" << normals_file.value() << "\" does not have .png file extension!" << std::endl;
                assert(0);
            }

            // read the texture from PNG
            vtkNew<vtkPNGReader> normals_reader;
            normals_reader->SetFileName(normals_file.value().c_str());
            vtkNew<vtkTexture> normals;
            normals->SetMipmap(true);   // make sure to set mipmaps on, this eliminates "sparkly" effect
            normals->InterpolateOn();
            normals->SetMaximumAnisotropicFiltering(8);
            normals->SetInputConnection(normals_reader->GetOutputPort());
            actor->GetProperty()->SetNormalTexture(normals);
        }

        // set the color of the object
        // if a base color texture is not specified (or the object has no UV coordinates), then it will fall back to the base color
        if (render_config.color().has_value())
        {
            Vec3r solid_color = render_config.color().value();
            actor->GetProperty()->SetColor(solid_color[0], solid_color[1], solid_color[2]);
        }
        else if (render_config.colors().has_value())
        {
            Vec3r solid_color = render_config.colors().value()[0];
            actor->GetProperty()->SetColor(solid_color[0], solid_color[1], solid_color[2]);
        }
        

        // set the roughness and metallicity
        actor->GetProperty()->SetMetallic(render_config.metallic());
        actor->GetProperty()->SetRoughness(render_config.roughness());

        // set opacity
        actor->GetProperty()->SetOpacity(render_config.opacity());

        /** TODO: add more render options */
    }
};

} // namespace Graphics