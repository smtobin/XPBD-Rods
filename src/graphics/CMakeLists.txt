cmake_minimum_required(VERSION 3.13)
project(XPBDRods)

set(GRAPHICS_INCLUDE_DIR "../../include/graphics")
set(GRAPHICS_SRC_DIR "./")

set(GRAPHICS_HEADERS
    ${GRAPHICS_INCLUDE_DIR}/RodGraphicsObject.hpp
    ${GRAPHICS_INCLUDE_DIR}/GraphicsScene.hpp
    ${GRAPHICS_INCLUDE_DIR}/CustomVTKInteractorStyle.hpp
)

set(GRAPHICS_SOURCES
    ${GRAPHICS_SRC_DIR}/RodGraphicsObject.cpp
    ${GRAPHICS_SRC_DIR}/GraphicsScene.cpp
    ${GRAPHICS_SRC_DIR}/CustomVTKInteractorStyle.cpp
)

target_sources(Graphics
    PRIVATE
        ${GRAPHICS_HEADERS}
        ${GRAPHICS_SOURCES}
        
)

install(TARGETS Graphics EXPORT Graphics-targets
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME})
install(FILES ${GRAPHICS_HEADERS}
    DESTINATION include/${PROJECT_NAME}/graphics)