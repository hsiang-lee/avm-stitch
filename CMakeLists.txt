cmake_minimum_required(VERSION 3.25)
project(avm-stitch VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(nlohmann_json CONFIG REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d)
find_package(unofficial-osg CONFIG REQUIRED)
find_package(OpenGL REQUIRED COMPONENTS EGL GLES3)
find_package(GTest CONFIG REQUIRED)

# -- avm-renderer (private registry, optional for renderer backend) --
# avm_renderer's exported targets reference CLI11::CLI11 in their link
# interface but the config file does not find_dependency(CLI11); find it
# explicitly so the imported target exists when avm_rendererTargets loads.
find_package(CLI11 CONFIG REQUIRED)
# avm_renderer's exported targets reference a bare `glfw` target in their
# link interface but the config file does not find_dependency(glfw3); find
# it explicitly so the imported target exists and -lglfw is not emitted.
find_package(glfw3 CONFIG REQUIRED)
find_package(avm_renderer CONFIG)
set(AVMRENDERER_AVAILABLE OFF)
if(avm_renderer_FOUND)
    set(AVMRENDERER_AVAILABLE ON)
    message(STATUS "avm-renderer found: renderer backend enabled")
else()
    message(STATUS "avm-renderer not found: renderer backend will return errors in production")
endif()

# -- OSG static libs need --whole-archive (X11 WSI static registration) --
function(target_osg_whole_archive target)
    set(_osg_libs osg osgViewer osgDB osgUtil)
    set(_wa "")
    foreach(_lib IN LISTS _osg_libs)
        get_target_property(_loc unofficial::osg::${_lib} IMPORTED_LOCATION_RELEASE)
        if(_loc)
            list(APPEND _wa "-Wl,--whole-archive,${_loc}")
        endif()
    endforeach()
    if(_wa)
        list(APPEND _wa "-Wl,--no-whole-archive")
        target_link_options(${target} PRIVATE ${_wa})
    endif()
endfunction()

# -- avm_stitch static library --
add_library(avm_stitch STATIC
    src/geometry/blend_weights.cpp
    src/render/egl_headless_context.cpp
    src/geometry/mesh_bake.cpp
    src/render/osg_scene.cpp
    src/render/osg_view.cpp
    src/core/stitch_engine.cpp
    src/core/stitch_backend.cpp
    src/core/osg_backend.cpp
    src/core/subprocess_backend.cpp
    src/core/stitch_result.cpp
    src/core/renderer_backend.cpp
    src/core/real_renderer.cpp
    src/core/calib_files_generator.cpp
    src/common/image_converter.cpp
    src/core/view_mapping.cpp
    src/render/fbo_manager.cpp
)
# PUBLIC include: src/ 提供 geometry/ render/ core/ util/ common/ eval_metrics/
target_include_directories(avm_stitch PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(avm_stitch PUBLIC
    nlohmann_json::nlohmann_json
    ${OpenCV_LIBS}
    unofficial::osg::osg unofficial::osg::osgViewer unofficial::osg::osgDB unofficial::osg::osgUtil
    OpenGL::EGL OpenGL::OpenGL
)
# Same whole-archive requirement: vcpkg osg is a static build; without
# --whole-archive the X11 WSI static-registration object is not pulled in.
target_osg_whole_archive(avm_stitch)

# -- avm-renderer (private registry, optional for renderer backend) --
if(AVMRENDERER_AVAILABLE)
    target_link_libraries(avm_stitch PUBLIC
        avm_renderer::avm_renderer_lib
        avm_renderer::avm_fbs
    )
    target_compile_definitions(avm_stitch PUBLIC AVMRENDERER_AVAILABLE)
endif()

enable_testing()
include(GoogleTest)

# -- test_bowl_model (header-only bowl_model.hpp)
add_executable(test_bowl_model
    tests/geometry/test_bowl_model.cpp
)
target_include_directories(test_bowl_model PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(test_bowl_model PRIVATE
    GTest::gtest_main
    nlohmann_json::nlohmann_json
    ${OpenCV_LIBS}
)
gtest_discover_tests(test_bowl_model)

# -- test_projection (header-only projection.hpp)
add_executable(test_projection
    tests/geometry/test_projection.cpp
)
target_include_directories(test_projection PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(test_projection PRIVATE
    GTest::gtest_main
    ${OpenCV_LIBS}
)
gtest_discover_tests(test_projection)

# -- test_blend_weights
add_executable(test_blend_weights
    tests/geometry/test_blend_weights.cpp
    src/geometry/blend_weights.cpp
)
target_include_directories(test_blend_weights PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(test_blend_weights PRIVATE
    GTest::gtest_main
    ${OpenCV_LIBS}
)
gtest_discover_tests(test_blend_weights)

# -- test_mesh_bake
add_executable(test_mesh_bake
    tests/geometry/test_mesh_bake.cpp
    src/geometry/blend_weights.cpp
    src/geometry/mesh_bake.cpp
)
target_include_directories(test_mesh_bake PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(test_mesh_bake PRIVATE
    GTest::gtest_main
    ${OpenCV_LIBS}
)
gtest_discover_tests(test_mesh_bake)

# -- test_view_config (header-only view_config.hpp)
add_executable(test_view_config
    tests/util/test_view_config.cpp
)
target_include_directories(test_view_config PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(test_view_config PRIVATE
    GTest::gtest_main
    ${OpenCV_LIBS}
)
gtest_discover_tests(test_view_config)

# -- test_osg_scene (OSG scene graph construction; structural, no GL) --
add_executable(test_osg_scene
    tests/render/test_osg_scene.cpp
    src/geometry/blend_weights.cpp
    src/geometry/mesh_bake.cpp
    src/render/osg_scene.cpp
)
target_include_directories(test_osg_scene PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(test_osg_scene PRIVATE
    GTest::gtest_main
    nlohmann_json::nlohmann_json
    ${OpenCV_LIBS}
    unofficial::osg::osg unofficial::osg::osgViewer unofficial::osg::osgDB unofficial::osg::osgUtil
)
gtest_discover_tests(test_osg_scene)

# -- test_stitch_engine (OSGViewRenderer + StitchEngine; GL-gated rendering) --
add_executable(test_stitch_engine
    tests/core/test_stitch_engine.cpp
    src/geometry/blend_weights.cpp
    src/render/egl_headless_context.cpp
    src/geometry/mesh_bake.cpp
    src/render/osg_scene.cpp
    src/render/osg_view.cpp
    src/core/stitch_engine.cpp
)
target_include_directories(test_stitch_engine PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(test_stitch_engine PRIVATE
    GTest::gtest_main
    nlohmann_json::nlohmann_json
    ${OpenCV_LIBS}
    unofficial::osg::osg unofficial::osg::osgViewer unofficial::osg::osgDB unofficial::osg::osgUtil
    OpenGL::EGL OpenGL::OpenGL
)
target_osg_whole_archive(test_stitch_engine)
gtest_discover_tests(test_stitch_engine)

# -- test_egl_headless_context (headless EGL surfaceless GL context) --
add_executable(test_egl_headless_context
    tests/render/test_egl_headless_context.cpp
    src/render/egl_headless_context.cpp
)
target_include_directories(test_egl_headless_context PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(test_egl_headless_context PRIVATE
    GTest::gtest_main
    ${OpenCV_LIBS}
    unofficial::osg::osg unofficial::osg::osgViewer unofficial::osg::osgDB unofficial::osg::osgUtil
    OpenGL::EGL OpenGL::OpenGL
)
target_osg_whole_archive(test_egl_headless_context)
gtest_discover_tests(test_egl_headless_context)

# -- test_stitch_engine_regression (public-API smoke tests, links avm_stitch) --
add_executable(test_stitch_engine_regression
    tests/core/test_stitch_engine_regression.cpp
)
target_include_directories(test_stitch_engine_regression PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(test_stitch_engine_regression PRIVATE
    GTest::gtest_main
    avm_stitch
)
gtest_discover_tests(test_stitch_engine_regression)

# -- test_fbo_manager (GL-gated: requires EGL surfaceless context) --
add_executable(test_fbo_manager
    tests/render/test_fbo_manager.cpp
    src/render/fbo_manager.cpp
    src/render/egl_headless_context.cpp
)
target_include_directories(test_fbo_manager PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(test_fbo_manager PRIVATE
    GTest::gtest_main
    ${OpenCV_LIBS}
    unofficial::osg::osg unofficial::osg::osgViewer unofficial::osg::osgDB unofficial::osg::osgUtil
    OpenGL::EGL OpenGL::OpenGL
)
target_osg_whole_archive(test_fbo_manager)
gtest_discover_tests(test_fbo_manager)

# -- test_egl_gles3_context (standalone EGL surfaceless GLES3 context) --
add_executable(test_egl_gles3_context
    tests/render/test_egl_gles3_context.cpp
)
target_include_directories(test_egl_gles3_context PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(test_egl_gles3_context PRIVATE
    GTest::gtest_main
    OpenGL::EGL OpenGL::GLES3
)
gtest_discover_tests(test_egl_gles3_context)

# -- test_view_mapping (view name → AVMDisplayMode int, no external deps) --
add_executable(test_view_mapping
    tests/core/test_view_mapping.cpp
    src/core/view_mapping.cpp
)
target_include_directories(test_view_mapping PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(test_view_mapping PRIVATE
    GTest::gtest_main
)
gtest_discover_tests(test_view_mapping)
