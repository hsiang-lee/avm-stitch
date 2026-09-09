cmake_minimum_required(VERSION 3.25)
project(avm-stitch VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d)
find_package(unofficial-osg CONFIG REQUIRED)
find_package(OpenGL REQUIRED COMPONENTS EGL)
find_package(GTest CONFIG REQUIRED)

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
    src/geometry/bowl_model.cpp
    src/geometry/blend_weights.cpp
    src/render/egl_headless_context.cpp
    src/geometry/mesh_bake.cpp
    src/render/osg_scene.cpp
    src/render/osg_view.cpp
    src/core/stitch_engine.cpp
)
# PUBLIC include: src/ 提供 geometry/ render/ core/ util/
# NOTE: avm-worker's src/ mirrors these core/|geometry/|util/ paths (mutually
# exclusive file sets by convention) — do not create same-name headers on
# both sides.
target_include_directories(avm_stitch PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(avm_stitch PUBLIC
    ${OpenCV_LIBS}
    unofficial::osg::osg unofficial::osg::osgViewer unofficial::osg::osgDB unofficial::osg::osgUtil
    OpenGL::EGL
)
# Same whole-archive requirement: vcpkg osg is a static build; without
# --whole-archive the X11 WSI static-registration object is not pulled in.
target_osg_whole_archive(avm_stitch)

enable_testing()
include(GoogleTest)

# -- test_bowl_model (bowl_model.hpp + bowl_model.cpp)
add_executable(test_bowl_model
    tests/geometry/test_bowl_model.cpp
    src/geometry/bowl_model.cpp
)
target_include_directories(test_bowl_model PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(test_bowl_model PRIVATE
    GTest::gtest_main
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
    src/geometry/bowl_model.cpp
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
    src/geometry/bowl_model.cpp
    src/geometry/blend_weights.cpp
    src/geometry/mesh_bake.cpp
    src/render/osg_scene.cpp
)
target_include_directories(test_osg_scene PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(test_osg_scene PRIVATE
    GTest::gtest_main
    ${OpenCV_LIBS}
    unofficial::osg::osg unofficial::osg::osgViewer unofficial::osg::osgDB unofficial::osg::osgUtil
)
gtest_discover_tests(test_osg_scene)

# -- test_stitch_engine (OSGViewRenderer + StitchEngine; GL-gated rendering) --
add_executable(test_stitch_engine
    tests/core/test_stitch_engine.cpp
    src/geometry/bowl_model.cpp
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
