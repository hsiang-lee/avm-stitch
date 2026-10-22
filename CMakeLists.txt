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
    src/bowl_model/bowl_params.cpp
    src/bowl_model/bowl_model.cpp
    src/bowl_model/blend_weights.cpp
    src/camera_model/camera_model.cpp
    src/camera_model/camera.cpp
    src/camera_model/kb_camera_model.cpp
    src/camera_model/lut_camera_model.cpp
    src/camera_model/ocam_camera_model.cpp
    src/depth/spherical_rectification.cpp
    src/depth/ground_depth.cpp
    src/render/egl_headless_context.cpp
    src/render/stitch_scene.cpp
    src/render/osg_view_renderer.cpp
    src/core/stitch_engine.cpp
)
# PUBLIC include: src/ 提供 camera_model/ bowl_model/ render/ core/ depth/
# NOTE: avm-worker's src/ mirrors these camera_model/|bowl_model/|util/ paths
# (mutually exclusive file sets by convention) — do not create same-name
# headers on both sides.
# camera_model/|bowl_model/ 已平铺在 src/ 顶层（不再有 geometry/ 中间层）。
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

# -- examples --
add_subdirectory(examples)

enable_testing()
include(GoogleTest)

# -- test_bowl_model (bowl_model.hpp + bowl_model.cpp, incl. Bake)
add_executable(test_bowl_model
    tests/geometry/test_bowl_model.cpp
    src/camera_model/camera_model.cpp
    src/camera_model/camera.cpp
    src/camera_model/kb_camera_model.cpp
    src/camera_model/lut_camera_model.cpp
    src/camera_model/ocam_camera_model.cpp
    src/bowl_model/bowl_params.cpp
    src/bowl_model/bowl_model.cpp
    src/bowl_model/blend_weights.cpp
)
target_include_directories(test_bowl_model PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_link_libraries(test_bowl_model PRIVATE
    GTest::gtest_main
    ${OpenCV_LIBS}
)
gtest_discover_tests(test_bowl_model)

# -- test_bowl_params (bowl_params.hpp + bowl_params.cpp; pure math) --
# Links the bowl_model sources too: GenerateBlockBoatMesh's bit-parity guard
# constructs a BowlModel to prove the constructor path is the same generator.
add_executable(test_bowl_params
    tests/geometry/test_bowl_params.cpp
    src/camera_model/camera_model.cpp
    src/camera_model/camera.cpp
    src/camera_model/kb_camera_model.cpp
    src/camera_model/lut_camera_model.cpp
    src/camera_model/ocam_camera_model.cpp
    src/bowl_model/bowl_params.cpp
    src/bowl_model/bowl_model.cpp
    src/bowl_model/blend_weights.cpp
)
target_include_directories(test_bowl_params PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_link_libraries(test_bowl_params PRIVATE
    GTest::gtest_main
    ${OpenCV_LIBS}
)
gtest_discover_tests(test_bowl_params)

# -- test_projection (CameraExtrinsics + KbCameraModel projection/gating) --
# No longer header-only: instantiates KbCameraModel + the out-of-line base
# CameraModel surface, so it compiles the camera_model sources like the other
# model suites.
add_executable(test_projection
    tests/geometry/test_projection.cpp
    src/camera_model/camera_model.cpp
    src/camera_model/camera.cpp
    src/camera_model/kb_camera_model.cpp
    src/camera_model/lut_camera_model.cpp
    src/camera_model/ocam_camera_model.cpp
)
target_include_directories(test_projection PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(test_projection PRIVATE
    GTest::gtest_main
    ${OpenCV_LIBS}
)
gtest_discover_tests(test_projection)

# -- test_camera_model (CameraModel abstraction + factory + kb implementation) --
add_executable(test_camera_model
    tests/geometry/test_camera_model.cpp
    src/camera_model/camera_model.cpp
    src/camera_model/camera.cpp
    src/camera_model/kb_camera_model.cpp
    src/camera_model/lut_camera_model.cpp
    src/camera_model/ocam_camera_model.cpp
)
target_include_directories(test_camera_model PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(test_camera_model PRIVATE
    GTest::gtest_main
    ${OpenCV_LIBS}
)
gtest_discover_tests(test_camera_model)

# -- test_lut_camera_model (LutCameraModel: OQ-1 direction pin + LUT projection) --
add_executable(test_lut_camera_model
    tests/geometry/test_lut_camera_model.cpp
    src/camera_model/camera_model.cpp
    src/camera_model/camera.cpp
    src/camera_model/kb_camera_model.cpp
    src/camera_model/lut_camera_model.cpp
    src/camera_model/ocam_camera_model.cpp
)
target_include_directories(test_lut_camera_model PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(test_lut_camera_model PRIVATE
    GTest::gtest_main
    ${OpenCV_LIBS}
)
gtest_discover_tests(test_lut_camera_model)

# -- test_ocam_camera_model (OcamCameraModel: forward projection + round trip) --
add_executable(test_ocam_camera_model
    tests/geometry/test_ocam_camera_model.cpp
    src/camera_model/camera_model.cpp
    src/camera_model/camera.cpp
    src/camera_model/kb_camera_model.cpp
    src/camera_model/lut_camera_model.cpp
    src/camera_model/ocam_camera_model.cpp
)
target_include_directories(test_ocam_camera_model PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(test_ocam_camera_model PRIVATE
    GTest::gtest_main
    ${OpenCV_LIBS}
)
gtest_discover_tests(test_ocam_camera_model)

# -- test_blend_weights
add_executable(test_blend_weights
    tests/geometry/test_blend_weights.cpp
    src/bowl_model/blend_weights.cpp
)
target_include_directories(test_blend_weights PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_link_libraries(test_blend_weights PRIVATE
    GTest::gtest_main
    ${OpenCV_LIBS}
)
gtest_discover_tests(test_blend_weights)


# -- test_stitch_scene (OSG scene graph construction; structural, no GL) --
add_executable(test_stitch_scene
    tests/render/test_stitch_scene.cpp
    src/bowl_model/bowl_params.cpp
    src/bowl_model/bowl_model.cpp
    src/bowl_model/blend_weights.cpp
    src/camera_model/camera_model.cpp
    src/camera_model/camera.cpp
    src/camera_model/kb_camera_model.cpp
    src/camera_model/lut_camera_model.cpp
    src/camera_model/ocam_camera_model.cpp
    src/render/stitch_scene.cpp
)
target_include_directories(test_stitch_scene PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_link_libraries(test_stitch_scene PRIVATE
    GTest::gtest_main
    ${OpenCV_LIBS}
    unofficial::osg::osg unofficial::osg::osgViewer unofficial::osg::osgDB unofficial::osg::osgUtil
)
gtest_discover_tests(test_stitch_scene)

# -- test_stitch_engine (OSGViewRenderer + StitchEngine; GL-gated rendering) --
add_executable(test_stitch_engine
    tests/core/test_stitch_engine.cpp
    src/camera_model/camera_model.cpp
    src/camera_model/camera.cpp
    src/camera_model/kb_camera_model.cpp
    src/camera_model/lut_camera_model.cpp
    src/camera_model/ocam_camera_model.cpp
    src/bowl_model/bowl_params.cpp
    src/bowl_model/bowl_model.cpp
    src/bowl_model/blend_weights.cpp
    src/render/egl_headless_context.cpp
    src/render/stitch_scene.cpp
    src/render/osg_view_renderer.cpp
    src/core/stitch_engine.cpp
)
target_include_directories(test_stitch_engine PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_link_libraries(test_stitch_engine PRIVATE
    GTest::gtest_main
    ${OpenCV_LIBS}
    unofficial::osg::osg unofficial::osg::osgViewer unofficial::osg::osgDB unofficial::osg::osgUtil
    OpenGL::EGL OpenGL::OpenGL
)
target_osg_whole_archive(test_stitch_engine)
gtest_discover_tests(test_stitch_engine)

# -- test_osg_view_renderer (OSGViewRenderer; GL-gated rendering) --
add_executable(test_osg_view_renderer
    tests/render/test_osg_view_renderer.cpp
    src/bowl_model/bowl_params.cpp
    src/bowl_model/bowl_model.cpp
    src/bowl_model/blend_weights.cpp
    src/camera_model/camera_model.cpp
    src/camera_model/camera.cpp
    src/camera_model/kb_camera_model.cpp
    src/camera_model/lut_camera_model.cpp
    src/camera_model/ocam_camera_model.cpp
    src/render/egl_headless_context.cpp
    src/render/stitch_scene.cpp
    src/render/osg_view_renderer.cpp
)
target_include_directories(test_osg_view_renderer PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/tests
)
target_link_libraries(test_osg_view_renderer PRIVATE
    GTest::gtest_main
    ${OpenCV_LIBS}
    unofficial::osg::osg unofficial::osg::osgViewer unofficial::osg::osgDB unofficial::osg::osgUtil
    OpenGL::EGL OpenGL::OpenGL
)
target_osg_whole_archive(test_osg_view_renderer)
gtest_discover_tests(test_osg_view_renderer)

# -- test_egl_headless_context (headless EGL surfaceless GL context) --
add_executable(test_egl_headless_context
    tests/render/test_egl_headless_context.cpp
    src/render/egl_headless_context.cpp
)
target_include_directories(test_egl_headless_context PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/tests)
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
target_include_directories(test_stitch_engine_regression PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_link_libraries(test_stitch_engine_regression PRIVATE
    GTest::gtest_main
    avm_stitch
)
gtest_discover_tests(test_stitch_engine_regression)

# -- test_depth (spherical rectification + analytic ground depth; pure math) --
add_executable(test_depth
    tests/depth/test_spherical_rectification.cpp
    tests/depth/test_ground_depth.cpp
    src/depth/spherical_rectification.cpp
    src/depth/ground_depth.cpp
)
target_include_directories(test_depth PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_link_libraries(test_depth PRIVATE
    GTest::gtest_main
    ${OpenCV_LIBS}
)
gtest_discover_tests(test_depth)
