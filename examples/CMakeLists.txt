# avm_viewer — AVM stitch visualisation tool.
# Current stage: bowl-model viewer.

add_executable(avm_viewer
    avm_viewer.cpp
)
target_include_directories(avm_viewer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)
find_package(OpenCV REQUIRED COMPONENTS imgcodecs)
target_link_libraries(avm_viewer PRIVATE
    avm_stitch
    ${OpenCV_LIBS}
    unofficial::osg::osg unofficial::osg::osgViewer
    unofficial::osg::osgDB unofficial::osg::osgUtil
)
target_osg_whole_archive(avm_viewer)

# bowl_diag — TEMPORARY diagnostic tool (root-cause check, uncommitted).
# Pure geometry: only needs bowl_model + blend_weights (avm_stitch compiles them,
# but linking the full lib drags in OSG/EGL; compile the two sources directly
# instead, mirroring how test_bowl_model links).
add_executable(bowl_diag
    bowl_diag.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/geometry/bowl_model.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/geometry/blend_weights.cpp
)
target_include_directories(bowl_diag PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)
target_link_libraries(bowl_diag PRIVATE ${OpenCV_LIBS})
