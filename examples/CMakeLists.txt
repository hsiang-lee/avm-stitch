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

# render_dump — 离线离屏渲染真实 stitch 任务到 PNG（诊断四块对齐）
add_executable(render_dump render_dump/render_dump.cpp)
target_include_directories(render_dump PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)
target_link_libraries(render_dump PRIVATE avm_stitch ${OpenCV_LIBS}
    unofficial::osg::osg unofficial::osg::osgViewer
    unofficial::osg::osgDB unofficial::osg::osgUtil)
target_osg_whole_archive(render_dump)
