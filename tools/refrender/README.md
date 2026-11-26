# refrender — C++ reference frames for the Rust parity test

Standalone program that renders `BowlParams::DefaultViewPoses()` through the
C++/OSG pipeline and writes one PNG per view. It is the ground-truth generator
for `crates/avm-render/tests/fixtures/parity/`; it links the C++ sources without
modifying them and will be deleted together with them in B4.

## Build

Needs the vcpkg tree the main CMake build already populated (OpenCV 4.12 with
`core imgproc imgcodecs calib3d`, OSG static with `osg osgViewer osgDB osgUtil`,
GTest, nlohmann_json). `$BUILD` is that CMake build directory.

```sh
BUILD=/path/to/avm-stitch/build
SRC=/path/to/avm-stitch                # repository root
c++ -O2 -DNDEBUG -std=gnu++20 \
    -I"$SRC/src" \
    -isystem "$BUILD/vcpkg_installed/x64-linux/include/opencv4" \
    -isystem "$BUILD/vcpkg_installed/x64-linux/include" \
    "$SRC/tools/refrender/main.cpp" \
    "$SRC"/src/bowl_model/{bowl_params,bowl_model,blend_weights}.cpp \
    "$SRC"/src/camera_model/{camera_model,camera,kb_camera_model,lut_camera_model,ocam_camera_model}.cpp \
    "$SRC"/src/render/{egl_headless_context,stitch_scene,osg_view_renderer}.cpp \
    -o refrender \
    $(sed -e 's|^/usr/bin/c++ -O3 -DNDEBUG ||' \
          -e 's|CMakeFiles/avm_viewer.dir/avm_viewer.cpp.o||' \
          -e 's|\.\./libavm_stitch.a||' \
          -e 's|-o avm_viewer||' \
          -e 's|-Wl,--dependency-file=[^ ]*||' \
          -e "s|\.\./vcpkg_installed|$BUILD/vcpkg_installed|g" \
          "$BUILD/examples/CMakeFiles/avm_viewer.dir/link.txt")
```

The link line is reused from the `avm_viewer` example target so the static-OSG
`--whole-archive` flags and the transitive libraries stay exactly as CMake
configures them. Linking the prebuilt `libavm_stitch.a` instead is not safe: that
archive can be stale relative to the checked-out headers and then crashes in
`BowlModel::BakeVertices`.

## Run

```sh
./refrender <data_dir> <out_dir>
# e.g. ./refrender examples/data_4 /tmp/parity-out
```

Reads `<data_dir>/calibration.json`, the images it names, and writes
`3d_front.png` / `3d_rear.png` / `3d_left.png` / `3d_right.png` (960x640, RGB8)
into `<out_dir>`. Rendering is deterministic on a fixed driver stack: identical
MD5 between runs. A different MD5 means the C++ code, the GPU driver or the
layers changed — re-check the fixtures before updating them.

## Output contract

| field | value |
| --- | --- |
| output size | 960x640 |
| views | `DefaultViewPoses()`, fovy 60 deg, `ClipPlanesFor(max eye-to-target)` |
| clear colour | `BackgroundClearColor()` = RGB(30, 36, 50) |
| row order | top-first (`RenderAllViews` flips the FBO readback) |
| format | RGB8 PNG, no alpha |
