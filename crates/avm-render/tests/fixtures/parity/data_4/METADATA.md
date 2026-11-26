# C++ OSG reference frames (`data_4`)

Ground truth for `crates/avm-render/tests/parity.rs`.

| field | value |
| --- | --- |
| C++ sources | commit `6baf46a` of this repository, unmodified |
| build | Release-like: `-O2 -DNDEBUG -std=gnu++20`, compiled into a standalone program outside the repository against the existing vcpkg tree (OpenCV 4.12, OSG static) |
| input | `examples/data_4/calibration.json` and its four 640x480 PNGs |
| views | `BowlParams::DefaultViewPoses()` — `3d_front`, `3d_rear`, `3d_left`, `3d_right`, fovy 60 deg, `ClipPlanesFor(max eye-to-target)` |
| output | 960x640 RGBA8 PNG, one per view |
| clear colour | `BackgroundClearColor()` = RGB(30, 36, 50), alpha 1.0 |
| pipeline | `BowlModel(bowl, cameras)` -> `StitchScene` -> `UploadCameraImages` -> `OSGViewRenderer(..., 960, 640, RenderMode::Offscreen)::RenderAllViews()` -> `cv::imwrite` |

`RenderAllViews` returns the FBO readback flipped to top-first, so row 0 of each
PNG is the top of the rendered frame, matching the Rust readback.

Regenerating: build the same program from the C++ sources at `6baf46a` and run it
as `<program> examples/data_4 <out_dir>`. Rendering is deterministic (identical
MD5 on repeat runs), so a byte difference means the C++ code or the GPU stack
changed.
