# Reference frames (`data_4`)

Ground truth for `tests/parity.rs`. The frames were produced by
the C++/OSG implementation that has since been removed from the tree; the driver
tool can be recovered from tag `archive/cpp-implementation` (`tools/refrender/`)
if they ever need to be regenerated. The four fixture MD5s below stay as a
tamper check.

| field | value |
| --- | --- |
| reference sources | tag `archive/cpp-implementation`, `tools/refrender/` |
| build | Release-like (`-O2 -DNDEBUG -std=gnu++20`), compiled by the reference driver |
| input | `examples/data_4/calibration.json` and its four 640x480 PNGs |
| views | `BowlParams::DefaultViewPoses()` — `3d_front`, `3d_rear`, `3d_left`, `3d_right`, fovy 60 deg, `ClipPlanesFor(max eye-to-target)` |
| output | 960x640 RGB8 PNG (no alpha), one per view, top-first |
| clear colour | `BackgroundClearColor()` = RGB(30, 36, 50), alpha 1.0 |
| pipeline | `BowlModel` -> `StitchScene` -> `UploadCameraImages` -> `OSGViewRenderer(..., 960, 640, RenderMode::Offscreen)::RenderAllViews()` -> `cv::imwrite` |

## Driver stack

| side | API | device | driver |
| --- | --- | --- | --- |
| C++ reference | OpenGL via EGL surfaceless (`EGL_PLATFORM_SURFACELESS_MESA`, GLVND vendor `50_mesa.json` -> `libEGL_mesa.so.0`) | Intel(R) Graphics (RPL-P) | Mesa 26.0.8-1ubuntu0.3 |
| Rust test | wgpu -> Vulkan | Intel(R) Graphics (RPL-P) | Mesa 26.0.8-1ubuntu0.3 (Intel open-source Mesa driver) |

The two sides share the GPU and the Mesa release but use different APIs, so the
PARITY test compares rasterisations, not two runs of the same driver.

## Checksums

```
201c8d162c2fbfdb7c148f66e14a5eab  3d_front.png
ca271e426b9956bfcb56f02a58715664  3d_rear.png
46674da0ba90db0036bbc1d6732e83a7  3d_left.png
bab64aa36cceff8fc9de6456b8ca43e1  3d_right.png
```

`tools/refrender` is deterministic: two runs on this stack produced identical
MD5s. `data_4` frames are fully covered by the bowl, so the clear colour is not
observable in them.
