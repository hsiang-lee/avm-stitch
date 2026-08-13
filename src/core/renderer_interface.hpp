#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace avm_worker { namespace avm_stitch {

/// Thin abstraction over avm-renderer's C API, enabling mock injection for
/// Layer 1 unit tests. RealRenderer wraps the actual C calls.
class RendererInterface {
public:
    virtual ~RendererInterface() = default;

    /// Initialize avm-renderer with calibration directory, output FBO, and
    /// output resolution. Must be called once before any rendering.
    /// @param calib_dir_path  path to the directory containing model .bin files
    /// @param output_fbo      OpenGL FBO ID for off-screen rendering
    /// @param output_width    output image width (pixels)
    /// @param output_height   output image height (pixels)
    /// @param flags           initialization flags (e.g. FLAG_WINDOW_HIDDEN)
    /// @return true on success
    virtual bool Init(const std::string& calib_dir_path,
                      unsigned int output_fbo,
                      int output_width, int output_height,
                      int flags) = 0;

    /// Update the input frame (4 camera images in left/right/front/rear order).
    /// @param image_buffers 4 raw RGBA8888 pixel buffers (non-owning pointers)
    /// @param width         image width in pixels
    /// @param height        image height in pixels
    /// @param pixel_format  pixel format enum (avm-renderer RGBA8888 = 0)
    /// @return true on success
    virtual bool UpdateFrame(const std::vector<const std::uint8_t*>& image_buffers,
                             int width, int height, int pixel_format) = 0;

    /// Set the current display mode (view) for the next Render() call.
    /// @param display_mode AVMDisplayMode enum value
    /// @return true on success
    virtual bool UpdateView(int display_mode) = 0;

    /// Execute one frame of rendering to the output FBO.
    /// @return true on success
    virtual bool Render() = 0;

    /// Release avm-renderer resources. Safe to call multiple times.
    virtual void Destroy() = 0;
};

}} // namespace avm_worker::avm_stitch
