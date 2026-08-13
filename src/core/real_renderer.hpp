#pragma once

#include "core/renderer_interface.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace avm_worker { namespace avm_stitch {

/// Real avm-renderer wrapper (implemented in real_renderer.cpp). The
/// avm_renderer types stay behind a pImpl so this header compiles even when
/// AVMRENDERER_AVAILABLE is off.
class RealRenderer : public RendererInterface {
public:
    RealRenderer();
    ~RealRenderer() override;
    RealRenderer(const RealRenderer&) = delete;
    RealRenderer& operator=(const RealRenderer&) = delete;

    /// Initialize avm-renderer. Precondition: the caller must have a current
    /// GLES3 context (the EGL context is owned and managed by RendererBackend).
    bool Init(const std::string&, unsigned int, int, int, int) override;
    bool UpdateFrame(const std::vector<const std::uint8_t*>&, int, int, int)
        override;
    bool UpdateView(int) override;
    bool Render() override;
    void Destroy() override;   // does not destroy the EGL context (not owned)

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}} // namespace avm_worker::avm_stitch
