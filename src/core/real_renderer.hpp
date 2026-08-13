#pragma once

#include "core/renderer_interface.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace avm_worker { namespace avm_stitch {

/// Real avm-renderer wrapper (implemented in Task 6). This stub compiles
/// without the avm-renderer headers so the source list stays buildable even
/// when AVMRENDERER_AVAILABLE is off.
class RealRenderer : public RendererInterface {
public:
    bool Init(const std::string&, unsigned int, int, int, int) override {
        return false;
    }
    bool UpdateFrame(const std::vector<const std::uint8_t*>&, int, int, int)
        override {
        return false;
    }
    bool UpdateView(int) override { return false; }
    bool Render() override { return false; }
    void Destroy() override {}
};

}} // namespace avm_worker::avm_stitch
