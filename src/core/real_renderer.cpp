// RealRenderer implementation. Task 6d-A: the stub previously defined inline
// in real_renderer.hpp is moved here behind a pImpl so the header no longer
// depends on avm_renderer types. Behavior is unchanged (all methods return
// false). Task 6d-B replaces the AVMRENDERER_AVAILABLE branch with the real
// avm_renderer wrapper.
#include "core/real_renderer.hpp"

namespace avm_worker { namespace avm_stitch {

struct RealRenderer::Impl {};

RealRenderer::RealRenderer() : impl_(std::make_unique<Impl>()) {}
RealRenderer::~RealRenderer() = default;

bool RealRenderer::Init(const std::string&, unsigned int, int, int, int) {
    return false;
}
bool RealRenderer::UpdateFrame(const std::vector<const std::uint8_t*>&, int,
                               int, int) {
    return false;
}
bool RealRenderer::UpdateView(int) { return false; }
bool RealRenderer::Render() { return false; }
void RealRenderer::Destroy() {}

}} // namespace avm_worker::avm_stitch
