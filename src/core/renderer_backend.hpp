#pragma once

#include "core/stitch_backend.hpp"
#include "core/renderer_interface.hpp"

#include <memory>
#include <nlohmann/json.hpp>

namespace avm_worker { namespace avm_stitch {

/// In-process AVM render backend using avm-renderer library.
/// Owns a RendererInterface; when the interface is nullptr (avm-renderer
/// not available at build time), Run() returns a stitch_error immediately.
class RendererBackend : public StitchBackend {
public:
    /// @param renderer  avm-renderer wrapper (nullptr → degraded mode)
    explicit RendererBackend(std::unique_ptr<RendererInterface> renderer);
    ~RendererBackend() override;

    nlohmann::json Run(const nlohmann::json& task) override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}} // namespace avm_worker::avm_stitch
