#pragma once

#include "core/renderer_interface.hpp"

#include <nlohmann/json.hpp>

#include <memory>
#include <string>

namespace avm_worker { namespace avm_stitch {

/// Abstract interface: lets avm-worker's session manager inject a fake for
/// no-GL unit tests.
class StitchSessionInterface {
public:
    virtual ~StitchSessionInterface() = default;
    virtual void StartSession(const nlohmann::json& calibration,
                              const std::string& car_model_base64,
                              const std::string& task_base_dir) = 0;
    virtual nlohmann::json FeedFrame(const nlohmann::json& images) = 0;
    virtual void EndSession() = 0;
};

class StitchSession : public StitchSessionInterface {
public:
    /// @param renderer  avm-renderer wrapper (test injects mock; production
    ///                  injects RealRenderer). nullptr → degraded mode.
    explicit StitchSession(std::unique_ptr<RendererInterface> renderer);
    ~StitchSession() override;

    void StartSession(const nlohmann::json& calibration,
                      const std::string& car_model_base64,
                      const std::string& task_base_dir) override;
    nlohmann::json FeedFrame(const nlohmann::json& images) override;
    void EndSession() override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}} // namespace avm_worker::avm_stitch
