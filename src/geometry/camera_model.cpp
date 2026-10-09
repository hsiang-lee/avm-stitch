#include "geometry/camera_model.hpp"

#include "geometry/kb_camera_model.hpp"

#include <stdexcept>
#include <utility>

namespace avm_stitch {

CameraModel::CameraModel(int width, int height, double cx, double cy)
    : width_(width), height_(height), cx_(cx), cy_(cy) {}

std::unique_ptr<CameraModel> MakeCameraModel(const std::string& model,
                                             const CameraModelParams& params) {
    if (model == "kb") {
        if (!std::holds_alternative<KbParams>(params)) {
            throw std::invalid_argument(
                "MakeCameraModel: params for model 'kb' must be KbParams");
        }
        const auto& intr = std::get<KbParams>(params).intr;
        if (intr.width <= 0 || intr.height <= 0) {
            throw std::invalid_argument(
                "MakeCameraModel: kb camera width/height must be > 0, got " +
                std::to_string(intr.width) + "x" + std::to_string(intr.height));
        }
        return std::make_unique<KbCameraModel>(intr);
    }
    // Task A1 placeholder: lut/ocam implementations land in batch B
    // (B1 -> "table", B3 -> "ocam"); not test-locked in A1.
    if (model == "table" || model == "ocam") {
        throw std::invalid_argument("MakeCameraModel: model '" + model +
                                    "' not supported yet");
    }
    throw std::invalid_argument("MakeCameraModel: unknown model '" + model + "'");
}

} // namespace avm_stitch
