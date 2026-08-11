#pragma once

#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>

namespace avm_worker {

inline nlohmann::json ComputeStitchMetrics() {
  // R2: All stub values
  double ssim = 0.92;
  double psnr = 38.5;
  double stitch_error = 0.23;
  double edge_distortion = 0.31;
  double brightness_consistency = 92.5;
  double structure_continuity = 0.89;

  return {
    {"ssim", ssim},
    {"psnr", psnr},
    {"stitch_error", stitch_error},
    {"edge_distortion", edge_distortion},
    {"brightness_consistency", brightness_consistency},
    {"structure_continuity", structure_continuity},
  };
}

inline double ComputeOverallScore(const nlohmann::json& metrics) {
  double ssim = metrics.value("ssim", 0.0);
  double psnr = metrics.value("psnr", 0.0);
  double stitch_error = metrics.value("stitch_error", 0.0);
  double edge_distortion = metrics.value("edge_distortion", 0.0);
  double brightness_consistency = metrics.value("brightness_consistency", 0.0);
  double structure_continuity = metrics.value("structure_continuity", 0.0);

  // overall = (0.20*ssim + 0.15*(psnr/50) + 0.20*(1 - clamp(stitch_error/5,0,1))
  //           + 0.15*(1 - clamp(edge_distortion/5,0,1)) + 0.15*(brightness_consistency/100)
  //           + 0.15*structure_continuity) * 100
  auto clamp = [](double v) { return std::max(0.0, std::min(1.0, v)); };

  double score =
      0.20 * ssim +
      0.15 * (psnr / 50.0) +
      0.20 * (1.0 - clamp(stitch_error / 5.0)) +
      0.15 * (1.0 - clamp(edge_distortion / 5.0)) +
      0.15 * (brightness_consistency / 100.0) +
      0.15 * structure_continuity;

  return score * 100.0;
}

}  // namespace avm_worker
