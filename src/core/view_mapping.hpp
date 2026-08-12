#pragma once

#include <string>

namespace avm_worker { namespace avm_stitch {

// ── AVMDisplayMode mapping constants ──
// Values mirror avm-renderer's AVMDisplayMode enum (AvmConstants.h).
// We intentionally use plain int constants instead of including the
// avm-renderer header, keeping avm-stitch decoupled from the renderer
// dependency. RealRenderer performs the final enum conversion.
//
// NOTE: Design doc 5.3 lists "topdown → TOP_DOWN_AND_AERIAL" but
// avm-renderer 0.1.15 has no such enumerator; the closest semantic
// (top-down + aerial side-by-side) is D3_TOP_AND_AERIAL. Verify against
// AvmConstants.h when bumping avm-renderer.

constexpr int kTopDownAndAerial   = 20; // D3_TOP_AND_AERIAL
constexpr int kD3FrontAndAerial   = 16; // D3_FRONT_AND_AERIAL
constexpr int kD3RearAndAerial    = 12; // D3_REAR_AND_AERIAL
constexpr int kD3LeftAndAerial    = 14; // D3_LEFT_AND_AERIAL
constexpr int kD3RightAndAerial   = 18; // D3_RIGHT_AND_AERIAL
constexpr int kInvalidDisplayMode = -1;

/// Map task view name string to AVMDisplayMode int value.
/// Valid inputs: "topdown", "3d_front", "3d_rear", "3d_left", "3d_right"
/// Unknown/unrecognized names return kInvalidDisplayMode (-1).
int MapViewNameToDisplayMode(const std::string& view_name);

/// The standard 5 view names in the order they should be rendered.
inline const char* kRenderViewNames[] = {
    "topdown", "3d_front", "3d_rear", "3d_left", "3d_right"
};
constexpr int kRenderViewCount = 5;

}} // namespace avm_worker::avm_stitch
