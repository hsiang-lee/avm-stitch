#include "core/view_mapping.hpp"

namespace avm_worker { namespace avm_stitch {

int MapViewNameToDisplayMode(const std::string& view_name) {
    if (view_name == "topdown")  return kTopDownAndAerial;
    if (view_name == "3d_front") return kD3FrontAndAerial;
    if (view_name == "3d_rear")  return kD3RearAndAerial;
    if (view_name == "3d_left")  return kD3LeftAndAerial;
    if (view_name == "3d_right") return kD3RightAndAerial;
    return kInvalidDisplayMode;
}

}} // namespace avm_worker::avm_stitch
