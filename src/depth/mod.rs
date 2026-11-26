//! Spherical rectification and the analytic ground-depth map. Pure geometry over `CameraExtrinsics`; no image access and no OpenCV.

mod ground_depth;
mod rectification;

pub use ground_depth::{
    compute_ground_depth_map, triangulate_depth, GroundDepthMap, GroundDepthOptions,
};
pub use rectification::{
    camera_direction, make_rectification_basis, phi_deg, rectified_direction,
    sky_up_needs_mirror, CameraSide, RectificationBasis, SphericalGrid,
};
