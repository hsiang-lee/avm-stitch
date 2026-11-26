//! Bowl layer: parameters, blend weights and the baked mesh.
//!
//! Port of `src/bowl_model/*`.

pub mod blend;
pub mod model;
pub mod params;

pub use blend::BlendWeightsCalculator;
pub use model::{BakedVertex, BowlModel};
pub use params::{
    BowlParams, ClipPlanes, Mesh, ViewPose, CALIBRATED_ANCHOR_OUTER_RADIUS,
};
