//! WGSL port of the GLSL 330 shader pair in `src/render/stitch_scene.cpp`.
//!
//! Differences from the GLSL original, both behaviour-preserving:
//! * `uNumCameras` is read straight from the uniform buffer in the fragment
//!   stage instead of being passed through a `flat out` varying.
//! * `sampler2DArray` becomes `texture_2d_array` + an explicit sampler.

/// The single shader module used by the offscreen pipeline.
pub const SHADER_SOURCE: &str = include_str!("shader.wgsl");
