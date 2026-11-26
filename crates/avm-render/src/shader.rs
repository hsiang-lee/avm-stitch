//! Differences from the GLSL 330 original, both behaviour-preserving:
//! * `uNumCameras` is read from the uniform buffer in the fragment stage
//!   instead of a `flat out` varying.
//! * `sampler2DArray` becomes `texture_2d_array` + an explicit sampler.

pub const SHADER_SOURCE: &str = include_str!("shader.wgsl");
