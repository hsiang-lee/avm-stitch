//! `num_cameras` is read from the uniform buffer in the fragment stage, so the
//! camera array needs only `texture_2d_array` plus an explicit sampler.

pub const SHADER_SOURCE: &str = include_str!("shader.wgsl");
