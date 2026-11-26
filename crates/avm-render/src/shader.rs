//! `num_cameras` is read from the uniform buffer in the fragment stage (no `flat`
//! varying), and the camera array is `texture_2d_array` plus an explicit sampler
//! rather than a combined `sampler2DArray`.

pub const SHADER_SOURCE: &str = include_str!("shader.wgsl");
