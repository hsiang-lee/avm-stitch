//! N-camera fisheye AVM stitching.
//!
//! The algorithm modules (`camera`, `bowl`, `depth`, `calibration`) carry no GPU
//! or OS dependency. The `render` feature adds the wgpu offscreen path and the
//! `viewer` feature adds the interactive window on top of it.

pub mod angle_utils;
pub mod bowl;
pub mod calibration;
pub mod camera;
pub mod depth;

#[cfg(feature = "render")]
pub mod engine;
#[cfg(feature = "render")]
pub mod render;

pub use nalgebra::{Matrix3, Vector2, Vector3};
