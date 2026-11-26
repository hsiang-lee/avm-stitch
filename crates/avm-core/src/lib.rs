//! CPU core of the avm-stitch Rust port.
//!
//! This crate is deliberately free of GPU, OpenCV, OSG and OS dependencies:
//! it holds the numerically sensitive algorithm code that the C++ implementation
//! in `src/camera_model` and `src/bowl_model` defines.

pub mod angle_utils;
pub mod bowl;
pub mod camera;
pub mod calibration;

pub use nalgebra::{Matrix3, Vector2, Vector3};
