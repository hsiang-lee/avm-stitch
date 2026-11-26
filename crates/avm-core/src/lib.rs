//! Deliberately free of GPU, OpenCV, OSG and OS dependencies.

pub mod angle_utils;
pub mod bowl;
pub mod camera;
pub mod calibration;
pub mod depth;

pub use nalgebra::{Matrix3, Vector2, Vector3};
