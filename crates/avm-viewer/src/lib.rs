//! Interactive window for the stitch scene: a pure camera controller and a
//! winit/wgpu shell that draws the same scene as the offscreen renderer.

pub mod controller;
pub mod windowed;

pub use controller::OrbitController;
pub use windowed::{run_windowed, ViewerError, ViewerOptions, WINDOW_HEIGHT, WINDOW_WIDTH};
