//! Interactive window for the stitch scene: pure camera controller plus the
//! winit/wgpu shell.

pub mod controller;
pub mod windowed;

pub use controller::OrbitController;
pub use windowed::{run_windowed, ViewerError};
