#[allow(clippy::module_inception)] // `camera::camera` mirrors the file layout.
pub mod camera;
pub mod extrinsics;
pub mod kb;
pub mod lut;
pub mod model;
pub mod ocam;

pub use camera::Camera;
pub use extrinsics::CameraExtrinsics;
pub use kb::KbCameraModel;
pub use lut::LutCameraModel;
pub use model::{
    make_camera_model, CameraModel, CameraModelBase, CameraModelParams, KbParams, LutParams,
    ModelError, OcamParams, DEFAULT_EDGE_FEATHER_PX, EPS,
};
pub use ocam::OcamCameraModel;
