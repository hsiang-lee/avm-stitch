//! `Camera` value object: name + extrinsics + optional projection model.
//!
//! Port of `src/camera_model/camera.hpp`; the C++ side keeps the model
//! pointer in the calibration layer, here the name travels with the camera so
//! that JSON loading and reporting stay simple.

use crate::camera::extrinsics::CameraExtrinsics;
use crate::camera::model::CameraModel;
use crate::camera::model::ModelError;

#[derive(Default)]
pub struct Camera {
    name: String,
    extrinsics: CameraExtrinsics,
    model: Option<Box<dyn CameraModel>>,
}

impl Camera {
    pub fn new(extrinsics: CameraExtrinsics, model: Box<dyn CameraModel>) -> Self {
        Self::named("", extrinsics, model)
    }

    pub fn named(
        name: impl Into<String>,
        extrinsics: CameraExtrinsics,
        model: Box<dyn CameraModel>,
    ) -> Self {
        Self {
            name: name.into(),
            extrinsics,
            model: Some(model),
        }
    }

    pub fn is_valid(&self) -> bool {
        self.model.is_some()
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn extrinsics(&self) -> &CameraExtrinsics {
        &self.extrinsics
    }

    pub fn model(&self) -> Option<&dyn CameraModel> {
        self.model.as_deref()
    }

    /// Precondition: `is_valid()`.
    pub fn width(&self) -> i32 {
        self.model_ref().width()
    }

    /// Precondition: `is_valid()`.
    pub fn height(&self) -> i32 {
        self.model_ref().height()
    }

    /// Precondition: `is_valid()`.
    pub fn inside_fisheye_circle(&self, u: f64, v: f64) -> bool {
        self.model_ref().inside_fisheye_circle(u, v)
    }

    /// Precondition: `is_valid()`. Default gate width is 40 px, as in C++.
    pub fn edge_feather(&self, u: f64, v: f64, feather_px: f64) -> f64 {
        self.model_ref().edge_feather(u, v, feather_px)
    }

    /// Precondition: `is_valid()`.
    pub fn project_points(
        &self,
        cam_points: &[[f64; 3]],
        out_uv: &mut [[f64; 2]],
        out_valid: &mut [u8],
    ) -> Result<(), ModelError> {
        self.model_ref().project_points(cam_points, out_uv, out_valid)
    }

    /// Single-point forward projection returning uv plus the rect bit.
    ///
    /// Precondition: `is_valid()`.
    pub fn project_point(&self, p_cam: [f64; 3]) -> ([f64; 2], u8) {
        let mut uv = [[0.0; 2]];
        let mut valid = [0u8];
        self.project_points(&[p_cam], &mut uv, &mut valid)
            .expect("single-point projection uses equal-length slices");
        (uv[0], valid[0])
    }

    fn model_ref(&self) -> &dyn CameraModel {
        self.model
            .as_deref()
            .expect("Camera: model accessor called on an invalid camera")
    }
}
