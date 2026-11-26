use std::path::PathBuf;

use serde::Deserialize;
use thiserror::Error;

use crate::bowl::BowlParams;
use crate::camera::{make_camera_model, Camera, CameraModelParams, KbParams, ModelError};

#[derive(Debug, Clone, Deserialize)]
pub struct CarSpec {
    pub length: f64,
    pub width: f64,
    #[serde(default)]
    pub axis_x: f64,
    #[serde(default)]
    pub axis_y: f64,
    #[serde(default)]
    pub origin_convention: String,
}

#[derive(Debug, Clone, Default, Deserialize)]
pub struct IntrinsicsSpec {
    #[serde(default)]
    pub fx: f64,
    #[serde(default)]
    pub fy: f64,
    #[serde(default)]
    pub cx: f64,
    #[serde(default)]
    pub cy: f64,
    #[serde(default)]
    pub k1: f64,
    #[serde(default)]
    pub k2: f64,
    #[serde(default)]
    pub k3: f64,
    #[serde(default)]
    pub k4: f64,
}

#[derive(Debug, Clone, Deserialize)]
pub struct ExtrinsicsSpec {
    pub r: [[f64; 3]; 3],
    pub t: [f64; 3],
}

#[derive(Debug, Clone, Deserialize)]
pub struct CameraSpec {
    #[serde(default)]
    pub name: String,
    #[serde(default)]
    pub image: String,
    #[serde(default)]
    pub width: i32,
    #[serde(default)]
    pub height: i32,
    #[serde(default)]
    pub intrinsics: IntrinsicsSpec,
    pub extrinsics: ExtrinsicsSpec,
}

#[derive(Debug, Clone, Deserialize)]
pub struct CalibrationFile {
    pub car: CarSpec,
    pub cameras: Vec<CameraSpec>,
}

#[derive(Debug, Error)]
pub enum CalibrationError {
    #[error("failed to read {path}: {source}")]
    Io {
        path: String,
        #[source]
        source: std::io::Error,
    },
    #[error("failed to parse calibration JSON: {0}")]
    Json(#[from] serde_json::Error),
    #[error(
        "calibration.json must contain a 'car' block with positive length/width \
         — required to derive bowl params (got length={length}, width={width})"
    )]
    NonPositiveCarSize { length: f64, width: f64 },
    #[error("calibration contains no cameras")]
    NoCameras,
    #[error("camera '{name}': {source}")]
    CameraModel {
        name: String,
        #[source]
        source: ModelError,
    },
}

pub struct Calibration {
    pub car: CarSpec,
    pub cameras: Vec<Camera>,
    /// Image file names, index-aligned with `cameras` (may be empty strings).
    pub image_files: Vec<String>,
}

impl Calibration {
    pub fn load_from_str(content: &str) -> Result<Self, CalibrationError> {
        let file: CalibrationFile = serde_json::from_str(content)?;

        if file.car.length <= 0.0 || file.car.width <= 0.0 {
            return Err(CalibrationError::NonPositiveCarSize {
                length: file.car.length,
                width: file.car.width,
            });
        }
        if file.cameras.is_empty() {
            return Err(CalibrationError::NoCameras);
        }

        let mut cameras = Vec::with_capacity(file.cameras.len());
        let mut image_files = Vec::with_capacity(file.cameras.len());
        for spec in &file.cameras {
            let params = KbParams {
                width: spec.width,
                height: spec.height,
                fx: spec.intrinsics.fx,
                fy: spec.intrinsics.fy,
                cx: spec.intrinsics.cx,
                cy: spec.intrinsics.cy,
                k1: spec.intrinsics.k1,
                k2: spec.intrinsics.k2,
                k3: spec.intrinsics.k3,
                k4: spec.intrinsics.k4,
                fisheye_radius: 0.0,
            };
            let model = make_camera_model("kb", &CameraModelParams::Kb(params)).map_err(|source| {
                CalibrationError::CameraModel {
                    name: spec.name.clone(),
                    source,
                }
            })?;
            let extrinsics = crate::camera::CameraExtrinsics::from_rows(
                spec.extrinsics.r,
                spec.extrinsics.t,
            );
            cameras.push(Camera::named(spec.name.clone(), extrinsics, model));
            image_files.push(spec.image.clone());
        }

        Ok(Self {
            car: file.car,
            cameras,
            image_files,
        })
    }

    pub fn load_from_path(path: impl Into<PathBuf>) -> Result<Self, CalibrationError> {
        let path = path.into();
        let content = std::fs::read_to_string(&path).map_err(|source| CalibrationError::Io {
            path: path.display().to_string(),
            source,
        })?;
        Self::load_from_str(&content)
    }

    /// Bowl derived from the car block (positive length/width guaranteed at load).
    pub fn bowl_params(&self) -> BowlParams {
        BowlParams::for_car_size(self.car.length, self.car.width, self.car.axis_x)
    }
}
