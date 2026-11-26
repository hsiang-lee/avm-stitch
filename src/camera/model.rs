//! Projection model abstraction, gating surface and factory.

use thiserror::Error;

pub const EPS: f64 = 1e-9;

pub const DEFAULT_EDGE_FEATHER_PX: f64 = 40.0;

#[derive(Debug, Error, PartialEq)]
pub enum ModelError {
    #[error("ProjectPoints: cam_points/out_uv/out_valid must have equal lengths")]
    LengthMismatch,
    #[error("MakeCameraModel: unknown model '{0}'")]
    UnknownModel(String),
    #[error("MakeCameraModel: params for model '{model}' must be {expected}")]
    ParamMismatch {
        model: &'static str,
        expected: &'static str,
    },
    #[error("MakeCameraModel: {model} camera width/height must be > 0, got {width}x{height}")]
    NonPositiveSize {
        model: &'static str,
        width: i32,
        height: i32,
    },
    #[error("MakeCameraModel: table LUT needs >= 2 points, got {0}")]
    LutTooFewPoints(usize),
    #[error("MakeCameraModel: table LUT angle_deg column must be strictly ascending (row {0})")]
    LutNotAscending(usize),
    #[error("MakeCameraModel: table pixel_size_mm must be > 0, got {0}")]
    NonPositivePixelSize(f64),
    #[error("MakeCameraModel: ocam pol/invpol must be non-empty")]
    OcamEmptyPolynomials,
}

/// Image geometry plus the effective-circle gate configuration. Every model
/// shares this single gating implementation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CameraModelBase {
    width: i32,
    height: i32,
    cx: f64,
    cy: f64,
    gate_radius_raw: f64,
}

impl CameraModelBase {
    pub fn new(width: i32, height: i32, cx: f64, cy: f64, fisheye_radius: f64) -> Self {
        Self {
            width,
            height,
            cx,
            cy,
            gate_radius_raw: fisheye_radius,
        }
    }

    pub fn width(&self) -> i32 {
        self.width
    }

    pub fn height(&self) -> i32 {
        self.height
    }

    pub fn cx(&self) -> f64 {
        self.cx
    }

    pub fn cy(&self) -> f64 {
        self.cy
    }

    /// `1` when the pixel lies inside `[0,width) x [0,height)`.
    pub fn rect_contains(&self, u: f64, v: f64) -> u8 {
        u8::from(
            u >= 0.0 && u < f64::from(self.width) && v >= 0.0 && v < f64::from(self.height),
        )
    }

    /// Effective-circle radius; `0` in the raw config means the inscribed circle.
    pub fn fisheye_circle_radius(&self) -> f64 {
        if self.gate_radius_raw > 0.0 {
            return self.gate_radius_raw;
        }
        let r = self
            .cx
            .min(self.cy)
            .min(f64::from(self.width) - self.cx)
            .min(f64::from(self.height) - self.cy);
        r.max(0.0)
    }

    pub fn inside_fisheye_circle(&self, u: f64, v: f64) -> bool {
        (u - self.cx).hypot(v - self.cy) <= self.fisheye_circle_radius()
    }

    /// Hard gate for pixels near the fisheye effective-circle edge: `1.0` well
    /// inside, `0.0` within the feather band. `feather_px <= 0` disables it.
    pub fn edge_feather(&self, u: f64, v: f64, feather_px: f64) -> f64 {
        if feather_px <= 0.0 {
            return 1.0;
        }
        let r_px = (u - self.cx).hypot(v - self.cy);
        let margin = self.fisheye_circle_radius() - r_px;
        if margin >= feather_px {
            1.0
        } else {
            0.0
        }
    }
}

/// A camera projection model consuming camera-frame points.
pub trait CameraModel: Send + Sync {
    fn base(&self) -> &CameraModelBase;

    fn project_points(
        &self,
        cam_points: &[[f64; 3]],
        out_uv: &mut [[f64; 2]],
        out_valid: &mut [u8],
    ) -> Result<(), ModelError>;

    fn project_camera_point(&self, p_cam: [f64; 3]) -> [f64; 2] {
        let mut uv = [[0.0, 0.0]];
        let mut valid = [0u8];
        let _ = self.project_points(&[p_cam], &mut uv, &mut valid);
        uv[0]
    }

    fn width(&self) -> i32 {
        self.base().width()
    }

    fn height(&self) -> i32 {
        self.base().height()
    }

    fn cx(&self) -> f64 {
        self.base().cx()
    }

    fn cy(&self) -> f64 {
        self.base().cy()
    }

    fn fisheye_circle_radius(&self) -> f64 {
        self.base().fisheye_circle_radius()
    }

    fn inside_fisheye_circle(&self, u: f64, v: f64) -> bool {
        self.base().inside_fisheye_circle(u, v)
    }

    fn edge_feather(&self, u: f64, v: f64, feather_px: f64) -> f64 {
        self.base().edge_feather(u, v, feather_px)
    }
}

pub fn require_equal_lengths(
    cam_points: usize,
    out_uv: usize,
    out_valid: usize,
) -> Result<(), ModelError> {
    if cam_points != out_uv || cam_points != out_valid {
        return Err(ModelError::LengthMismatch);
    }
    Ok(())
}

fn require_positive_size(
    model: &'static str,
    width: i32,
    height: i32,
) -> Result<(), ModelError> {
    if width <= 0 || height <= 0 {
        return Err(ModelError::NonPositiveSize {
            model,
            width,
            height,
        });
    }
    Ok(())
}

/// Flat KB (Kannala-Brandt) parameters.
#[derive(Clone, Debug, PartialEq)]
pub struct KbParams {
    pub width: i32,
    pub height: i32,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub k1: f64,
    pub k2: f64,
    pub k3: f64,
    pub k4: f64,
    pub fisheye_radius: f64,
}

impl Default for KbParams {
    fn default() -> Self {
        Self {
            width: 0,
            height: 0,
            fx: 0.0,
            fy: 0.0,
            cx: 0.0,
            cy: 0.0,
            k1: 0.0,
            k2: 0.0,
            k3: 0.0,
            k4: 0.0,
            fisheye_radius: 0.0,
        }
    }
}

/// `(theta_deg, height_mm)` lookup table parameters.
#[derive(Clone, Debug, PartialEq)]
pub struct LutParams {
    pub points: Vec<[f64; 2]>,
    pub pixel_size_mm: f64,
    pub width: i32,
    pub height: i32,
    pub cx: f64,
    pub cy: f64,
}

impl Default for LutParams {
    fn default() -> Self {
        Self {
            points: Vec::new(),
            pixel_size_mm: 0.003,
            width: 0,
            height: 0,
            cx: 0.0,
            cy: 0.0,
        }
    }
}

/// OCam (unified spherical) parameters.
#[derive(Clone, Debug, PartialEq)]
pub struct OcamParams {
    pub pol: Vec<f64>,
    pub invpol: Vec<f64>,
    pub cx: f64,
    pub cy: f64,
    pub c: f64,
    pub d: f64,
    pub e: f64,
    pub width: i32,
    pub height: i32,
}

impl Default for OcamParams {
    fn default() -> Self {
        Self {
            pol: Vec::new(),
            invpol: Vec::new(),
            cx: 0.0,
            cy: 0.0,
            c: 1.0,
            d: 0.0,
            e: 0.0,
            width: 0,
            height: 0,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum CameraModelParams {
    Kb(KbParams),
    Lut(LutParams),
    Ocam(OcamParams),
}

/// Factory. The validation order is part of the contract: first failure wins.
pub fn make_camera_model(
    model: &str,
    params: &CameraModelParams,
) -> Result<Box<dyn CameraModel>, ModelError> {
    match model {
        "kb" => {
            let CameraModelParams::Kb(kb) = params else {
                return Err(ModelError::ParamMismatch {
                    model: "kb",
                    expected: "KbParams",
                });
            };
            require_positive_size("kb", kb.width, kb.height)?;
            Ok(Box::new(crate::camera::KbCameraModel::new(kb)))
        }
        "table" => {
            let CameraModelParams::Lut(lut) = params else {
                return Err(ModelError::ParamMismatch {
                    model: "table",
                    expected: "LutParams",
                });
            };
            if lut.points.len() < 2 {
                return Err(ModelError::LutTooFewPoints(lut.points.len()));
            }
            // `!(a < b)` / `!(a > b)` are deliberate: the negated form also
            // rejects NaN. Rewriting them as `a >= b` / `a <= b` would let NaN
            // through.
            #[allow(clippy::neg_cmp_op_on_partial_ord)]
            for i in 1..lut.points.len() {
                if !(lut.points[i - 1][0] < lut.points[i][0]) {
                    return Err(ModelError::LutNotAscending(i));
                }
            }
            #[allow(clippy::neg_cmp_op_on_partial_ord)]
            if !(lut.pixel_size_mm > 0.0) {
                return Err(ModelError::NonPositivePixelSize(lut.pixel_size_mm));
            }
            require_positive_size("table", lut.width, lut.height)?;
            Ok(Box::new(crate::camera::LutCameraModel::new(lut)))
        }
        "ocam" => {
            let CameraModelParams::Ocam(ocam) = params else {
                return Err(ModelError::ParamMismatch {
                    model: "ocam",
                    expected: "OcamParams",
                });
            };
            if ocam.pol.is_empty() || ocam.invpol.is_empty() {
                return Err(ModelError::OcamEmptyPolynomials);
            }
            require_positive_size("ocam", ocam.width, ocam.height)?;
            Ok(Box::new(crate::camera::OcamCameraModel::new(ocam)))
        }
        other => Err(ModelError::UnknownModel(other.to_string())),
    }
}
