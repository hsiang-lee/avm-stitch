//! OCam (unified spherical) projection model.
//!
//! Port of `src/camera_model/ocam_camera_model.{hpp,cpp}`.

use crate::camera::model::{
    require_equal_lengths, CameraModel, CameraModelBase, ModelError, OcamParams,
};

/// OCam uses its own degenerate epsilon (`1e-12`), not the shared `kEps`.
const OCAM_EPS: f64 = 1e-12;

#[derive(Clone, Debug)]
pub struct OcamCameraModel {
    base: CameraModelBase,
    /// Forward polynomial; kept for parity with the C++ parameter bundle (the
    /// batch projection uses `invpol`, the inverse direction used it).
    #[allow(dead_code)]
    pol: Vec<f64>,
    invpol: Vec<f64>,
    c: f64,
    d: f64,
    e: f64,
}

impl OcamCameraModel {
    pub fn new(p: &OcamParams) -> Self {
        Self {
            base: CameraModelBase::new(p.width, p.height, p.cx, p.cy, 0.0),
            pol: p.pol.clone(),
            invpol: p.invpol.clone(),
            c: p.c,
            d: p.d,
            e: p.e,
        }
    }
}

impl CameraModel for OcamCameraModel {
    fn base(&self) -> &CameraModelBase {
        &self.base
    }

    fn project_points(
        &self,
        cam_points: &[[f64; 3]],
        out_uv: &mut [[f64; 2]],
        out_valid: &mut [u8],
    ) -> Result<(), ModelError> {
        require_equal_lengths(cam_points.len(), out_uv.len(), out_valid.len())?;

        for (i, p) in cam_points.iter().enumerate() {
            let (x, y, z) = (p[0], p[1], p[2]);
            let r_xy = (x * x + y * y).sqrt();

            if r_xy < OCAM_EPS {
                out_uv[i] = [self.base.cx(), self.base.cy()];
                out_valid[i] = if z > 0.0 {
                    self.base.rect_contains(self.base.cx(), self.base.cy())
                } else {
                    0
                };
                continue;
            }
            if z <= 0.0 {
                out_uv[i] = [self.base.cx(), self.base.cy()];
                out_valid[i] = 0;
                continue;
            }

            let theta = (-z / r_xy).atan();
            let mut rho = self.invpol.first().copied().unwrap_or(0.0);
            let mut t_i = 1.0;
            for k in 1..self.invpol.len() {
                t_i *= theta;
                rho += t_i * self.invpol[k];
            }
            let scale = rho / r_xy;
            let px = x * scale;
            let py = y * scale;
            let u = px * self.c + py * self.d + self.base.cx();
            let v = px * self.e + py + self.base.cy();

            out_uv[i] = [u, v];
            out_valid[i] = self.base.rect_contains(u, v);
        }
        Ok(())
    }
}
