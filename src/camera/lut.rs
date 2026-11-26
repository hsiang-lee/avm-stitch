//! Table-LUT (θ_deg → height_mm) projection model.

use crate::camera::model::{
    require_equal_lengths, CameraModel, CameraModelBase, LutParams, ModelError, EPS,
};

#[derive(Clone, Debug)]
pub struct LutCameraModel {
    base: CameraModelBase,
    points: Vec<[f64; 2]>,
    pixel_size_mm: f64,
}

impl LutCameraModel {
    pub fn new(p: &LutParams) -> Self {
        Self {
            base: CameraModelBase::new(p.width, p.height, p.cx, p.cy, 0.0),
            points: p.points.clone(),
            pixel_size_mm: p.pixel_size_mm,
        }
    }

    /// Piecewise-linear `theta_deg -> radius_px`, clamped outside the table.
    ///
    /// Precondition (enforced by the factory): at least two rows, strictly
    /// ascending angle column, positive pixel size.
    pub fn radius_px_for_angle(&self, theta_deg: f64) -> f64 {
        let xs = &self.points;
        let height_at = |i: usize| xs[i][1];

        if theta_deg <= xs[0][0] {
            return height_at(0) / self.pixel_size_mm;
        }
        if theta_deg >= xs[xs.len() - 1][0] {
            return height_at(xs.len() - 1) / self.pixel_size_mm;
        }
        let hi = xs.partition_point(|p| p[0] <= theta_deg);
        let lo = hi - 1;
        let t = (theta_deg - xs[lo][0]) / (xs[hi][0] - xs[lo][0]);
        (height_at(lo) + t * (height_at(hi) - height_at(lo))) / self.pixel_size_mm
    }
}

impl CameraModel for LutCameraModel {
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

            if r_xy < EPS {
                out_uv[i] = [self.base.cx(), self.base.cy()];
                out_valid[i] = self.base.rect_contains(self.base.cx(), self.base.cy());
                continue;
            }

            let theta_deg = r_xy.atan2(z) * 180.0 / std::f64::consts::PI;
            let r_px = self.radius_px_for_angle(theta_deg);
            let scale = r_px / r_xy;
            let u = self.base.cx() + scale * x;
            let v = self.base.cy() + scale * y;

            out_uv[i] = [u, v];
            out_valid[i] = self.base.rect_contains(u, v);
        }
        Ok(())
    }
}
