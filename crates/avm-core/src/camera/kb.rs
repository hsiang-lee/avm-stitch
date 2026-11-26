//! Kannala-Brandt θ-polynomial projection model.

use crate::camera::model::{
    require_equal_lengths, CameraModel, CameraModelBase, KbParams, ModelError, EPS,
};

#[derive(Clone, Debug)]
pub struct KbCameraModel {
    base: CameraModelBase,
    fx: f64,
    fy: f64,
    k1: f64,
    k2: f64,
    k3: f64,
    k4: f64,
}

impl KbCameraModel {
    pub fn new(p: &KbParams) -> Self {
        Self {
            base: CameraModelBase::new(p.width, p.height, p.cx, p.cy, p.fisheye_radius),
            fx: p.fx,
            fy: p.fy,
            k1: p.k1,
            k2: p.k2,
            k3: p.k3,
            k4: p.k4,
        }
    }
}

impl CameraModel for KbCameraModel {
    fn base(&self) -> &CameraModelBase {
        &self.base
    }

    /// `r_xy < kEps` snaps to the principal point.
    fn project_camera_point(&self, p_cam: [f64; 3]) -> [f64; 2] {
        let (x, y, z) = (p_cam[0], p_cam[1], p_cam[2]);
        let r_xy = (x * x + y * y).sqrt();

        if r_xy < EPS {
            return [self.base.cx(), self.base.cy()];
        }

        let theta = r_xy.atan2(z);
        let theta2 = theta * theta;
        let theta_d = theta
            * (1.0
                + self.k1 * theta2
                + self.k2 * theta2 * theta2
                + self.k3 * theta2 * theta2 * theta2
                + self.k4 * theta2 * theta2 * theta2 * theta2);
        let factor = theta_d / r_xy;
        [
            self.base.cx() + self.fx * factor * x,
            self.base.cy() + self.fy * factor * y,
        ]
    }

    fn project_points(
        &self,
        cam_points: &[[f64; 3]],
        out_uv: &mut [[f64; 2]],
        out_valid: &mut [u8],
    ) -> Result<(), ModelError> {
        require_equal_lengths(cam_points.len(), out_uv.len(), out_valid.len())?;

        for (i, p) in cam_points.iter().enumerate() {
            let uv = self.project_camera_point(*p);
            out_uv[i] = uv;
            out_valid[i] = self.base.rect_contains(uv[0], uv[1]);
        }
        Ok(())
    }
}
