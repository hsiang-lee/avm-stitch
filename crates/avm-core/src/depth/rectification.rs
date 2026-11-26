use nalgebra::{Matrix3, Vector3};

use crate::angle_utils::{DEG_TO_RAD, RAD_TO_DEG};
use crate::camera::CameraExtrinsics;

/// Rotation from the two camera frames into the rectified frame, where the
/// baseline is the x axis. Fields are fixed at construction: `m1` and `m2` stay
/// right-handed and orthonormal because they are built from a normalised basis.
#[derive(Clone, Copy, Debug)]
pub struct RectificationBasis {
    e1: Vector3<f64>,
    e2: Vector3<f64>,
    e3: Vector3<f64>,
    m1: Matrix3<f64>,
    m2: Matrix3<f64>,
    baseline: f64,
}

impl RectificationBasis {
    pub fn e1(&self) -> Vector3<f64> {
        self.e1
    }

    pub fn e2(&self) -> Vector3<f64> {
        self.e2
    }

    pub fn e3(&self) -> Vector3<f64> {
        self.e3
    }

    pub fn m1(&self) -> &Matrix3<f64> {
        &self.m1
    }

    pub fn m2(&self) -> &Matrix3<f64> {
        &self.m2
    }

    pub fn baseline(&self) -> f64 {
        self.baseline
    }
}

fn normalize_or(v: Vector3<f64>, fallback: Vector3<f64>) -> Vector3<f64> {
    let norm = v.norm();
    if norm > 0.0 {
        v / norm
    } else {
        fallback
    }
}

pub fn make_rectification_basis(
    a: &CameraExtrinsics,
    b: &CameraExtrinsics,
) -> RectificationBasis {
    let rotation = b.relative_rotation_from(a);
    let t_ab = b.to_camera(a.position());
    let t_ba_a = -(rotation.transpose() * t_ab);

    let baseline = t_ba_a.norm();
    let e1 = normalize_or(t_ba_a, Vector3::new(1.0, 0.0, 0.0));

    let z_b = rotation.transpose() * Vector3::new(0.0, 0.0, 1.0);
    let z_m = normalize_or(
        Vector3::new(0.0, 0.0, 1.0) + z_b,
        Vector3::new(0.0, 0.0, 1.0),
    );
    let e2_raw = z_m - z_m.dot(&e1) * e1;
    let e2 = if e2_raw.norm() < 1e-9 {
        let mut reference = Vector3::new(0.0, 0.0, 1.0);
        if e1.dot(&reference).abs() > 0.9 {
            reference = Vector3::new(1.0, 0.0, 0.0);
        }
        normalize_or(
            reference - reference.dot(&e1) * e1,
            Vector3::new(0.0, 1.0, 0.0),
        )
    } else {
        e2_raw / e2_raw.norm()
    };
    let e3 = e1.cross(&e2);

    let m1 = Matrix3::from_row_slice(&[
        e1.x, e1.y, e1.z, //
        e2.x, e2.y, e2.z, //
        e3.x, e3.y, e3.z,
    ]);
    let m2 = m1 * rotation.transpose();

    RectificationBasis {
        e1,
        e2,
        e3,
        m1,
        m2,
        baseline,
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SphericalGrid {
    pub width: usize,
    pub height: usize,
    pub psi_min_deg: f64,
    pub psi_max_deg: f64,
    pub mirror_rows: bool,
}

impl Default for SphericalGrid {
    fn default() -> Self {
        Self {
            width: 1440,
            height: 720,
            psi_min_deg: 0.0,
            psi_max_deg: 180.0,
            mirror_rows: false,
        }
    }
}

impl SphericalGrid {
    pub fn psi_deg(&self, col: usize) -> f64 {
        let t = if self.width > 1 {
            col as f64 / (self.width - 1) as f64
        } else {
            0.0
        };
        self.psi_min_deg + t * (self.psi_max_deg - self.psi_min_deg)
    }

    pub fn phi_deg(&self, row: usize) -> f64 {
        let step = if self.height > 1 {
            360.0 / (self.height - 1) as f64
        } else {
            0.0
        };
        let phi = -180.0 + row as f64 * step;
        if self.mirror_rows {
            -phi
        } else {
            phi
        }
    }

    pub fn direction_c(&self, row: usize, col: usize) -> Vector3<f64> {
        let psi = self.psi_deg(col) * DEG_TO_RAD;
        let phi = self.phi_deg(row) * DEG_TO_RAD;
        Vector3::new(
            psi.cos(),
            psi.sin() * phi.cos(),
            psi.sin() * phi.sin(),
        )
    }
}

pub fn sky_up_needs_mirror(
    basis: &RectificationBasis,
    a: &CameraExtrinsics,
    height: usize,
) -> bool {
    let sky = Vector3::new(0.0, 0.0, 1.0);
    let phi = phi_deg(rectified_direction(basis, sky, a, false));
    let half = if height > 0 { 0.5 * height as f64 } else { 360.0 };
    let row = if height > 1 {
        (phi + 180.0) * (height - 1) as f64 / 360.0
    } else {
        0.0
    };
    row >= half
}

pub fn camera_direction(
    basis: &RectificationBasis,
    dir_c: Vector3<f64>,
    second: bool,
) -> Vector3<f64> {
    if second {
        basis.m2.transpose() * dir_c
    } else {
        basis.m1.transpose() * dir_c
    }
}

pub fn rectified_direction(
    basis: &RectificationBasis,
    dir_world: Vector3<f64>,
    extrinsics: &CameraExtrinsics,
    second: bool,
) -> Vector3<f64> {
    let dir_cam = extrinsics.rotate_world_to_camera(dir_world);
    if second {
        basis.m2 * dir_cam
    } else {
        basis.m1 * dir_cam
    }
}

pub fn phi_deg(dir_c: Vector3<f64>) -> f64 {
    let mut phi = dir_c[2].atan2(dir_c[1]) * RAD_TO_DEG;
    if phi <= -180.0 {
        phi += 360.0;
    }
    phi
}
