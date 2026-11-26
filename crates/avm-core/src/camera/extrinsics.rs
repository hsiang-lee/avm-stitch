//! OpenCV convention: the camera y-axis points down; `rotation` maps camera to
//! world and `translation` is the camera position in world coordinates.

use nalgebra::{Matrix3, Vector3};

use crate::angle_utils::{normalize_angle_deg, RAD_TO_DEG};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CameraExtrinsics {
    rotation: Matrix3<f64>,
    translation: Vector3<f64>,
}

impl Default for CameraExtrinsics {
    fn default() -> Self {
        Self::identity()
    }
}

impl CameraExtrinsics {
    pub fn new(rotation: Matrix3<f64>, translation: Vector3<f64>) -> Self {
        Self {
            rotation,
            translation,
        }
    }

    /// Row-major rotation, same element order as the C++ matrix literals.
    pub fn from_rows(rows: [[f64; 3]; 3], translation: [f64; 3]) -> Self {
        let r = [
            rows[0][0], rows[0][1], rows[0][2], //
            rows[1][0], rows[1][1], rows[1][2], //
            rows[2][0], rows[2][1], rows[2][2],
        ];
        Self::new(
            Matrix3::from_row_slice(&r),
            Vector3::new(translation[0], translation[1], translation[2]),
        )
    }

    pub fn identity() -> Self {
        Self::new(Matrix3::identity(), Vector3::zeros())
    }

    pub fn rotation(&self) -> &Matrix3<f64> {
        &self.rotation
    }

    /// World (VCS) point to camera frame: `R^T (P - t)`.
    pub fn to_camera(&self, p_world: Vector3<f64>) -> Vector3<f64> {
        self.rotation.transpose() * (p_world - self.translation)
    }

    /// Camera frame point to world (VCS): `R P + t`.
    pub fn to_world(&self, p_cam: Vector3<f64>) -> Vector3<f64> {
        self.rotation * p_cam + self.translation
    }

    pub fn rotate_world_to_camera(&self, v: Vector3<f64>) -> Vector3<f64> {
        self.rotation.transpose() * v
    }

    pub fn rotate_camera_to_world(&self, v: Vector3<f64>) -> Vector3<f64> {
        self.rotation * v
    }

    pub fn position(&self) -> Vector3<f64> {
        self.translation
    }

    /// Azimuth of the camera forward axis in the world XY plane, `[0, 360)`.
    pub fn azimuth_deg(&self) -> f64 {
        let axis = self.rotate_camera_to_world(Vector3::new(0.0, 0.0, 1.0));
        normalize_angle_deg(axis[1].atan2(axis[0]) * RAD_TO_DEG)
    }

    /// Rotation from `other` camera frame into this camera frame: `R_this^T R_other`.
    pub fn relative_rotation_from(&self, other: &CameraExtrinsics) -> Matrix3<f64> {
        self.rotation.transpose() * other.rotation
    }
}
