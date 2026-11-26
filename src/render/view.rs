//! The render basis is y-up right-handed with the `(x, z, -y)` axis remap.

use nalgebra::{Matrix4, Point3};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ViewSpec {
    /// Column-major view-projection matrix, ready for the WGSL uniform.
    pub view_proj: [[f32; 4]; 4],
}

impl ViewSpec {
    pub fn new(view_proj: [[f32; 4]; 4]) -> Self {
        Self { view_proj }
    }

    pub fn look_at(
        eye: [f32; 3],
        target: [f32; 3],
        up: [f32; 3],
        fov_y_deg: f32,
        aspect: f32,
        near: f32,
        far: f32,
    ) -> Self {
        let eye = Point3::new(eye[0], eye[1], eye[2]);
        let target = Point3::new(target[0], target[1], target[2]);
        let up = nalgebra::Vector3::new(up[0], up[1], up[2]);

        let view = Matrix4::look_at_rh(&eye, &target, &up);
        let view_proj = wgpu_perspective(fov_y_deg, aspect, near, far) * view;

        let mut columns = [[0.0f32; 4]; 4];
        for col in 0..4 {
            for row in 0..4 {
                columns[col][row] = view_proj[(row, col)];
            }
        }
        Self {
            view_proj: columns,
        }
    }
}

/// Right-handed perspective for wgpu clip space (`0 <= z_ndc <= 1`).
///
/// `nalgebra::Perspective3` uses the OpenGL convention `-1 <= z_ndc <= 1`, so
/// its near plane would be clipped away and the depth test would reject most of
/// the scene. With `w_clip = -z_view`, the wgpu mapping is
/// `z_clip = far * (z_view + near) / (near - far)` and therefore
/// `z_ndc = far * (z_view + near) / (z_view * (far - near))`, i.e.
/// `m22 = far/(near-far)`, `m23 = far*near/(near-far)`, `m32 = -1`.
fn wgpu_perspective(fov_y_deg: f32, aspect: f32, near: f32, far: f32) -> Matrix4<f32> {
    let f = 1.0 / (0.5 * fov_y_deg.to_radians()).tan();
    let mut projection = Matrix4::zeros();
    projection[(0, 0)] = f / aspect;
    projection[(1, 1)] = f;
    projection[(2, 2)] = far / (near - far);
    projection[(2, 3)] = far * near / (near - far);
    projection[(3, 2)] = -1.0;
    projection
}
