//! Camera view matrices for the offscreen renders.
//!
//! The render basis is the OSG conversion used by the C++ scene builder
//! (`x, z, -y`), i.e. y-up right-handed.

use nalgebra::{Matrix4, Perspective3, Point3};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ViewSpec {
    /// Column-major view-projection matrix, ready for the WGSL uniform.
    pub view_proj: [[f32; 4]; 4],
}

impl ViewSpec {
    pub fn new(view_proj: [[f32; 4]; 4]) -> Self {
        Self { view_proj }
    }

    /// Right-handed look-at perspective view.
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
        let projection = Perspective3::new(aspect, fov_y_deg.to_radians(), near, far);
        let view_proj = projection.to_homogeneous() * view;

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
