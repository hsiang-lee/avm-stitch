//! Test support shared by the camera test binaries.
//!
//! Mirrors `tests/test_support/geometry_test_helpers.hpp`.
#![allow(dead_code)]

use avm_core::bowl::{BowlModel, BowlParams};
use avm_core::camera::{
    make_camera_model, Camera, CameraExtrinsics, CameraModel, CameraModelParams, KbCameraModel,
    KbParams,
};
use avm_core::{Matrix3, Vector3};

pub fn vec3(x: f64, y: f64, z: f64) -> Vector3<f64> {
    Vector3::new(x, y, z)
}

pub fn arr(v: Vector3<f64>) -> [f64; 3] {
    [v.x, v.y, v.z]
}

pub fn from_rows(r: [[f64; 3]; 3], t: [f64; 3]) -> CameraExtrinsics {
    CameraExtrinsics::from_rows(r, t)
}

/// `KbParams` for the 640x480 fx=fy=300, cx=320, cy=240 fixture.
pub fn kb_params() -> KbParams {
    kb_intrinsics(0.0, 0.0, 0.0, 0.0)
}

pub fn kb_intrinsics(k1: f64, k2: f64, k3: f64, k4: f64) -> KbParams {
    kb_intrinsics_with_focal(k1, k2, k3, k4, 300.0, 300.0)
}

pub fn kb_intrinsics_with_focal(k1: f64, k2: f64, k3: f64, k4: f64, fx: f64, fy: f64) -> KbParams {
    KbParams {
        width: 640,
        height: 480,
        fx,
        fy,
        cx: 320.0,
        cy: 240.0,
        k1,
        k2,
        k3,
        k4,
        fisheye_radius: 0.0,
    }
}

/// Real 1920x1536 fixture used by the bake tests.
pub fn real_intrinsics() -> KbParams {
    KbParams {
        width: 1920,
        height: 1536,
        fx: 441.45,
        fy: 441.45,
        cx: 952.14,
        cy: 758.27,
        k1: 0.0,
        k2: 0.0,
        k3: 0.0,
        k4: 0.0,
        fisheye_radius: 0.0,
    }
}

pub fn identity_extrinsics() -> CameraExtrinsics {
    CameraExtrinsics::identity()
}

/// Mirrors `MakeKbModel` of `tests/camera_model/test_kb_camera_model.cpp`.
#[allow(clippy::too_many_arguments)]
pub fn make_kb_model(
    width: i32,
    height: i32,
    fx: f64,
    fy: f64,
    cx: f64,
    cy: f64,
    k1: f64,
    k2: f64,
    k3: f64,
    k4: f64,
    fisheye_radius: f64,
) -> KbCameraModel {
    KbCameraModel::new(&KbParams {
        width,
        height,
        fx,
        fy,
        cx,
        cy,
        k1,
        k2,
        k3,
        k4,
        fisheye_radius,
    })
}

/// The default 640x480/300px model used by most projection tests.
pub fn standard_kb_model() -> KbCameraModel {
    make_kb_model(
        640, 480, 300.0, 300.0, 320.0, 240.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    )
}

pub fn kb_camera(k1: f64, k2: f64, k3: f64, k4: f64) -> Camera {
    let params = CameraModelParams::Kb(kb_intrinsics(k1, k2, k3, k4));
    Camera::new(
        identity_extrinsics(),
        make_camera_model("kb", &params).expect("kb fixture must build"),
    )
}

/// `MakeSideCameras`: camera on a 3 m circle looking at the origin.
pub fn side_cameras(azimuths_deg: &[f64]) -> Vec<CameraExtrinsics> {
    azimuths_deg
        .iter()
        .map(|az| {
            let rad = az * std::f64::consts::PI / 180.0;
            let pos = vec3(rad.cos() * 3.0, rad.sin() * 3.0, 1.0);
            let forward = vec3(rad.cos(), rad.sin(), 0.0);
            let down = vec3(0.0, 0.0, -1.0);
            let right = down.cross(&forward);
            CameraExtrinsics::from_rows(
                [
                    [right.x, down.x, forward.x],
                    [right.y, down.y, forward.y],
                    [right.z, down.z, forward.z],
                ],
                [pos.x, pos.y, pos.z],
            )
        })
        .collect()
}

pub fn wrap_kb_side_cameras(azimuths_deg: &[f64]) -> Vec<Camera> {
    let params = CameraModelParams::Kb(kb_params());
    side_cameras(azimuths_deg)
        .into_iter()
        .map(|ext| {
            Camera::new(
                ext,
                make_camera_model("kb", &params).expect("kb fixture must build"),
            )
        })
        .collect()
}

pub fn wrap_kb_cameras(intrinsics: &[KbParams], extrinsics: &[CameraExtrinsics]) -> Vec<Camera> {
    intrinsics
        .iter()
        .zip(extrinsics)
        .map(|(intr, ext)| {
            let params = CameraModelParams::Kb(intr.clone());
            Camera::new(
                *ext,
                make_camera_model("kb", &params).expect("kb fixture must build"),
            )
        })
        .collect()
}

/// `MakeLookAtCamera`: eye on a circle of radius `r` at height `h`, looking at
/// `(0, 0, target_z)`; camera down axis points towards the ground.
pub fn make_look_at_camera(az_deg: f64, r: f64, h: f64, target_z: f64) -> CameraExtrinsics {
    let rad = az_deg * std::f64::consts::PI / 180.0;
    let pos = vec3(r * rad.cos(), r * rad.sin(), h);
    let target = vec3(0.0, 0.0, target_z);
    let f = (target - pos).normalize();
    let up = vec3(0.0, 0.0, 1.0);
    let right = f.cross(&up).normalize();
    let u = right.cross(&f);
    let down = -u;
    CameraExtrinsics::from_rows(
        [
            [right.x, down.x, f.x],
            [right.y, down.y, f.y],
            [right.z, down.z, f.z],
        ],
        [pos.x, pos.y, pos.z],
    )
}

/// `MakeLookAtRotation` of `tests/camera_model/test_kb_camera_model.cpp`.
pub fn look_at_rotation(pos: Vector3<f64>, target: Vector3<f64>) -> Matrix3<f64> {
    let f = (target - pos).normalize();
    let world_up = vec3(0.0, 0.0, 1.0);
    let mut r = world_up.cross(&f);
    if r.norm() < 1e-9 {
        r = vec3(1.0, 0.0, 0.0);
    }
    r = r.normalize();
    let u = f.cross(&r);
    Matrix3::from_row_slice(&[
        r.x, u.x, f.x, //
        r.y, u.y, f.y, //
        r.z, u.z, f.z,
    ])
}

pub fn project_vec(model: &dyn CameraModel, pts: &[[f64; 3]]) -> (Vec<[f64; 2]>, Vec<u8>) {
    let mut uv = vec![[0.0f64; 2]; pts.len()];
    let mut valid = vec![0u8; pts.len()];
    model
        .project_points(pts, &mut uv, &mut valid)
        .expect("equal-length batch");
    (uv, valid)
}

/// Batch projection must agree with one-element calls for uv and the rect bit.
pub fn assert_batch_matches_single(model: &dyn CameraModel, pts: &[[f64; 3]]) {
    let (uv_batch, valid_batch) = project_vec(model, pts);
    for (i, p) in pts.iter().enumerate() {
        let (uv_one, valid_one) = project_vec(model, &[*p]);
        assert_eq!(uv_one[0][0], uv_batch[i][0], "point {i} u");
        assert_eq!(uv_one[0][1], uv_batch[i][1], "point {i} v");
        assert_eq!(valid_one[0], valid_batch[i], "point {i} valid");
    }
}

// ---------------------------------------------------------------------------
// Bowl fixtures (mirror `tests/test_support/stitch_test_helpers.hpp`).
// ---------------------------------------------------------------------------

/// `MakeCarBowl`: the calibrated 5.6 m car bowl.
pub fn make_car_bowl() -> BowlParams {
    BowlParams::for_car_size(5.6, 2.2, 2.9)
}

/// `MakeCoarseBowlParams`: same shape, 1.0 m grid step.
pub fn make_coarse_bowl_params(outer_radius: f64) -> BowlParams {
    let derived = make_car_bowl();
    BowlParams::new(
        5.6,
        2.2,
        derived.flat_platform_radius(),
        1.0,
        outer_radius,
        derived.slope_coefficient(),
        derived.height_exponent(),
        derived.max_height(),
        [0.0, 0.0],
    )
}

/// Car-derived bowl with the calibrated grid step and a centred car.
pub fn mesh_params() -> BowlParams {
    let derived = make_car_bowl();
    BowlParams::new(
        5.6,
        2.2,
        derived.flat_platform_radius(),
        derived.grid_step(),
        derived.outer_radius(),
        derived.slope_coefficient(),
        derived.height_exponent(),
        derived.max_height(),
        [0.0, 0.0],
    )
}

/// `CarBowlCentered` of `tests/bowl_model/test_bowl_model.cpp`.
pub fn car_bowl_centered() -> BowlParams {
    mesh_params()
}

/// Steep synthetic bowl used by the gating tests.
pub fn tall_bowl_params() -> BowlParams {
    BowlParams::new(2.0, 1.0, 0.5, 0.1, 6.5, 3.2, 1.0, 15.0, [0.0, 0.0])
}

pub fn default_intrinsics() -> Vec<KbParams> {
    vec![real_intrinsics(); 4]
}

pub fn default_bake_cameras() -> Vec<CameraExtrinsics> {
    side_cameras(&[0.0, 180.0, 90.0, -90.0])
}

pub fn make_default_kb_cameras() -> Vec<Camera> {
    wrap_kb_cameras(&default_intrinsics(), &default_bake_cameras())
}

pub fn make_coarse_bowl(
    outer_radius: f64,
    intrinsics: &[KbParams],
    extrinsics: &[CameraExtrinsics],
) -> BowlModel {
    BowlModel::new(
        &make_coarse_bowl_params(outer_radius),
        &wrap_kb_cameras(intrinsics, extrinsics),
    )
}

/// Replica of the C++ `IsVertexVisibleToCamera` helper.
pub fn is_vertex_visible_to_camera(
    p: &[f64; 3],
    intr: &KbParams,
    extr: &CameraExtrinsics,
) -> bool {
    let rep = make_kb_model(
        intr.width, intr.height, intr.fx, intr.fy, intr.cx, intr.cy, intr.k1, intr.k2, intr.k3,
        intr.k4, intr.fisheye_radius,
    );
    let uv = rep.project_camera_point(arr(extr.to_camera(vec3(p[0], p[1], p[2]))));
    uv[0] >= 0.0
        && uv[0] < f64::from(rep.width())
        && uv[1] >= 0.0
        && uv[1] < f64::from(rep.height())
        && rep.inside_fisheye_circle(uv[0], uv[1])
}

/// Replica of the C++ `IsVertexBakedVisible` helper (visibility + edge gate).
pub fn is_vertex_baked_visible(
    p: &[f64; 3],
    intr: &KbParams,
    extr: &CameraExtrinsics,
) -> bool {
    let rep = make_kb_model(
        intr.width, intr.height, intr.fx, intr.fy, intr.cx, intr.cy, intr.k1, intr.k2, intr.k3,
        intr.k4, intr.fisheye_radius,
    );
    let uv = rep.project_camera_point(arr(extr.to_camera(vec3(p[0], p[1], p[2]))));
    is_vertex_visible_to_camera(p, intr, extr)
        && rep.edge_feather(uv[0], uv[1], 40.0) > 0.0
}
