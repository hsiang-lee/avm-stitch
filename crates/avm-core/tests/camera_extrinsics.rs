//! Port of `tests/camera_model/test_camera_extrinsics.cpp`.

mod common;

use avm_core::camera::CameraExtrinsics;
use common::{from_rows, vec3};

fn half_angle_rotation() -> [[f64; 3]; 3] {
    let s = 0.5;
    let c = 3.0f64.sqrt() / 2.0;
    [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]
}

#[test]
fn to_camera_matches_hand_computed_rt_p_minus_t() {
    let rotation = half_angle_rotation();
    let t = [1.0, 2.0, -3.0];
    let extr = from_rows(rotation, t);

    let p = vec3(4.0, -5.0, 6.0);
    let expected = extr.rotation().transpose() * (p - vec3(t[0], t[1], t[2]));
    let got = extr.to_camera(p);
    assert_eq!(expected[0], got[0]);
    assert_eq!(expected[1], got[1]);
    assert_eq!(expected[2], got[2]);
}

#[test]
fn to_world_round_trips_to_camera() {
    let extr = from_rows(half_angle_rotation(), [1.0, -2.0, 3.0]);
    let p = vec3(-2.0, 0.5, 4.0);
    let back = extr.to_world(extr.to_camera(p));
    assert!((p[0] - back[0]).abs() < 1e-12);
    assert!((p[1] - back[1]).abs() < 1e-12);
    assert!((p[2] - back[2]).abs() < 1e-12);
}

#[test]
fn to_world_matches_hand_computed_r_p_plus_t() {
    let extr = from_rows(half_angle_rotation(), [1.0, -2.0, 3.0]);
    let p_cam = vec3(0.3, -0.7, 1.4);
    let expected = extr.rotation() * p_cam + extr.position();
    let got = extr.to_world(p_cam);
    assert_eq!(expected[0], got[0]);
    assert_eq!(expected[1], got[1]);
    assert_eq!(expected[2], got[2]);
}

#[test]
fn rotate_world_to_camera_matches_hand_computed_rt_v() {
    let extr = from_rows(half_angle_rotation(), [0.0, 0.0, 0.0]);
    let v = vec3(0.5, -0.25, 0.75);
    let expected = extr.rotation().transpose() * v;
    let got = extr.rotate_world_to_camera(v);
    assert_eq!(expected[0], got[0]);
    assert_eq!(expected[1], got[1]);
    assert_eq!(expected[2], got[2]);
}

#[test]
fn rotate_camera_to_world_matches_hand_computed_r_v() {
    let extr = from_rows(half_angle_rotation(), [0.0, 0.0, 0.0]);
    let v = vec3(0.5, -0.25, 0.75);
    let expected = extr.rotation() * v;
    let got = extr.rotate_camera_to_world(v);
    assert_eq!(expected[0], got[0]);
    assert_eq!(expected[1], got[1]);
    assert_eq!(expected[2], got[2]);
}

#[test]
fn position_matches_translation_vector() {
    let extr = from_rows(half_angle_rotation(), [1.0, 2.0, -3.0]);
    assert_eq!(1.0, extr.position()[0]);
    assert_eq!(2.0, extr.position()[1]);
    assert_eq!(-3.0, extr.position()[2]);
}

#[test]
fn default_constructor_is_identity() {
    let extr = CameraExtrinsics::default();
    let p = vec3(1.0, 2.0, 3.0);
    let out = extr.to_camera(p);
    assert_eq!(p[0], out[0]);
    assert_eq!(p[1], out[1]);
    assert_eq!(p[2], out[2]);
    assert_eq!(0.0, extr.position()[0]);
    assert_eq!(0.0, extr.position()[1]);
    assert_eq!(0.0, extr.position()[2]);
}

#[test]
fn relative_rotation_from_matches_hand_computed_rb_t_ra() {
    let (sa, ca) = (0.5, 3.0f64.sqrt() / 2.0);
    let (sb, cb) = (-0.3, 0.91f64.sqrt());
    let ra = [[ca, 0.0, sa], [0.0, 1.0, 0.0], [-sa, 0.0, ca]];
    let rb = [[cb, sb, 0.0], [-sb, cb, 0.0], [0.0, 0.0, 1.0]];
    let a = from_rows(ra, [1.0, 0.0, 0.0]);
    let b = from_rows(rb, [2.0, 1.0, 0.5]);

    let expected = b.rotation().transpose() * a.rotation();
    let got = b.relative_rotation_from(&a);
    for r in 0..3 {
        for c in 0..3 {
            assert_eq!(expected[(r, c)], got[(r, c)], "element ({r},{c})");
        }
    }
}

#[test]
fn azimuth_deg_of_identity_axis_is_zero() {
    assert_eq!(0.0, CameraExtrinsics::default().azimuth_deg());
}

#[test]
fn azimuth_deg_forward_along_world_y_is_90() {
    // Columns are world axes of the camera basis:
    //   right = (1,0,0), down = (0,0,-1), forward = (0,1,0).
    let extr = from_rows([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]], [0.0; 3]);
    assert!((90.0 - extr.azimuth_deg()).abs() < 1e-12);
}

#[test]
fn azimuth_deg_forward_along_world_45_deg() {
    let s = 2.0f64.sqrt() / 2.0;
    // right = down x forward = (s,-s,0), down = (0,0,-1), forward = (s,s,0).
    let extr = from_rows([[s, 0.0, s], [-s, 0.0, s], [0.0, -1.0, 0.0]], [0.0; 3]);
    assert!((45.0 - extr.azimuth_deg()).abs() < 1e-12);
}

#[test]
fn to_camera_of_other_position_matches_hand_computed_ta_minus_tb() {
    let (sa, ca) = (0.5, 3.0f64.sqrt() / 2.0);
    let (sb, cb) = (-0.3, 0.91f64.sqrt());
    let ra = [[ca, 0.0, sa], [0.0, 1.0, 0.0], [-sa, 0.0, ca]];
    let rb = [[cb, sb, 0.0], [-sb, cb, 0.0], [0.0, 0.0, 1.0]];
    let a = from_rows(ra, [1.0, 0.0, 0.0]);
    let b = from_rows(rb, [2.0, 1.0, 0.5]);

    let expected = b.rotation().transpose() * (a.position() - b.position());
    let got = b.to_camera(a.position());
    assert_eq!(expected[0], got[0]);
    assert_eq!(expected[1], got[1]);
    assert_eq!(expected[2], got[2]);
}
