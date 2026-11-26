mod common;

use avm_core::camera::{make_camera_model, Camera, CameraModelParams};
use common::{identity_extrinsics, kb_params};

#[test]
fn default_camera_is_invalid() {
    let cam = Camera::default();
    assert!(!cam.is_valid());
    assert_eq!("", cam.name());
}

#[test]
fn constructed_camera_is_valid_and_delegates_dimensions() {
    let model = make_camera_model("kb", &CameraModelParams::Kb(kb_params())).expect("kb model");
    let cam = Camera::new(identity_extrinsics(), model);
    assert!(cam.is_valid());
    assert_eq!(640, cam.width());
    assert_eq!(480, cam.height());
}

#[test]
fn moved_camera_preserves_validity() {
    let model = make_camera_model("kb", &CameraModelParams::Kb(kb_params())).expect("kb model");
    let src = Camera::new(identity_extrinsics(), model);
    assert!(src.is_valid());
    let dst = src;
    assert!(dst.is_valid());
    assert_eq!(640, dst.width());
}

#[test]
fn named_camera_keeps_its_name() {
    let model = make_camera_model("kb", &CameraModelParams::Kb(kb_params())).expect("kb model");
    let cam = Camera::named("front", identity_extrinsics(), model);
    assert_eq!("front", cam.name());
    assert!(cam.is_valid());
}
