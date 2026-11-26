mod common;

use avm_core::camera::{
    make_camera_model, CameraModelParams, KbParams, LutParams, ModelError,
};

fn kb_params(width: i32, height: i32) -> KbParams {
    KbParams {
        width,
        height,
        fx: 300.0,
        fy: 300.0,
        cx: 320.0,
        cy: 240.0,
        ..KbParams::default()
    }
}

#[test]
fn kb_returns_model_with_image_dimensions() {
    let params = CameraModelParams::Kb(kb_params(640, 480));
    let model = make_camera_model("kb", &params).expect("kb model");
    assert_eq!(640, model.width());
    assert_eq!(480, model.height());
}

#[test]
fn unknown_model_is_rejected_and_named() {
    let params = CameraModelParams::Kb(kb_params(640, 480));
    let err = make_camera_model("fov", &params).err().expect("unknown model must fail");
    assert!(
        err.to_string().contains("fov"),
        "error must name the unknown model, got: {err}"
    );
    assert_eq!(err, ModelError::UnknownModel("fov".to_string()));
}

#[test]
fn variant_mismatch_is_rejected_and_names_requested_model() {
    let params = CameraModelParams::Lut(LutParams::default());
    let err = make_camera_model("kb", &params).err().expect("variant mismatch must fail");
    assert!(
        err.to_string().contains("kb"),
        "error must name the requested model, got: {err}"
    );
}

#[test]
fn non_positive_size_is_rejected_and_mentions_width() {
    let params = CameraModelParams::Kb(kb_params(0, 480));
    let err = make_camera_model("kb", &params).err().expect("width 0 must fail");
    assert!(
        err.to_string().contains("width"),
        "error must mention width, got: {err}"
    );
}
