//! Port of `tests/camera_model/test_ocam_camera_model.cpp`.

mod common;

use avm_core::camera::{make_camera_model, CameraModel, CameraModelParams, ModelError, OcamParams};
use common::{assert_batch_matches_single, project_vec};

fn fish_back_params() -> OcamParams {
    OcamParams {
        pol: vec![
            -319.5284,
            0.0,
            8.991405e-04,
            -7.701385e-07,
            1.365101e-09,
        ],
        invpol: vec![
            589.4893,
            428.471,
            -6.866877,
            8.354607,
            75.49948,
            -5.837637,
            -54.56404,
            24.77632,
            70.42449,
            37.77541,
            6.678202,
        ],
        cx: 643.0012,
        cy: 480.3732,
        c: 0.9999853,
        d: 1.297014e-05,
        e: 1.772778e-05,
        width: 1280,
        height: 960,
    }
}

fn fish_back_model() -> Box<dyn CameraModel> {
    make_camera_model("ocam", &CameraModelParams::Ocam(fish_back_params())).expect("ocam model")
}

#[test]
fn axis_point_projects_to_principal_point() {
    let model = fish_back_model();
    let (uv, valid) = project_vec(model.as_ref(), &[[0.0, 0.0, 1.0]]);
    assert_eq!(643.0012, uv[0][0]);
    assert_eq!(480.3732, uv[0][1]);
    assert_eq!(1, valid[0]);
}

#[test]
fn reference_point_matches_the_renderer_kernel() {
    let model = fish_back_model();
    let n = (0.3f64 * 0.3 + 0.2 * 0.2 + 0.9327 * 0.9327).sqrt();
    let (uv, valid) = project_vec(model.as_ref(), &[[0.3 / n, 0.2 / n, 0.9327 / n]]);
    assert!((742.00773299911305 - uv[0][0]).abs() < 1e-6);
    assert!((546.37971005202451 - uv[0][1]).abs() < 1e-6);
    assert_eq!(1, valid[0]);
}

#[test]
fn ocam_round_trip_closes_under_0_01_px() {
    let model = fish_back_model();
    let params = fish_back_params();
    let pixels = [
        [643.0, 480.4],
        [700.0, 500.0],
        [580.0, 430.0],
        [900.0, 700.0],
        [400.0, 200.0],
        [1100.0, 850.0],
        [300.0, 600.0],
        [750.0, 300.0],
        [650.0, 900.0],
        [1000.0, 100.0],
    ];
    for px in pixels {
        let invdet = 1.0 / (params.c - params.d * params.e);
        let mx = px[0] - params.cx;
        let my = px[1] - params.cy;
        let xp = invdet * (mx - params.d * my);
        let yp = invdet * (-params.e * mx + params.c * my);
        let mut zp = params.pol[0];
        let mut r_i = 1.0;
        for i in 1..params.pol.len() {
            r_i *= (xp * xp + yp * yp).sqrt();
            zp += r_i * params.pol[i];
        }
        let (uv, _valid) = project_vec(model.as_ref(), &[[xp, yp, -zp]]);
        assert!((uv[0][0] - px[0]).abs() < 0.01, "pixel {px:?} u");
        assert!((uv[0][1] - px[1]).abs() < 0.01, "pixel {px:?} v");
    }
}

#[test]
fn behind_camera_is_invalid() {
    let model = fish_back_model();
    let (_, valid) = project_vec(model.as_ref(), &[[0.5, 0.0, -0.866]]);
    assert_eq!(0, valid[0]);
}

#[test]
fn empty_invpol_is_rejected() {
    let mut p = fish_back_params();
    p.invpol.clear();
    let err = make_camera_model("ocam", &CameraModelParams::Ocam(p))
        .err().expect("empty invpol must fail");
    assert_eq!(ModelError::OcamEmptyPolynomials, err);
}

#[test]
fn non_positive_size_is_rejected_and_mentions_width() {
    let mut p = fish_back_params();
    p.width = 0;
    let err = make_camera_model("ocam", &CameraModelParams::Ocam(p))
        .err().expect("width 0 must fail");
    assert!(
        err.to_string().contains("width"),
        "error must mention width, got: {err}"
    );
}

#[test]
fn batch_projection_matches_single_calls_and_rejects_length_mismatch() {
    let model = fish_back_model();
    let pts: Vec<[f64; 3]> = (0..30)
        .map(|i| {
            let t = i as f64 * 0.63;
            [t.sin(), (t * 0.7).cos(), t.cos().abs() + 1e-6]
        })
        .collect();
    assert_batch_matches_single(model.as_ref(), &pts);

    let mut short_uv = vec![[0.0f64; 2]; pts.len() - 1];
    let mut valid = vec![0u8; pts.len()];
    let err = model
        .project_points(&pts, &mut short_uv, &mut valid)
        .err().expect("short uv slice must be rejected");
    assert_eq!(ModelError::LengthMismatch, err);
}
