mod common;

use avm_core::angle_utils::PI;
use avm_core::camera::{make_camera_model, Camera, CameraModel, CameraModelParams, ModelError};
use common::{
    arr, assert_batch_matches_single, identity_extrinsics, kb_intrinsics, kb_intrinsics_with_focal,
    look_at_rotation, make_kb_model, project_vec, real_intrinsics, standard_kb_model, vec3,
};

#[test]
fn identity_camera_projects_axis_point_to_principal_point() {
    let model = standard_kb_model();
    let uv = model.project_camera_point(arr(identity_extrinsics().to_camera(vec3(0.0, 0.0, 1.0))));
    assert!((320.0 - uv[0]).abs() < 1e-6);
    assert!((240.0 - uv[1]).abs() < 1e-6);
}

#[test]
fn identity_camera_projects_off_center_point_with_equidistant_radius() {
    let model = standard_kb_model();
    let uv = model.project_camera_point(arr(identity_extrinsics().to_camera(vec3(1.0, 0.0, 1.0))));
    assert!((320.0 + 300.0 * 1.0f64.atan() - uv[0]).abs() < 1e-6);
    assert!((240.0 - uv[1]).abs() < 1e-6);
}

#[test]
fn rotated_camera_projects_axis_point_off_center() {
    let model = standard_kb_model();
    let c = 3.0f64.sqrt() / 2.0;
    let extr = common::from_rows(
        [[c, 0.0, 0.5], [0.0, 1.0, 0.0], [-0.5, 0.0, c]],
        [0.0, 0.0, 0.0],
    );
    let uv = model.project_camera_point(arr(extr.to_camera(vec3(0.0, 0.0, 1.0))));
    assert!((320.0 - 300.0 * (PI / 6.0) - uv[0]).abs() < 1e-6);
    assert!((240.0 - uv[1]).abs() < 1e-6);
}

#[test]
fn translated_camera_projects_point_at_camera_origin_to_center() {
    let model = standard_kb_model();
    let extr = common::from_rows([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], [1.0, 0.0, 0.0]);
    let uv = model.project_camera_point(arr(extr.to_camera(vec3(1.0, 0.0, 1.0))));
    assert!((320.0 - uv[0]).abs() < 1e-6);
    assert!((240.0 - uv[1]).abs() < 1e-6);
}

#[test]
fn point_at_camera_z_axis_projects_to_center() {
    let model = standard_kb_model();
    let uv = model.project_camera_point(arr(identity_extrinsics().to_camera(vec3(0.0, 0.0, 2.0))));
    assert!((320.0 - uv[0]).abs() < 1e-6);
    assert!((240.0 - uv[1]).abs() < 1e-6);
}

#[test]
fn large_distortion_k1_follows_the_theta_polynomial() {
    let model = make_kb_model(
        640, 480, 300.0, 300.0, 320.0, 240.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    );
    let uv = model.project_camera_point(arr(identity_extrinsics().to_camera(vec3(0.5, 0.0, 1.0))));
    assert!(uv[0].is_finite());
    assert!(uv[1].is_finite());
    let theta = 0.5f64.atan();
    let theta_d = theta * (1.0 + theta * theta);
    assert!((320.0 + 300.0 * theta_d - uv[0]).abs() < 1e-6);
    assert!((240.0 - uv[1]).abs() < 1e-6);
}

#[test]
fn point_behind_camera_still_returns_finite_uv() {
    let model = standard_kb_model();
    let axis = model.project_camera_point(arr(identity_extrinsics().to_camera(vec3(0.0, 0.0, -1.0))));
    assert!((320.0 - axis[0]).abs() < 1e-6);
    assert!((240.0 - axis[1]).abs() < 1e-6);

    let off_axis =
        model.project_camera_point(arr(identity_extrinsics().to_camera(vec3(1.0, 0.0, -1.0))));
    assert!(off_axis[0] > 320.0);
    assert!((240.0 - off_axis[1]).abs() < 1e-6);
}

#[test]
fn inside_fisheye_circle_uses_inclusive_auto_radius() {
    let model = standard_kb_model();
    assert!(model.inside_fisheye_circle(320.0 + 240.0, 240.0));
    assert!(!model.inside_fisheye_circle(320.0 + 240.0 + 1e-6, 240.0));
}

#[test]
fn edge_feather_hard_gate_excludes_the_rim_band() {
    let model = standard_kb_model();
    assert_eq!(0.0, model.edge_feather(320.0, 240.0 + 230.0, 40.0));
}

#[test]
fn edge_feather_non_positive_width_disables_the_gate() {
    let model = standard_kb_model();
    assert!((1.0 - model.edge_feather(320.0, 240.0 + 230.0, 0.0)).abs() < 1e-12);
    assert!((1.0 - model.edge_feather(320.0 + 239.0, 240.0, -5.0)).abs() < 1e-12);
}

#[test]
fn kb_factory_batch_projects_axis_point_exactly() {
    let params = CameraModelParams::Kb(kb_intrinsics(0.0, 0.0, 0.0, 0.0));
    let model = make_camera_model("kb", &params).expect("kb model");
    let (uv, valid) = project_vec(model.as_ref(), &[[0.0, 0.0, 1.0]]);
    assert_eq!(320.0, uv[0][0]);
    assert_eq!(240.0, uv[0][1]);
    assert_eq!(1, valid[0]);
}

#[test]
fn kb_factory_batch_projects_off_axis_point_without_distortion() {
    let params = CameraModelParams::Kb(kb_intrinsics(0.0, 0.0, 0.0, 0.0));
    let model = make_camera_model("kb", &params).expect("kb model");
    let (uv, valid) = project_vec(model.as_ref(), &[[1.0, 0.0, 1.0]]);
    assert!((320.0 + 300.0 * 1.0f64.atan() - uv[0][0]).abs() < 1e-9);
    assert_eq!(240.0, uv[0][1]);
    assert_eq!(1, valid[0]);
}

#[test]
fn kb_batch_matches_single_point_projection_with_distortion() {
    const PTS: [[f64; 3]; 10] = [
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 1.0],
        [0.0, 1.0, 1.0],
        [0.5, 0.5, 1.0],
        [1.0, -1.0, 0.5],
        [-2.0, 1.0, 2.0],
        [0.3, 0.2, -0.9327],
        [1.0, 1.0, -1.0],
        [2.0, -1.0, -0.5],
        [0.1, 0.05, 0.05],
    ];
    let params = CameraModelParams::Kb(kb_intrinsics(0.1, -0.05, 0.01, 0.002));
    let model = make_camera_model("kb", &params).expect("kb model");
    let (uv, _valid) = project_vec(model.as_ref(), &PTS);

    let extr = identity_extrinsics();
    for (i, p) in PTS.iter().enumerate() {
        let reference = model.project_camera_point(arr(extr.to_camera(vec3(p[0], p[1], p[2]))));
        assert!((reference[0] - uv[i][0]).abs() < 1e-9, "point {i} u mismatch");
        assert!((reference[1] - uv[i][1]).abs() < 1e-9, "point {i} v mismatch");
    }
}

#[test]
fn kb_snaps_degenerate_rxy_to_principal_point() {
    let params = CameraModelParams::Kb(kb_intrinsics(0.0, 0.0, 0.0, 0.0));
    let model = make_camera_model("kb", &params).expect("kb model");
    let (uv, valid) = project_vec(model.as_ref(), &[[0.0, 0.0, -1.0]]);
    assert_eq!(320.0, uv[0][0]);
    assert_eq!(240.0, uv[0][1]);
    assert_eq!(1, valid[0]);
}

#[test]
fn kb_point_outside_image_rectangle_is_invalid() {
    let params = CameraModelParams::Kb(kb_intrinsics_with_focal(0.0, 0.0, 0.0, 0.0, 3000.0, 3000.0));
    let model = make_camera_model("kb", &params).expect("kb model");
    let (uv, valid) = project_vec(model.as_ref(), &[[1.0, 0.0, 1.0]]);
    assert_eq!(0, valid[0]);
    assert!(uv[0][0] >= 640.0 || uv[0][0] < 0.0);
}

#[test]
fn kb_batch_matches_single_calls_and_rejects_length_mismatch() {
    let intr = kb_intrinsics(0.1, -0.05, 0.01, 0.002);
    let params = CameraModelParams::Kb(intr.clone());
    let model = make_camera_model("kb", &params).expect("kb model");

    let pts: Vec<[f64; 3]> = (0..100)
        .map(|i| {
            let t = i as f64 * 0.63;
            [
                t.sin(),
                (t * 0.7).cos(),
                t.cos() * if i % 2 == 0 { 1.0 } else { -1.0 },
            ]
        })
        .collect();
    let (uv_batch, valid_batch) = project_vec(model.as_ref(), &pts);

    assert_batch_matches_single(model.as_ref(), &pts);
    for (i, uv) in uv_batch.iter().enumerate() {
        let in_rect = uv[0] >= 0.0
            && uv[0] < intr.width as f64
            && uv[1] >= 0.0
            && uv[1] < intr.height as f64;
        assert_eq!(u8::from(in_rect), valid_batch[i], "point {i}");
    }

    let mut short_uv = vec![[0.0f64; 2]; pts.len() - 1];
    let mut valid = vec![0u8; pts.len()];
    let err = model
        .project_points(&pts, &mut short_uv, &mut valid)
        .expect_err("short uv slice must be rejected");
    assert_eq!(ModelError::LengthMismatch, err);
}

#[test]
fn kb_explicit_fisheye_radius_is_forwarded_to_the_gate() {
    let mut p = kb_intrinsics(0.0, 0.0, 0.0, 0.0);
    p.fisheye_radius = 300.0;
    let model = make_camera_model("kb", &CameraModelParams::Kb(p)).expect("kb model");
    assert!((300.0 - model.fisheye_circle_radius()).abs() < 1e-9);
}

#[test]
fn kb_zero_radius_uses_the_auto_inscribed_circle() {
    let model = make_camera_model("kb", &CameraModelParams::Kb(kb_intrinsics(0.0, 0.0, 0.0, 0.0)))
        .expect("kb model");
    assert!((240.0 - model.fisheye_circle_radius()).abs() < 1e-9);
}

#[test]
fn gating_matches_hand_formula_bit_exact_for_model_and_camera() {
    let mut p = kb_intrinsics(0.0, 0.0, 0.0, 0.0);
    p.fisheye_radius = 300.0;
    let model = make_camera_model("kb", &CameraModelParams::Kb(p.clone())).expect("kb model");
    let cam = Camera::new(identity_extrinsics(), Box::new(make_kb_model(
        p.width, p.height, p.fx, p.fy, p.cx, p.cy, p.k1, p.k2, p.k3, p.k4, p.fisheye_radius,
    )));

    let (cx, cy) = (320.0, 240.0);
    let radius = 300.0f64;
    let inside = |u: f64, v: f64| (u - cx).hypot(v - cy) <= radius;
    let feather = |u: f64, v: f64| -> f64 {
        let r_px = (u - cx).hypot(v - cy);
        if radius - r_px >= 40.0 {
            1.0
        } else {
            0.0
        }
    };

    let samples = [
        [320.0, 240.0],
        [320.0, 340.0],
        [420.0, 300.0],
        [560.0, 400.0],
        [620.0, 460.0],
        [320.0, 240.0 + 190.0],
    ];
    for uv in samples {
        let circle_expected = inside(uv[0], uv[1]);
        assert_eq!(circle_expected, model.inside_fisheye_circle(uv[0], uv[1]), "{uv:?}");
        assert_eq!(circle_expected, cam.inside_fisheye_circle(uv[0], uv[1]), "{uv:?}");
        let feather_expected = feather(uv[0], uv[1]).to_bits();
        assert_eq!(
            feather_expected,
            model.edge_feather(uv[0], uv[1], 40.0).to_bits(),
            "{uv:?}"
        );
        assert_eq!(
            feather_expected,
            cam.edge_feather(uv[0], uv[1], 40.0).to_bits(),
            "{uv:?}"
        );
    }
}

#[test]
fn look_at_extrinsics_keeps_projection_finite() {
    let model = make_kb_model(
        real_intrinsics().width,
        real_intrinsics().height,
        real_intrinsics().fx,
        real_intrinsics().fy,
        real_intrinsics().cx,
        real_intrinsics().cy,
        0.215,
        -0.05,
        0.003,
        0.0,
        0.0,
    );
    let pos = vec3(3.84, -0.016, 0.66);
    let extr = avm_core::camera::CameraExtrinsics::new(
        look_at_rotation(pos, vec3(0.0, 0.0, 0.2)),
        pos,
    );
    let uv = model.project_camera_point(arr(extr.to_camera(vec3(0.0, 0.0, 0.2))));
    assert!(uv[0].is_finite() && uv[1].is_finite());
    assert!(uv[0] > 0.0 && uv[0] < real_intrinsics().width as f64);
    assert!(uv[1] > 0.0 && uv[1] < real_intrinsics().height as f64);
}

/// Truth anchor: the expected pixels come from OpenCV 4.12
/// `cv::fisheye::projectPoints` run once outside the repository, so the crate
/// keeps its zero-OpenCV dependency and the constants must not be regenerated
/// from this model. Of the six points only the widest-field one departs from a
/// plain pinhole projection (by ~69 px); the other five are within 1 px, and the
/// `1e-6` tolerance still rejects a `1e-3` drift of the distortion polynomial.
#[test]
fn matches_opencv_fisheye_reference() {
    let model = make_kb_model(
        1920,
        1536,
        441.45,
        441.45,
        952.14,
        758.27,
        0.215,
        -0.05,
        0.003,
        0.0,
        0.0,
    );
    let pos = vec3(3.84, -0.016, 0.66);
    let extr =
        avm_core::camera::CameraExtrinsics::new(look_at_rotation(pos, vec3(0.0, 0.0, 0.2)), pos);

    let world_points = [
        [0.0, 0.0, 0.2],
        [1.0, 0.0, 0.2],
        [0.0, 1.0, 0.2],
        [2.0, 2.0, 0.5],
        [-2.0, -1.0, 0.8],
        [3.0, 0.0, 0.6],
    ];
    // External ground truth from `cv::fisheye::projectPoints` (OpenCV 4.12).
    let opencv_uv = [
        [952.14, 758.27],
        [951.5002625473055, 740.0082672371517],
        [839.0476859731364, 758.3260467989531],
        [542.6463204259896, 770.7043920967401],
        [1028.701970724108, 821.528992734919],
        [945.5824447841555, 779.4342024164037],
    ];

    let camera_points: Vec<[f64; 3]> = world_points
        .iter()
        .map(|p| arr(extr.to_camera(vec3(p[0], p[1], p[2]))))
        .collect();
    let (uv, valid) = project_vec(&model, &camera_points);

    for (i, expected) in opencv_uv.iter().enumerate() {
        assert_eq!(1, valid[i], "point {i} must project inside the image");
        assert!(
            (expected[0] - uv[i][0]).abs() < 1e-6,
            "point {i}: u = {} vs OpenCV {}",
            uv[i][0],
            expected[0]
        );
        assert!(
            (expected[1] - uv[i][1]).abs() < 1e-6,
            "point {i}: v = {} vs OpenCV {}",
            uv[i][1],
            expected[1]
        );
    }
}
