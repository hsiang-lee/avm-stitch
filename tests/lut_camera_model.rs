mod common;

use avm_stitch::camera::{
    make_camera_model, CameraModel, CameraModelParams, LutParams, ModelError,
};
use common::{assert_batch_matches_single, project_vec};

const THETA_30_HEIGHT_MM: f64 = 0.499038711;
const THETA_30_POINT1_HEIGHT_MM: f64 = 0.500792632;
const THETA_100_HEIGHT_MM: f64 = 1.949773294;
const PIXEL_MM: f64 = 0.003;

const WIDTH: i32 = 1280;
const HEIGHT: i32 = 960;
const CX: f64 = 640.0;
const CY: f64 = 480.0;

const CONE_X: f64 = 0.5;
const CONE_Z: f64 = 0.8660254037844386;

fn lut_params() -> LutParams {
    LutParams {
        points: vec![
            [30.0, THETA_30_HEIGHT_MM],
            [30.1, THETA_30_POINT1_HEIGHT_MM],
            [100.0, THETA_100_HEIGHT_MM],
        ],
        pixel_size_mm: PIXEL_MM,
        width: WIDTH,
        height: HEIGHT,
        cx: CX,
        cy: CY,
    }
}

fn lut_model(params: LutParams) -> Box<dyn CameraModel> {
    make_camera_model("table", &CameraModelParams::Lut(params)).expect("table model")
}

#[test]
fn axis_point_projects_to_principal_point() {
    let model = lut_model(lut_params());
    let (uv, valid) = project_vec(model.as_ref(), &[[0.0, 0.0, 1.0]]);
    assert_eq!(CX, uv[0][0]);
    assert_eq!(CY, uv[0][1]);
    assert_eq!(1, valid[0]);
}

#[test]
fn theta_30_lookup_returns_the_table_height_in_pixels() {
    let model = lut_model(lut_params());
    let (uv, valid) = project_vec(model.as_ref(), &[[CONE_X, 0.0, CONE_Z]]);
    assert!(
        (CX + THETA_30_HEIGHT_MM / PIXEL_MM - uv[0][0]).abs() < 1e-6,
        "direction pin: forward lookup must be theta -> r_px, got {}",
        uv[0][0]
    );
    assert!((CY - uv[0][1]).abs() < 1e-6);
    assert_eq!(1, valid[0]);
}

#[test]
fn table_end_clamps_like_the_renderer() {
    let model = lut_model(lut_params());
    let at_end = THETA_100_HEIGHT_MM / PIXEL_MM;
    let pts = [
        [
            (100.0f64 * std::f64::consts::PI / 180.0).sin(),
            0.0,
            (100.0f64 * std::f64::consts::PI / 180.0).cos(),
        ],
        [
            (110.0f64 * std::f64::consts::PI / 180.0).sin(),
            0.0,
            (110.0f64 * std::f64::consts::PI / 180.0).cos(),
        ],
    ];
    let (uv, valid) = project_vec(model.as_ref(), &pts);
    assert!((CX + at_end - uv[0][0]).abs() < 1e-6);
    assert!((CX + at_end - uv[1][0]).abs() < 1e-6, "past-table angle clamps");
    assert_eq!(0, valid[0]);
    assert_eq!(0, valid[1]);
}

#[test]
fn angles_between_rows_interpolate_linearly() {
    let model = lut_model(lut_params());
    let theta = 30.05f64 * std::f64::consts::PI / 180.0;
    let (uv, valid) = project_vec(model.as_ref(), &[[theta.sin(), 0.0, theta.cos()]]);
    let r_mm = 0.5 * (THETA_30_HEIGHT_MM + THETA_30_POINT1_HEIGHT_MM);
    assert!((CX + r_mm / PIXEL_MM - uv[0][0]).abs() < 1e-6);
    assert_eq!(1, valid[0]);
}

#[test]
fn non_ascending_table_is_rejected_and_mentions_ascending() {
    let mut p = lut_params();
    p.points = vec![[30.0, 0.5], [20.0, 0.4]];
    let err = make_camera_model("table", &CameraModelParams::Lut(p))
        .err().expect("non-ascending table must fail");
    assert!(
        err.to_string().contains("ascending"),
        "error must mention ascending, got: {err}"
    );
    assert_eq!(ModelError::LutNotAscending(1), err);
}

#[test]
fn single_point_table_is_rejected() {
    let mut p = lut_params();
    p.points = vec![[30.0, 0.5]];
    let err = make_camera_model("table", &CameraModelParams::Lut(p))
        .err().expect("single point table must fail");
    assert_eq!(ModelError::LutTooFewPoints(1), err);
}

#[test]
fn non_positive_pixel_size_is_rejected() {
    let mut p = lut_params();
    p.pixel_size_mm = 0.0;
    let err = make_camera_model("table", &CameraModelParams::Lut(p))
        .err().expect("zero pixel size must fail");
    assert_eq!(ModelError::NonPositivePixelSize(0.0), err);
}

#[test]
fn non_positive_size_is_rejected_and_mentions_width() {
    let mut p = lut_params();
    p.width = 0;
    let err = make_camera_model("table", &CameraModelParams::Lut(p))
        .err().expect("width 0 must fail");
    assert!(
        err.to_string().contains("width"),
        "error must mention width, got: {err}"
    );
}

#[test]
fn batch_projection_matches_single_point_calls() {
    let model = lut_model(lut_params());
    let pts: Vec<[f64; 3]> = (0..50)
        .map(|i| {
            let t = i as f64 * 0.63;
            [
                t.sin(),
                (t * 0.7).cos(),
                t.cos() * if i % 2 == 0 { 1.0 } else { -1.0 },
            ]
        })
        .collect();
    assert_batch_matches_single(model.as_ref(), &pts);
}

#[test]
fn point_outside_image_rectangle_is_invalid() {
    let model = lut_model(lut_params());
    let theta = 100.0f64 * std::f64::consts::PI / 180.0;
    let (uv, valid) = project_vec(model.as_ref(), &[[theta.sin(), 0.0, theta.cos()]]);
    assert_eq!(0, valid[0]);
    assert!(uv[0][0] > WIDTH as f64);
}
