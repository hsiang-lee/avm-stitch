use std::path::PathBuf;

use avm_core::bowl::BowlModel;
use avm_core::calibration::Calibration;

fn examples_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../examples")
}

fn load(data_set: &str) -> Calibration {
    let path = examples_dir().join(data_set).join("calibration.json");
    Calibration::load_from_path(path).expect("example calibration must load")
}

#[test]
fn data_4_loads_four_named_kb_cameras() {
    let cal = load("data_4");
    assert_eq!(4, cal.cameras.len());
    let names: Vec<&str> = cal.cameras.iter().map(|c| c.name()).collect();
    assert_eq!(vec!["front", "rear", "left", "right"], names);
    assert_eq!(
        vec!["front.png", "rear.png", "left.png", "right.png"],
        cal.image_files
    );
    for cam in &cal.cameras {
        assert!(cam.is_valid());
        assert_eq!(640, cam.width());
        assert_eq!(480, cam.height());
    }
}

#[test]
fn data_4_keeps_intrinsics_and_extrinsics() {
    let cal = load("data_4");
    let front = &cal.cameras[0];
    let model = front.model().expect("valid model");
    assert_eq!(320.0, model.cx());
    assert_eq!(240.0, model.cy());

    // Independent oracle: recompute the θ-polynomial by hand from the JSON
    // values instead of calling the model twice.
    let theta = 0.1f64.atan();
    let theta2 = theta * theta;
    let (fx, k1, k2, k3, k4) = (
        129.1823390515476,
        -0.1,
        0.05,
        0.01,
        0.02,
    );
    let theta_d = theta
        * (1.0
            + k1 * theta2
            + k2 * theta2 * theta2
            + k3 * theta2 * theta2 * theta2
            + k4 * theta2 * theta2 * theta2 * theta2);
    let uv = model.project_camera_point([0.1, 0.0, 1.0]);
    assert!((320.0 + fx * theta_d - uv[0]).abs() < 1e-9, "u={}", uv[0]);
    assert!((240.0 - uv[1]).abs() < 1e-9);

    let pos = front.extrinsics().position();
    assert!((4.116668167719906 - pos[0]).abs() < 1e-9);
    assert!((-0.053393489601802596 - pos[1]).abs() < 1e-9);
    assert!((0.6873827164324007 - pos[2]).abs() < 1e-9);
}

#[test]
fn data_4_derives_bowl_params_from_the_car_block() {
    let cal = load("data_4");
    let bowl = cal.bowl_params();
    assert_eq!(5.27, cal.car.length);
    assert_eq!(2.17, cal.car.width);
    assert_eq!("rear_axle", cal.car.origin_convention);
    let scale = 5.27 / 5.6;
    assert!((12.6 * scale - bowl.outer_radius()).abs() < 1e-12);
    assert!((0.2 * scale - bowl.grid_step()).abs() < 1e-12);
    assert!((2.9 / 2.0 - bowl.car_center()[0]).abs() < 1e-12);
    assert_eq!(0.0, bowl.car_center()[1]);
}

#[test]
fn data_4_bakes_a_non_empty_bowl_with_visible_vertices() {
    let cal = load("data_4");
    let model = BowlModel::new(&cal.bowl_params(), &cal.cameras);
    assert!(model.vertex_count() > 0);
    assert!(model.triangle_count() > 0);
    assert_eq!(4, model.baked_num_cameras());
    assert!(model.has_visible_vertices());
}

#[test]
fn data_8_loads_eight_large_format_cameras() {
    let cal = load("data_8");
    assert_eq!(8, cal.cameras.len());
    for (i, cam) in cal.cameras.iter().enumerate() {
        assert_eq!(format!("cam_{i}"), cam.name());
        assert_eq!(1920, cam.width());
        assert_eq!(1080, cam.height());
    }
    assert_eq!(48.107573890686034, cal.car.axis_y);
}

#[test]
fn malformed_json_is_rejected() {
    let err = Calibration::load_from_str("{ not json").err().expect("must fail to parse");
    assert!(matches!(
        err,
        avm_core::calibration::CalibrationError::Json(_)
    ));
}

#[test]
fn empty_camera_list_is_rejected() {
    let content = r#"{"car":{"length":5.6,"width":2.2},"cameras":[]}"#;
    let err = Calibration::load_from_str(content).err().expect("no cameras must fail");
    assert!(matches!(
        err,
        avm_core::calibration::CalibrationError::NoCameras
    ));
}

#[test]
fn non_positive_car_size_is_rejected() {
    let content = r#"{"car":{"length":0.0,"width":2.2},"cameras":[
        {"width":640,"height":480,"intrinsics":{},"extrinsics":{"r":[[1,0,0],[0,1,0],[0,0,1]],"t":[0,0,0]}}
    ]}"#;
    let err = Calibration::load_from_str(content).err().expect("zero car length must fail");
    match err {
        avm_core::calibration::CalibrationError::NonPositiveCarSize { length, width } => {
            assert_eq!(0.0, length);
            assert_eq!(2.2, width);
        }
        other => panic!("unexpected error: {other}"),
    }
}

#[test]
fn camera_model_error_names_the_offending_camera() {
    let content = r#"{"car":{"length":5.6,"width":2.2},"cameras":[
        {"name":"broken","width":0,"height":480,"intrinsics":{},"extrinsics":{"r":[[1,0,0],[0,1,0],[0,0,1]],"t":[0,0,0]}}
    ]}"#;
    let err = Calibration::load_from_str(content).err().expect("width 0 must fail");
    let message = err.to_string();
    assert!(message.contains("broken"), "got: {message}");
    assert!(message.contains("width"), "got: {message}");
}

#[test]
fn missing_file_reports_the_path() {
    let missing = examples_dir().join("data_4").join("does_not_exist.json");
    let err = Calibration::load_from_path(&missing).err().expect("missing file must fail");
    let message = err.to_string();
    assert!(message.contains("does_not_exist.json"), "got: {message}");
}
