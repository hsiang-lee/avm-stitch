//! Camera view matrices for the offscreen renderer.

use avm_render::ViewSpec;

/// Column-major mat4 * vec4 (the layout uploaded to WGSL).
fn transform(m: &[[f32; 4]; 4], p: [f32; 4]) -> [f32; 4] {
    let mut out = [0.0f32; 4];
    for row in 0..4 {
        out[row] = (0..4).map(|col| m[col][row] * p[col]).sum();
    }
    out
}

#[test]
fn look_at_projects_the_target_to_the_ndc_center() {
    let view = ViewSpec::look_at([0.0, 18.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 60.0, 4.0 / 3.0, 0.1, 100.0);
    let clip = transform(&view.view_proj, [0.0, 0.0, 0.0, 1.0]);
    assert!(clip[3] > 0.0, "target must be in front of the camera");
    let ndc_x = clip[0] / clip[3];
    let ndc_y = clip[1] / clip[3];
    assert!(ndc_x.abs() < 1e-5, "ndc x = {ndc_x}");
    assert!(ndc_y.abs() < 1e-5, "ndc y = {ndc_y}");
    assert!((0.0..=1.0).contains(&(clip[2] / clip[3])), "target must be visible in z");
}

#[test]
fn look_at_places_a_point_behind_the_eye_behind_the_camera() {
    let view = ViewSpec::look_at([0.0, 10.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 60.0, 1.0, 0.1, 100.0);
    let behind = transform(&view.view_proj, [0.0, 20.0, 0.0, 1.0]);
    assert!(behind[3] < 0.0, "w = {}", behind[3]);
}

/// Off-axis points pin the vertical fov and the aspect ratio: at 10 m the
/// half-extents are `tan(30°)`, divided by the aspect ratio on x. With the eye
/// on +y and up +z, world +x lies on the camera's left, so `ndc_x` is negative.
#[test]
fn look_at_scales_off_axis_points_by_fov_and_aspect() {
    let aspect = 4.0 / 3.0;
    let view = ViewSpec::look_at(
        [0.0, 10.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        60.0,
        aspect,
        0.1,
        100.0,
    );

    let half_extent = 1.0 / 30.0f32.to_radians().tan();

    let right = transform(&view.view_proj, [1.0, 0.0, 0.0, 1.0]);
    assert!(right[3] > 0.0, "w = {}", right[3]);
    let ndc_x = right[0] / right[3];
    let expected_x = -half_extent / aspect / 10.0;
    assert!(
        (expected_x - ndc_x).abs() < 1e-5,
        "ndc x = {ndc_x}, expected {expected_x}"
    );

    let above = transform(&view.view_proj, [0.0, 0.0, 1.0, 1.0]);
    assert!(above[3] > 0.0, "w = {}", above[3]);
    let ndc_y = above[1] / above[3];
    let expected_y = half_extent / 10.0;
    assert!(
        (expected_y - ndc_y).abs() < 1e-5,
        "ndc y = {ndc_y}, expected {expected_y}"
    );
}
