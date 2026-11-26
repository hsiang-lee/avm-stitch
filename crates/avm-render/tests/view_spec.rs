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

#[test]
fn fov_and_aspect_are_consumed_by_the_projection() {
    // f = 1/tan(fov_y/2) = 1/tan(30 deg); ndc_x = (f/aspect) * x / depth.
    let view = ViewSpec::look_at(
        [0.0, 10.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        60.0,
        4.0 / 3.0,
        0.1,
        100.0,
    );
    let f = 1.0f32 / 30.0f32.to_radians().tan();
    let aspect = 4.0f32 / 3.0;

    // World +x is on the camera's left (eye looks down -y with up +z).
    let clip = transform(&view.view_proj, [1.0, 0.0, 0.0, 1.0]);
    let ndc_x = clip[0] / clip[3];
    let expected_x = -(f / aspect) / 10.0;
    assert!((ndc_x - expected_x).abs() < 1e-5, "ndc_x = {ndc_x}, expected {expected_x}");

    let clip = transform(&view.view_proj, [0.0, 0.0, 1.0, 1.0]);
    let ndc_y = clip[1] / clip[3];
    let expected_y = f / 10.0;
    assert!((ndc_y - expected_y).abs() < 1e-5, "ndc_y = {ndc_y}, expected {expected_y}");
}

/// wgpu clip space is `0 <= z <= w` (not the OpenGL `-w <= z <= w`).
#[test]
fn depth_maps_near_to_zero_and_far_to_one() {
    let (near, far) = (0.1f32, 100.0f32);
    let view = ViewSpec::look_at(
        [0.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
        [0.0, 1.0, 0.0],
        60.0,
        1.0,
        near,
        far,
    );

    for distance in [near, near * 1.0001, 0.5, 1.0, 10.0, 99.9, far] {
        let clip = transform(&view.view_proj, [0.0, 0.0, -distance, 1.0]);
        assert!(
            clip[2] >= 0.0,
            "distance {distance}: clip.z = {} is below 0 (GL convention?)",
            clip[2]
        );
        assert!(
            clip[2] <= clip[3],
            "distance {distance}: clip.z = {} exceeds clip.w = {}",
            clip[2],
            clip[3]
        );
        let ndc_z = clip[2] / clip[3];
        assert!(
            (0.0..=1.0).contains(&ndc_z),
            "distance {distance}: ndc z = {ndc_z}"
        );
    }

    let at_near = transform(&view.view_proj, [0.0, 0.0, -near, 1.0]);
    assert!(
        (at_near[2] / at_near[3]).abs() < 1e-4,
        "near plane must map to ndc 0, got {}",
        at_near[2] / at_near[3]
    );
    let at_far = transform(&view.view_proj, [0.0, 0.0, -far, 1.0]);
    assert!(
        (at_far[2] / at_far[3] - 1.0).abs() < 1e-4,
        "far plane must map to ndc 1, got {}",
        at_far[2] / at_far[3]
    );
}
