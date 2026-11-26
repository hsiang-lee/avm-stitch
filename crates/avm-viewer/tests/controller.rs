//! The controller is pure math, so these run without a display or a GPU.

use avm_core::bowl::{BowlParams, ClipPlanes, ViewPose};
use avm_viewer::OrbitController;

/// Local copies of the per-pixel sensitivities: a change in the production
/// constants must fail these tests instead of moving the expectation with it.
const ROTATE_DEG_PER_PIXEL: f64 = 0.005 * (180.0 / std::f64::consts::PI);
const ZOOM_PER_PIXEL: f64 = 0.01;

const HOME_FOVY_DEG: f64 = 60.0;
const HOME_NEAR: f64 = 2.0;
const HOME_FAR: f64 = 50.0;
const VIEWPORT_HEIGHT: u32 = 720;

fn render_basis(p: [f64; 3]) -> [f64; 3] {
    [p[0], p[2], -p[1]]
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn scale(v: [f64; 3], s: f64) -> [f64; 3] {
    [v[0] * s, v[1] * s, v[2] * s]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn norm(v: [f64; 3]) -> f64 {
    dot(v, v).sqrt()
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn normalize(v: [f64; 3]) -> [f64; 3] {
    let n = norm(v);
    if n > 0.0 {
        scale(v, 1.0 / n)
    } else {
        v
    }
}

fn close(a: [f64; 3], b: [f64; 3], eps: f64) -> bool {
    norm(sub(a, b)) < eps
}

fn clip(near: f64, far: f64) -> ClipPlanes {
    ClipPlanes { near, far }
}

/// One fixture for the calibrated car used by every camera test.
fn bowl() -> BowlParams {
    BowlParams::for_car_size(5.6, 2.2, 2.9)
}

fn home_pose() -> ViewPose {
    bowl().home_view_pose()
}

fn home_controller() -> OrbitController {
    OrbitController::from_pose(&home_pose(), clip(HOME_NEAR, HOME_FAR))
}

/// The camera state an assertion needs before an action: eye, target, distance.
fn snapshot(controller: &OrbitController) -> ([f64; 3], [f64; 3], f64) {
    (controller.eye(), controller.target(), controller.distance())
}

/// Column-major mat4 * vec4 (the layout uploaded to WGSL), returning NDC.
fn ndc(m: &[[f32; 4]; 4], p: [f64; 3]) -> (f64, f64, f64) {
    let p = [p[0] as f32, p[1] as f32, p[2] as f32, 1.0];
    let mut out = [0.0f32; 4];
    for row in 0..4 {
        out[row] = (0..4).map(|col| m[col][row] * p[col]).sum();
    }
    (
        f64::from(out[0] / out[3]),
        f64::from(out[1] / out[3]),
        f64::from(out[2] / out[3]),
    )
}

#[test]
fn home_camera_matches_the_home_view_pose() {
    let bowl = bowl();
    let pose = bowl.home_view_pose();
    let controller = OrbitController::from_home(&bowl);

    let eye = render_basis(pose.position);
    let target = render_basis(pose.target);
    assert!(close(controller.eye(), eye, 1e-12));
    assert!(close(controller.target(), target, 1e-12));
    assert!(close(controller.up(), [0.0, 1.0, 0.0], 1e-12));

    let distance = norm(sub(eye, target));
    assert!((controller.distance() - distance).abs() < 1e-12);
    let expected = bowl.clip_planes_for(distance);
    assert!((controller.clip().near - expected.near).abs() < 1e-12);
    assert!((controller.clip().far - expected.far).abs() < 1e-12);
}

#[test]
fn rotate_keeps_the_target_and_the_distance() {
    let mut controller = home_controller();
    let (eye, target, distance) = snapshot(&controller);

    controller.rotate(240.0, -120.0);

    assert!(close(controller.target(), target, 1e-12));
    assert!((controller.distance() - distance).abs() < 1e-12);
    assert!(!close(controller.eye(), eye, 1e-6), "rotation must move the eye");
}

#[test]
fn rotate_clamps_pitch_and_never_flips_the_camera() {
    let mut controller = home_controller();

    controller.rotate(0.0, 1.0e6);
    assert!((controller.pitch_deg() - 89.0).abs() < 1e-9);
    controller.rotate(0.0, -1.0e6);
    assert!((controller.pitch_deg() + 89.0).abs() < 1e-9);
}

#[test]
fn zoom_in_stops_at_the_near_plane() {
    let mut controller = home_controller();

    controller.zoom(1.0e6);

    assert!((controller.distance() - HOME_NEAR).abs() < 1e-9);
}

#[test]
fn zoom_out_stops_at_the_far_plane() {
    let mut controller = home_controller();

    controller.zoom(-1.0e6);

    assert!((controller.distance() - HOME_FAR).abs() < 1e-9);
}

#[test]
fn zoom_changes_the_distance_but_not_the_target() {
    let mut controller = home_controller();
    let (_, target, before) = snapshot(&controller);

    controller.zoom(100.0);

    assert!(close(controller.target(), target, 1e-12));
    assert!(controller.distance() < before);
}

#[test]
fn pan_moves_eye_and_target_by_the_same_vector() {
    let mut controller = home_controller();
    let (eye_before, target_before, distance) = snapshot(&controller);

    controller.pan(120.0, -45.0, VIEWPORT_HEIGHT);

    let eye_delta = sub(controller.eye(), eye_before);
    let target_delta = sub(controller.target(), target_before);
    assert!(close(eye_delta, target_delta, 1e-9));
    assert!(norm(target_delta) > 1e-6, "pan must move the camera");
    assert!((controller.distance() - distance).abs() < 1e-9);
}

/// `eye() - target` is structural, so the direction of the shift needs its own
/// assertion: `+dx` must move the target against the camera's right axis and
/// `dy` along its up axis.
#[test]
fn pan_follows_the_camera_right_and_up_axes() {
    let mut controller = home_controller();
    let (eye_before, target_before, distance) = snapshot(&controller);
    let forward = normalize(sub(target_before, eye_before));
    let right = normalize(cross(forward, controller.up()));
    let camera_up = cross(right, forward);
    let world_per_pixel = 2.0
        * distance
        * (0.5 * HOME_FOVY_DEG.to_radians()).tan()
        / f64::from(VIEWPORT_HEIGHT);

    controller.pan(120.0, -45.0, VIEWPORT_HEIGHT);

    let expected = add(
        scale(right, -120.0 * world_per_pixel),
        scale(camera_up, -45.0 * world_per_pixel),
    );
    let delta = sub(controller.target(), target_before);
    assert!(
        close(delta, expected, 1e-9),
        "pan moved {delta:?}, expected {expected:?}"
    );
}

#[test]
fn drag_mode_maps_left_right_and_middle() {
    use avm_viewer::controller::{drag_mode, DragMode};

    assert_eq!(Some(DragMode::Rotate), drag_mode(true, false, false));
    assert_eq!(Some(DragMode::Zoom), drag_mode(false, true, false));
    assert_eq!(Some(DragMode::Pan), drag_mode(false, false, true));
    assert_eq!(None, drag_mode(false, false, false));
    assert_eq!(Some(DragMode::Rotate), drag_mode(true, true, true));
    // Middle loses to right, the documented left > right > middle priority.
    assert_eq!(Some(DragMode::Zoom), drag_mode(false, true, true));
}

#[test]
fn left_drag_rotates_and_keeps_the_distance() {
    use avm_viewer::controller::DragMode;

    let mut controller = home_controller();
    let (eye, target, distance) = snapshot(&controller);

    controller.drag(DragMode::Rotate, 90.0, 0.0, VIEWPORT_HEIGHT);

    assert!(close(controller.target(), target, 1e-12));
    assert!((controller.distance() - distance).abs() < 1e-12);
    assert!(!close(controller.eye(), eye, 1e-6));
}

/// A drag to the right must orbit the camera towards decreasing yaw; a flipped
/// sign keeps the eye moving, so only the angle itself can pin the direction.
#[test]
fn horizontal_drag_decreases_yaw() {
    let mut controller = home_controller();
    let yaw = controller.yaw_deg();

    controller.rotate(100.0, 0.0);

    assert!(controller.yaw_deg() < yaw, "dragging right must decrease yaw");
    assert!(
        (controller.yaw_deg() - (yaw - 100.0 * ROTATE_DEG_PER_PIXEL)).abs() < 1e-9,
        "yaw = {}",
        controller.yaw_deg()
    );
}

/// `rotate(dx, dy)` must feed the horizontal axis to yaw and the vertical axis
/// to pitch; a single-axis drag cannot tell the two apart.
#[test]
fn rotate_moves_yaw_and_pitch_independently() {
    use avm_viewer::controller::DragMode;

    let mut yaw_reference = home_controller();
    yaw_reference.rotate(90.0, 0.0);
    let mut pitch_reference = home_controller();
    pitch_reference.rotate(0.0, 40.0);

    let mut controller = home_controller();
    controller.drag(DragMode::Rotate, 90.0, 40.0, VIEWPORT_HEIGHT);

    assert!((controller.yaw_deg() - yaw_reference.yaw_deg()).abs() < 1e-9);
    assert!((controller.pitch_deg() - pitch_reference.pitch_deg()).abs() < 1e-9);
}

#[test]
fn right_drag_zooms_without_moving_the_target() {
    use avm_viewer::controller::DragMode;

    let mut controller = home_controller();
    let (_, target, distance) = snapshot(&controller);

    controller.drag(DragMode::Zoom, 0.0, -100.0, VIEWPORT_HEIGHT);

    assert!(close(controller.target(), target, 1e-12));
    assert!(controller.distance() < distance);
}

#[test]
fn middle_drag_pans_and_keeps_the_distance() {
    use avm_viewer::controller::DragMode;

    let mut controller = home_controller();
    let (eye, target, distance) = snapshot(&controller);

    controller.drag(DragMode::Pan, 80.0, -30.0, VIEWPORT_HEIGHT);

    assert!(norm(sub(controller.target(), target)) > 1e-6);
    assert!(close(sub(controller.eye(), eye), sub(controller.target(), target), 1e-9));
    assert!((controller.distance() - distance).abs() < 1e-9);
}

/// Only saturation and `<` comparisons used to cover the sensitivities, so a
/// tenfold change stayed green. Pin the per-pixel amounts numerically.
#[test]
fn rotate_sensitivity_is_five_milliradians_per_pixel() {
    let mut controller = home_controller();
    let yaw = controller.yaw_deg();
    let pitch = controller.pitch_deg();

    controller.rotate(60.0, 30.0);

    assert!((controller.yaw_deg() - (yaw - 60.0 * ROTATE_DEG_PER_PIXEL)).abs() < 1e-9);
    assert!((controller.pitch_deg() - (pitch + 30.0 * ROTATE_DEG_PER_PIXEL)).abs() < 1e-9);
}

#[test]
fn zoom_sensitivity_is_one_percent_per_pixel() {
    let mut controller = home_controller();
    let distance = controller.distance();

    controller.zoom(10.0);

    assert!(
        (controller.distance() - distance * (-10.0 * ZOOM_PER_PIXEL).exp()).abs() < 1e-12,
        "distance = {}",
        controller.distance()
    );
}

/// `view_spec` connects the controller to the projection matrix; without this
/// test its fovy, aspect and clip planes could be transposed or swapped.
#[test]
fn view_spec_pins_fovy_aspect_and_clip_planes() {
    let controller = home_controller();
    let aspect: f64 = 4.0 / 3.0;
    let view = controller.view_spec(aspect as f32);

    let eye = controller.eye();
    let target = controller.target();
    let forward = normalize(sub(target, eye));
    let right = normalize(cross(forward, controller.up()));
    let camera_up = cross(right, forward);

    let center = ndc(&view.view_proj, target);
    assert!(
        center.0.abs() < 1e-5 && center.1.abs() < 1e-5,
        "target must project to the centre, got {center:?}"
    );

    let distance = controller.distance();
    let f = 1.0 / (0.5 * HOME_FOVY_DEG.to_radians()).tan();
    let up_ndc = ndc(&view.view_proj, add(target, camera_up));
    assert!(
        (up_ndc.1 - f / distance).abs() < 1e-5,
        "fovy is not {HOME_FOVY_DEG} deg: ndc y = {}",
        up_ndc.1
    );
    let right_ndc = ndc(&view.view_proj, add(target, right));
    assert!(
        (right_ndc.0 - (f / aspect) / distance).abs() < 1e-5,
        "aspect is not {aspect}: ndc x = {}",
        right_ndc.0
    );

    let at_near = ndc(&view.view_proj, add(eye, scale(forward, HOME_NEAR)));
    let at_far = ndc(&view.view_proj, add(eye, scale(forward, HOME_FAR)));
    assert!(at_near.2.abs() < 1e-4, "near plane ndc z = {}", at_near.2);
    assert!(
        (at_far.2 - 1.0).abs() < 1e-4,
        "far plane ndc z = {}",
        at_far.2
    );
}
