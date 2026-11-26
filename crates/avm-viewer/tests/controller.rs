//! The controller is pure math, so these run without a display or a GPU.

use avm_core::bowl::{BowlParams, ClipPlanes};
use avm_viewer::OrbitController;

fn render_basis(p: [f64; 3]) -> [f64; 3] {
    [p[0], p[2], -p[1]]
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn norm(v: [f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

fn close(a: [f64; 3], b: [f64; 3], eps: f64) -> bool {
    norm(sub(a, b)) < eps
}

fn clip(near: f64, far: f64) -> ClipPlanes {
    ClipPlanes { near, far }
}

#[test]
fn home_camera_matches_the_home_view_pose() {
    let bowl = BowlParams::for_car_size(5.6, 2.2, 2.9);
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
    let pose = BowlParams::for_car_size(5.6, 2.2, 2.9).home_view_pose();
    let mut controller = OrbitController::from_pose(&pose, clip(2.0, 50.0));
    let target = controller.target();
    let distance = controller.distance();
    let eye = controller.eye();

    controller.rotate(240.0, -120.0);

    assert!(close(controller.target(), target, 1e-12));
    assert!((controller.distance() - distance).abs() < 1e-12);
    assert!(!close(controller.eye(), eye, 1e-6), "rotation must move the eye");
}

#[test]
fn rotate_clamps_pitch_and_never_flips_the_camera() {
    let pose = BowlParams::for_car_size(5.6, 2.2, 2.9).home_view_pose();
    let mut controller = OrbitController::from_pose(&pose, clip(2.0, 50.0));

    controller.rotate(0.0, 1.0e6);
    assert!((controller.pitch_deg() - 89.0).abs() < 1e-9);
    controller.rotate(0.0, -1.0e6);
    assert!((controller.pitch_deg() + 89.0).abs() < 1e-9);
}

#[test]
fn zoom_in_stops_at_the_near_plane() {
    let pose = BowlParams::for_car_size(5.6, 2.2, 2.9).home_view_pose();
    let mut controller = OrbitController::from_pose(&pose, clip(2.0, 50.0));

    controller.zoom(1.0e6);

    assert!((controller.distance() - 2.0).abs() < 1e-9);
}

#[test]
fn zoom_out_stops_at_the_far_plane() {
    let pose = BowlParams::for_car_size(5.6, 2.2, 2.9).home_view_pose();
    let mut controller = OrbitController::from_pose(&pose, clip(2.0, 50.0));

    controller.zoom(-1.0e6);

    assert!((controller.distance() - 50.0).abs() < 1e-9);
}

#[test]
fn zoom_changes_the_distance_but_not_the_target() {
    let pose = BowlParams::for_car_size(5.6, 2.2, 2.9).home_view_pose();
    let mut controller = OrbitController::from_pose(&pose, clip(2.0, 50.0));
    let target = controller.target();
    let before = controller.distance();

    controller.zoom(100.0);

    assert!(close(controller.target(), target, 1e-12));
    assert!(controller.distance() < before);
}

#[test]
fn pan_moves_eye_and_target_by_the_same_vector() {
    let pose = BowlParams::for_car_size(5.6, 2.2, 2.9).home_view_pose();
    let mut controller = OrbitController::from_pose(&pose, clip(2.0, 50.0));
    let eye_before = controller.eye();
    let target_before = controller.target();
    let distance = controller.distance();

    controller.pan(120.0, -45.0, 720);

    let eye_delta = sub(controller.eye(), eye_before);
    let target_delta = sub(controller.target(), target_before);
    assert!(close(eye_delta, target_delta, 1e-9));
    assert!(norm(target_delta) > 1e-6, "pan must move the camera");
    assert!((controller.distance() - distance).abs() < 1e-9);
}

#[test]
fn drag_mode_maps_left_right_and_middle() {
    use avm_viewer::controller::{drag_mode, DragMode};

    assert_eq!(Some(DragMode::Rotate), drag_mode(true, false, false));
    assert_eq!(Some(DragMode::Zoom), drag_mode(false, true, false));
    assert_eq!(Some(DragMode::Pan), drag_mode(false, false, true));
    assert_eq!(None, drag_mode(false, false, false));
    assert_eq!(Some(DragMode::Rotate), drag_mode(true, true, true));
}

#[test]
fn left_drag_rotates_and_keeps_the_distance() {
    use avm_viewer::controller::DragMode;

    let pose = BowlParams::for_car_size(5.6, 2.2, 2.9).home_view_pose();
    let mut controller = OrbitController::from_pose(&pose, clip(2.0, 50.0));
    let target = controller.target();
    let distance = controller.distance();
    let eye = controller.eye();

    controller.drag(DragMode::Rotate, 90.0, 0.0, 720);

    assert!(close(controller.target(), target, 1e-12));
    assert!((controller.distance() - distance).abs() < 1e-12);
    assert!(!close(controller.eye(), eye, 1e-6));
}

#[test]
fn right_drag_zooms_without_moving_the_target() {
    use avm_viewer::controller::DragMode;

    let pose = BowlParams::for_car_size(5.6, 2.2, 2.9).home_view_pose();
    let mut controller = OrbitController::from_pose(&pose, clip(2.0, 50.0));
    let target = controller.target();
    let distance = controller.distance();

    controller.drag(DragMode::Zoom, 0.0, -100.0, 720);

    assert!(close(controller.target(), target, 1e-12));
    assert!(controller.distance() < distance);
}

#[test]
fn middle_drag_pans_and_keeps_the_distance() {
    use avm_viewer::controller::DragMode;

    let pose = BowlParams::for_car_size(5.6, 2.2, 2.9).home_view_pose();
    let mut controller = OrbitController::from_pose(&pose, clip(2.0, 50.0));
    let target = controller.target();
    let eye = controller.eye();
    let distance = controller.distance();

    controller.drag(DragMode::Pan, 80.0, -30.0, 720);

    assert!(norm(sub(controller.target(), target)) > 1e-6);
    assert!(close(sub(controller.eye(), eye), sub(controller.target(), target), 1e-9));
    assert!((controller.distance() - distance).abs() < 1e-9);
}
