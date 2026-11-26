use avm_core::bowl::{BowlParams, CALIBRATED_ANCHOR_OUTER_RADIUS};

fn bowl_at_scale(scale: f64, car_center: [f64; 2]) -> BowlParams {
    BowlParams::new(
        2.0,
        1.0,
        0.5,
        0.1,
        CALIBRATED_ANCHOR_OUTER_RADIUS * scale,
        3.2,
        1.0,
        15.0,
        car_center,
    )
}

fn max_eye_to_target(scale: f64) -> f64 {
    bowl_at_scale(scale, [0.0, 0.0])
        .default_view_poses()
        .iter()
        .map(|pose| {
            let d = [
                pose.position[0] - pose.target[0],
                pose.position[1] - pose.target[1],
                pose.position[2] - pose.target[2],
            ];
            (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
        })
        .fold(0.0, f64::max)
}

#[test]
fn default_view_poses_returns_4_views() {
    let poses = bowl_at_scale(1.0, [0.0, 0.0]).default_view_poses();
    assert_eq!(4, poses.len());
}

#[test]
fn default_view_poses_have_correct_names() {
    let poses = bowl_at_scale(1.0, [0.0, 0.0]).default_view_poses();
    assert_eq!(4, poses.len());
    assert_eq!("3d_front", poses[0].name);
    assert_eq!("3d_rear", poses[1].name);
    assert_eq!("3d_left", poses[2].name);
    assert_eq!("3d_right", poses[3].name);
    for pose in &poses {
        assert_ne!("topdown", pose.name);
    }
}

#[test]
fn default_view_poses_all_use_fovy_60() {
    let poses = bowl_at_scale(1.0, [0.0, 0.0]).default_view_poses();
    assert_eq!(4, poses.len());
    for pose in &poses {
        assert_eq!(60.0, pose.fovy_deg);
    }
}

#[test]
fn default_view_poses_share_target_and_up() {
    let poses = bowl_at_scale(1.0, [0.0, 0.0]).default_view_poses();
    assert_eq!(4, poses.len());
    for pose in &poses {
        assert!((0.0 - pose.target[0]).abs() < 1e-9);
        assert!((0.0 - pose.target[1]).abs() < 1e-9);
        assert!((0.2 - pose.target[2]).abs() < 1e-9);
        assert!((0.0 - pose.up[0]).abs() < 1e-9);
        assert!((0.0 - pose.up[1]).abs() < 1e-9);
        assert!((1.0 - pose.up[2]).abs() < 1e-9);
    }
}

#[test]
fn default_view_poses_are_offset_by_the_car_center() {
    let car_center = [2.0, 1.0];
    let poses = bowl_at_scale(1.0, car_center).default_view_poses();
    let reference = bowl_at_scale(1.0, [0.0, 0.0]).default_view_poses();
    assert_eq!(reference.len(), poses.len());

    for (i, (pose, base)) in poses.iter().zip(&reference).enumerate() {
        assert!(
            (base.position[0] + car_center[0] - pose.position[0]).abs() < 1e-9,
            "pose {i}"
        );
        assert!(
            (base.position[1] + car_center[1] - pose.position[1]).abs() < 1e-9,
            "pose {i}"
        );
        assert!((base.position[2] - pose.position[2]).abs() < 1e-9, "pose {i}");
        assert!(
            (base.target[0] + car_center[0] - pose.target[0]).abs() < 1e-9,
            "pose {i}"
        );
        assert!(
            (base.target[1] + car_center[1] - pose.target[1]).abs() < 1e-9,
            "pose {i}"
        );
        assert!((base.target[2] - pose.target[2]).abs() < 1e-9, "pose {i}");
        for k in 0..3 {
            assert!((base.up[k] - pose.up[k]).abs() < 1e-9, "pose {i}");
        }
        assert_eq!(base.fovy_deg, pose.fovy_deg, "pose {i}");
        assert_eq!(base.name, pose.name, "pose {i}");
    }
}

#[test]
fn default_view_poses_at_scale_one_match_historical_constants() {
    let poses = bowl_at_scale(1.0, [0.0, 0.0]).default_view_poses();
    assert_eq!(4, poses.len());
    assert!((5.8f64.abs() - poses[0].position[0].abs()).abs() < 1e-12);
    assert!((0.0 - poses[0].position[1]).abs() < 1e-12);
    assert!((2.0 - poses[0].position[2]).abs() < 1e-12);
    assert!((0.0 - poses[0].target[0]).abs() < 1e-12);
    assert!((0.0 - poses[0].target[1]).abs() < 1e-12);
    assert!((0.2 - poses[0].target[2]).abs() < 1e-12);
    for (i, pose) in poses.iter().enumerate() {
        let offset = if i == 2 || i == 3 {
            pose.position[1]
        } else {
            pose.position[0]
        };
        assert!((5.8f64.abs() - offset.abs()).abs() < 1e-12, "pose {i}");
        assert!((2.0 - pose.position[2]).abs() < 1e-12, "pose {i}");
        assert!((0.0 - pose.target[0]).abs() < 1e-12, "pose {i}");
        assert!((0.0 - pose.target[1]).abs() < 1e-12, "pose {i}");
        assert!((0.2 - pose.target[2]).abs() < 1e-12, "pose {i}");
    }
}

#[test]
fn default_view_poses_scale_offsets_and_heights() {
    let s = 2.5;
    let reference = bowl_at_scale(1.0, [0.0, 0.0]).default_view_poses();
    let scaled = bowl_at_scale(s, [0.0, 0.0]).default_view_poses();
    assert_eq!(reference.len(), scaled.len());

    for (i, (base, pose)) in reference.iter().zip(&scaled).enumerate() {
        for k in 0..3 {
            assert!((base.position[k] * s - pose.position[k]).abs() < 1e-9, "pose {i}");
            assert!((base.target[k] * s - pose.target[k]).abs() < 1e-9, "pose {i}");
        }
        assert_eq!(base.fovy_deg, pose.fovy_deg, "pose {i}");
        assert_eq!(base.name, pose.name, "pose {i}");
        for k in 0..3 {
            assert_eq!(base.up[k], pose.up[k], "pose {i}");
        }
    }
}

#[test]
fn home_view_pose_agrees_with_the_hand_built_scale() {
    for params in [bowl_at_scale(1.0, [0.0; 2]), bowl_at_scale(2.5, [0.0; 2])] {
        let s = params.outer_radius() / CALIBRATED_ANCHOR_OUTER_RADIUS;
        let via_bowl = params.home_view_pose();
        let hand_built = (
            "",
            [10.0 * s, -6.0 * s, 6.0 * s],
            [0.0, 0.0, 0.5 * s],
            [0.0, 0.0, 1.0],
            60.0,
        );
        assert_eq!(hand_built.0, via_bowl.name);
        for k in 0..3 {
            assert_eq!(hand_built.1[k], via_bowl.position[k], "position {k}");
            assert_eq!(hand_built.2[k], via_bowl.target[k], "target {k}");
            assert_eq!(hand_built.3[k], via_bowl.up[k], "up {k}");
        }
        assert_eq!(hand_built.4, via_bowl.fovy_deg);
        assert_eq!(12.0 * s, params.home_view_distance());
    }
}

#[test]
fn home_view_pose_car_center_offsets_position_and_target() {
    let home = bowl_at_scale(1.0, [2.0, 1.0]).home_view_pose();
    assert!((12.0 - home.position[0]).abs() < 1e-12);
    assert!((-6.0 - home.position[1]).abs() < 1e-12);
    assert!((5.0 - home.position[2]).abs() < 1e-12);
    assert!((2.0 - home.target[0]).abs() < 1e-12);
    assert!((0.0 - home.target[1]).abs() < 1e-12);
    assert!((-0.5 - home.target[2]).abs() < 1e-12);
    assert!((0.0 - home.up[0]).abs() < 1e-12);
    assert!((0.0 - home.up[1]).abs() < 1e-12);
    assert!((1.0 - home.up[2]).abs() < 1e-12);
    assert_eq!(60.0, home.fovy_deg);
}

#[test]
fn default_view_poses_agree_with_the_hand_built_scale_for_car_sizes() {
    for params in [
        BowlParams::for_car_size(5.6, 2.2, 2.9),
        BowlParams::for_car_size(12.6, 5.0, 6.3),
    ] {
        let s = params.outer_radius() / CALIBRATED_ANCHOR_OUTER_RADIUS;
        let via_bowl = params.default_view_poses();
        let [cx, cy] = params.car_center();
        let z_ground = 0.2 * s;
        let hand_built = [
            ("3d_front", [cx + 5.8 * s, cy, 2.0 * s]),
            ("3d_rear", [cx - 5.8 * s, cy, 2.0 * s]),
            ("3d_left", [cx, cy + 5.8 * s, 2.0 * s]),
            ("3d_right", [cx, cy - 5.8 * s, 2.0 * s]),
        ];
        assert_eq!(hand_built.len(), via_bowl.len());
        for (i, pose) in via_bowl.iter().enumerate() {
            let (name, position) = hand_built[i];
            assert_eq!(name, pose.name, "pose {i}");
            for k in 0..3 {
                assert_eq!(position[k], pose.position[k], "pose {i} position {k}");
                assert_eq!([cx, cy, z_ground][k], pose.target[k], "pose {i} target {k}");
            }
            assert_eq!([0.0, 0.0, 1.0], pose.up, "pose {i} up");
            assert_eq!(60.0, pose.fovy_deg, "pose {i}");
        }
    }
}

#[test]
fn clip_planes_car_scale_cover_the_bowl() {
    let scene_radius = CALIBRATED_ANCHOR_OUTER_RADIUS;
    let max_eye = max_eye_to_target(1.0);
    let cp = bowl_at_scale(1.0, [0.0, 0.0]).clip_planes_for(max_eye);

    assert!((0.1 - cp.near).abs() < 1e-3, "near = {}", cp.near);
    assert!((100.0 - cp.far).abs() < 1e-2, "far = {}", cp.far);
    assert!(cp.far > max_eye + 2.0 * scene_radius);
    assert!(cp.near < 1.0);
    assert!(cp.near < cp.far);
    assert!(cp.near / cp.far >= 5.0e-4);
    assert!(cp.near / cp.far <= 1.0e-2);
}

#[test]
fn clip_planes_ship_scale_cover_the_bowl() {
    let scene_radius = 476.0;
    let max_eye = max_eye_to_target(scene_radius / CALIBRATED_ANCHOR_OUTER_RADIUS);
    let cp = bowl_at_scale(scene_radius / CALIBRATED_ANCHOR_OUTER_RADIUS, [0.0, 0.0])
        .clip_planes_for(max_eye);

    assert!(max_eye > 200.0);
    assert!(cp.far > 1000.0);
    assert!(cp.far > max_eye + scene_radius);
    assert!(cp.near < 10.0);
    assert!(cp.near > 0.1);
    assert!(cp.near < cp.far);
    assert!(cp.near / cp.far >= 5.0e-4);
    assert!(cp.near / cp.far <= 1.0e-2);
}

#[test]
fn clip_planes_are_scale_invariant() {
    let k = 476.0 / CALIBRATED_ANCHOR_OUTER_RADIUS;
    let car = bowl_at_scale(1.0, [0.0, 0.0]).clip_planes_for(max_eye_to_target(1.0));
    let ship = bowl_at_scale(k, [0.0, 0.0]).clip_planes_for(max_eye_to_target(k));

    assert!((car.near * k - ship.near).abs() < (car.near * k).abs() * 1e-6);
    assert!((car.far * k - ship.far).abs() < (car.far * k).abs() * 1e-6);
}

/// Degenerate parameters (the all-zero default) must not panic: C++ `clamp`
/// with `min > max` is undefined behaviour, so the documented Rust rule is
/// "no panic, and `near <= far`" — for a zero-radius bowl both are 0.
#[test]
fn clip_planes_on_degenerate_params_do_not_panic() {
    let default = BowlParams::default();
    let cp = default.clip_planes_for(0.0);
    assert_eq!(0.0, cp.near);
    assert_eq!(0.0, cp.far);

    let with_distance = default.clip_planes_for(7.0);
    assert!(with_distance.near >= 0.0);
    assert!(with_distance.near <= with_distance.far);
}

/// `min(max(v, lo), hi)` keeps a NaN radius NaN, like the C++ release build
/// (verified with a throw-away probe: `std::min(std::max(nan, 1e-3), nan)` is
/// NaN). Rust's `f64::max`/`min` would substitute the non-NaN operand instead.
#[test]
fn clip_planes_propagate_a_nan_radius() {
    let nan = BowlParams::new(
        f64::NAN,
        1.0,
        0.5,
        0.0,
        f64::NAN,
        0.0,
        0.0,
        0.0,
        [0.0, 0.0],
    );
    let planes = nan.clip_planes_for(0.0);
    assert!(
        planes.near.is_nan() && planes.far.is_nan(),
        "expected NaN planes, got {planes:?}"
    );
}
