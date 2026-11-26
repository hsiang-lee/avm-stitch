//! Port of `tests/depth/test_spherical_rectification.cpp` and
//! `tests/depth/test_ground_depth.cpp`.

use avm_core::camera::CameraExtrinsics;
use avm_core::depth::{
    compute_ground_depth_map, make_rectification_basis, phi_deg, rectified_direction,
    sky_up_needs_mirror, triangulate_depth, CameraSide, GroundDepthOptions, SphericalGrid,
};
use avm_core::{Matrix3, Vector3};

fn vec3(x: f64, y: f64, z: f64) -> Vector3<f64> {
    Vector3::new(x, y, z)
}

/// Mirrors `tests/test_support/depth_test_helpers.hpp`.
fn make_look_at_rotation(position: Vector3<f64>, target: Vector3<f64>) -> Matrix3<f64> {
    let forward = (target - position).normalize();
    let world_up = vec3(0.0, 0.0, 1.0);
    let mut right = world_up.cross(&forward);
    if right.norm() < 1e-9 {
        right = vec3(1.0, 0.0, 0.0);
    }
    right = right.normalize();
    let up = forward.cross(&right);
    Matrix3::from_row_slice(&[
        right.x, up.x, forward.x, //
        right.y, up.y, forward.y, //
        right.z, up.z, forward.z,
    ])
}

fn make_stereo_pair(elevation: f64) -> (CameraExtrinsics, CameraExtrinsics) {
    let (s_pitch, c_pitch) = 35.0f64.to_radians().sin_cos();
    let (s_yaw, c_yaw) = 15.0f64.to_radians().sin_cos();

    let ta = vec3(0.0, 0.0, elevation);
    let fa = vec3(c_pitch, 0.0, -s_pitch);
    let a = CameraExtrinsics::new(make_look_at_rotation(ta, ta + fa), ta);

    let tb = vec3(1.1, 0.0, elevation);
    let fb = vec3(c_pitch * c_yaw, c_pitch * s_yaw, -s_pitch);
    let b = CameraExtrinsics::new(make_look_at_rotation(tb, tb + fb), tb);
    (a, b)
}

fn make_look_down_pair() -> (CameraExtrinsics, CameraExtrinsics) {
    let look_down = |position: Vector3<f64>| {
        let forward = vec3(0.0, 0.0, -1.0);
        let right = vec3(1.0, 0.0, 0.0);
        let up = forward.cross(&right);
        CameraExtrinsics::new(
            Matrix3::from_rows(&[
                right.transpose(),
                up.transpose(),
                forward.transpose(),
            ]),
            position,
        )
    };
    (
        look_down(vec3(0.0, 0.0, 1.0)),
        look_down(vec3(1.0, 0.0, 1.0)),
    )
}

fn make_look_up_pair() -> (CameraExtrinsics, CameraExtrinsics) {
    let look_up = |position: Vector3<f64>| {
        let forward = vec3(0.0, 0.0, 1.0);
        let right = vec3(1.0, 0.0, 0.0);
        let up = forward.cross(&right);
        CameraExtrinsics::new(
            Matrix3::from_rows(&[right.transpose(), up.transpose(), forward.transpose()]),
            position,
        )
    };
    (
        look_up(vec3(0.0, 0.0, 1.0)),
        look_up(vec3(1.0, 0.0, 1.0)),
    )
}

fn wrap_phi(phi: f64) -> f64 {
    if phi <= -180.0 {
        phi + 360.0
    } else {
        phi
    }
}

fn angular_distance(p: f64, q: f64) -> f64 {
    let d = (wrap_phi(p) - wrap_phi(q)).abs();
    d.min(360.0 - d)
}

fn make_grid(width: usize, height: usize) -> SphericalGrid {
    SphericalGrid {
        width,
        height,
        psi_min_deg: 0.0,
        psi_max_deg: 180.0,
        mirror_rows: false,
    }
}

/// Deterministic SplitMix64; the C++ fixtures used mt19937, so only the
/// distribution matters here.
struct Rng(u64);

impl Rng {
    fn uniform(&mut self, low: f64, high: f64) -> f64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^= z >> 31;
        let unit = (z >> 11) as f64 / (1u64 << 53) as f64;
        low + (high - low) * unit
    }
}

#[test]
fn det_is_right_handed() {
    let (a, b) = make_stereo_pair(0.0);
    let basis = make_rectification_basis(&a, &b);
    assert!((basis.m1().determinant() - 1.0).abs() < 1e-9);
    assert!((basis.m2().determinant() - 1.0).abs() < 1e-9);
}

#[test]
fn basis_is_orthonormal() {
    let (a, b) = make_stereo_pair(0.0);
    let basis = make_rectification_basis(&a, &b);
    let eye = Matrix3::<f64>::identity();
    let id1 = basis.m1() * basis.m1().transpose();
    let id2 = basis.m2() * basis.m2().transpose();
    for i in 0..3usize {
        for j in 0..3usize {
            assert!((eye[(i, j)] - id1[(i, j)]).abs() < 1e-9);
            assert!((eye[(i, j)] - id2[(i, j)]).abs() < 1e-9);
        }
    }
}

#[test]
fn baseline_points_from_a_to_b() {
    let (a, b) = make_stereo_pair(0.0);
    let basis = make_rectification_basis(&a, &b);
    let v = a
        .rotate_world_to_camera(b.position() - a.position())
        .normalize();
    assert!(basis.e1().dot(&v) > 0.999);
}

#[test]
fn row_alignment_holds_for_random_points() {
    let (a, b) = make_stereo_pair(0.0);
    let basis = make_rectification_basis(&a, &b);
    let mut rng = Rng(12345);

    for _ in 0..50 {
        let mut d_a = vec3(
            rng.uniform(-0.9, 0.9),
            rng.uniform(-0.9, 0.9),
            rng.uniform(0.1, 1.0),
        );
        d_a.normalize_mut();
        let p = a.position() + rng.uniform(1.0, 30.0) * a.rotate_camera_to_world(d_a);

        let d_a_w = (p - a.position()).normalize();
        let d_b_w = (p - b.position()).normalize();
        let phi_a = phi_deg(rectified_direction(&basis, d_a_w, &a, CameraSide::First));
        let phi_b = phi_deg(rectified_direction(&basis, d_b_w, &b, CameraSide::Second));
        assert!(angular_distance(phi_a, phi_b) < 1e-9);
    }
}

#[test]
fn grid_direction_is_unit() {
    let grid = SphericalGrid::default();
    assert!((grid.psi_deg(0) - 0.0).abs() < 1e-12);
    assert!((grid.psi_deg(grid.width - 1) - 180.0).abs() < 1e-12);
    assert!((grid.phi_deg(0) + 180.0).abs() < 1e-12);
    assert!((grid.phi_deg(grid.height - 1) - 180.0).abs() < 1e-12);

    for row in [0, grid.height / 2, grid.height - 1] {
        for col in [0, grid.width / 2, grid.width - 1] {
            assert!((grid.direction_c(row, col).norm() - 1.0).abs() < 1e-12);
        }
    }
}

#[test]
fn mirror_keeps_basis_right_handed() {
    let (a, b) = make_stereo_pair(0.0);
    let basis = make_rectification_basis(&a, &b);
    assert!((basis.m1().determinant() - 1.0).abs() < 1e-9);

    let grid = SphericalGrid::default();
    let mirrored = SphericalGrid {
        mirror_rows: true,
        ..grid
    };
    assert!((mirrored.phi_deg(0) - 180.0).abs() < 1e-12);
    for row in 0..grid.height {
        assert!((-grid.phi_deg(row) - mirrored.phi_deg(row)).abs() < 1e-9);
    }
    for row in [0, grid.height / 2, grid.height - 1] {
        for col in [0, grid.width / 2, grid.width - 1] {
            assert!((mirrored.direction_c(row, col).norm() - 1.0).abs() < 1e-12);
        }
    }
}

#[test]
fn sky_up_needs_mirror_detects_flipped_row_order() {
    let (a, b) = make_stereo_pair(0.0);
    let basis = make_rectification_basis(&a, &b);
    const HEIGHT: usize = 720;
    const STEP: f64 = 360.0 / (HEIGHT - 1) as f64;

    // Pinned: this fixture puts the sky in the half that needs mirroring.
    assert!(sky_up_needs_mirror(&basis, &a, HEIGHT));

    let sky = vec3(0.0, 0.0, 1.0);
    let phi_sky = phi_deg(rectified_direction(&basis, sky, &a, CameraSide::First));
    let row_unmirrored = (phi_sky + 180.0) / STEP;
    assert!((row_unmirrored - 698.0870430056614).abs() < 1e-6);
    assert!(row_unmirrored >= HEIGHT as f64 * 0.5);

    let row_mirrored = (-phi_sky + 180.0) / STEP;
    assert!((row_mirrored - 20.912956994338685).abs() < 1e-6);
    assert!(row_mirrored >= 0.0);
    assert!(row_mirrored < HEIGHT as f64 * 0.5);
}

/// The unmirrored half: an upward-looking pair maps the sky to row 359.5, just
/// below the half-row boundary, so `sky_up_needs_mirror` must say no.
#[test]
fn sky_up_needs_mirror_is_false_for_an_upward_camera() {
    let (a, b) = make_look_up_pair();
    let basis = make_rectification_basis(&a, &b);
    const HEIGHT: usize = 720;
    const STEP: f64 = 360.0 / (HEIGHT - 1) as f64;

    assert!(!sky_up_needs_mirror(&basis, &a, HEIGHT));

    let sky = vec3(0.0, 0.0, 1.0);
    let phi_sky = phi_deg(rectified_direction(&basis, sky, &a, CameraSide::First));
    let row_unmirrored = (phi_sky + 180.0) / STEP;
    assert!((row_unmirrored - 359.5).abs() < 1e-6);
    assert!(row_unmirrored >= 0.0);
    assert!(row_unmirrored < HEIGHT as f64 * 0.5);
}

#[test]
fn ground_depth_known_case() {
    let (a, b) = make_look_down_pair();
    let basis = make_rectification_basis(&a, &b);
    let grid = make_grid(33, 33);

    let map = compute_ground_depth_map(&basis, &a, &b, &grid, GroundDepthOptions::default());
    assert!(map.valid(16, 16));
    assert!((map.depth(16, 16) - 1.0).abs() < 1e-9);
}

#[test]
fn disparity_monotonic_with_depth() {
    let (a, b) = make_stereo_pair(1.5);
    let basis = make_rectification_basis(&a, &b);
    let grid = make_grid(33, 33);

    let map = compute_ground_depth_map(&basis, &a, &b, &grid, GroundDepthOptions::default());
    let col = 16;
    let mut samples: Vec<(f32, f32)> = (0..grid.height)
        .filter(|row| map.valid(*row, col))
        .map(|row| (map.depth(row, col), map.disparity(row, col)))
        .collect();
    assert!(samples.len() >= 5);
    samples.sort_by(|x, y| x.0.partial_cmp(&y.0).expect("finite samples"));
    for pair in samples.windows(2) {
        assert!(pair[1].1 < pair[0].1 + 1e-6);
    }
}

#[test]
fn triangulate_depth_round_trip() {
    let origin_a = vec3(0.0, 0.0, 1.5);
    let origin_b = vec3(1.1, 0.0, 1.5);
    let mut rng = Rng(7);

    for _ in 0..20 {
        let p = vec3(
            rng.uniform(-5.0, 5.0),
            rng.uniform(-5.0, 5.0),
            rng.uniform(0.5, 5.0),
        );
        let dir_a = (p - origin_a).normalize();
        let dir_b = (p - origin_b).normalize();

        let lambda = triangulate_depth(dir_a, origin_a, dir_b, origin_b);
        let reconstructed = origin_a + lambda * dir_a;
        assert!((p.x - reconstructed.x).abs() < 1e-9);
        assert!((p.y - reconstructed.y).abs() < 1e-9);
        assert!((p.z - reconstructed.z).abs() < 1e-9);
    }
}

#[test]
fn invalid_outside_range() {
    let (a, b) = make_look_down_pair();
    let basis = make_rectification_basis(&a, &b);
    let grid = make_grid(33, 33);

    let map = compute_ground_depth_map(&basis, &a, &b, &grid, GroundDepthOptions::default());
    assert!(!map.valid(32, 16));
    assert_eq!(0.0f32, map.depth(32, 16));
}

#[test]
fn map_shapes() {
    let (a, b) = make_look_down_pair();
    let basis = make_rectification_basis(&a, &b);
    let grid = make_grid(64, 32);

    let map = compute_ground_depth_map(&basis, &a, &b, &grid, GroundDepthOptions::default());
    assert_eq!(64, map.width());
    assert_eq!(32, map.height());
    assert!(!map.valid(31, 63));
    assert_eq!(0.0f32, map.depth(31, 63));
    assert_eq!(0.0f32, map.disparity(31, 63));
}

#[test]
fn disparity_analytic_value() {
    let (a, b) = make_look_down_pair();
    let basis = make_rectification_basis(&a, &b);
    let grid = make_grid(33, 33);

    let map = compute_ground_depth_map(&basis, &a, &b, &grid, GroundDepthOptions::default());
    const ROWS: [(usize, f32); 15] = [
        (9, 1.962523),
        (10, 3.722848),
        (11, 5.165377),
        (12, 6.269225),
        (13, 7.065332),
        (14, 7.597193),
        (15, 7.901194),
        (16, 8.000000),
        (17, 7.901194),
        (18, 7.597193),
        (19, 7.065332),
        (20, 6.269225),
        (21, 5.165377),
        (22, 3.722848),
        (23, 1.962523),
    ];
    for (row, expected) in ROWS {
        assert!(map.valid(row, 16), "row {row}");
        let actual = map.disparity(row, 16);
        assert!(
            (expected - actual).abs() < 1e-3,
            "row {row}: expected {expected}, got {actual}"
        );
    }
}

#[test]
fn depth_range_bounds_invalid() {
    let (a, b) = make_look_down_pair();
    let basis = make_rectification_basis(&a, &b);
    let grid = make_grid(33, 33);
    let opts = GroundDepthOptions {
        min_depth: 2.0,
        max_depth: 3.0,
        ..GroundDepthOptions::default()
    };

    let map = compute_ground_depth_map(&basis, &a, &b, &grid, opts);
    let col = 16;

    assert!(map.valid(10, col));
    assert!((map.depth(10, col) - 2.6131).abs() < 1e-3);
    assert!(map.valid(22, col));
    assert!((map.depth(22, col) - 2.6131).abs() < 1e-3);

    assert!(!map.valid(16, col));
    assert_eq!(0.0f32, map.depth(16, col));

    assert!(!map.valid(9, col));
    assert_eq!(0.0f32, map.depth(9, col));
    assert!(!map.valid(23, col));
    assert_eq!(0.0f32, map.depth(23, col));
}
