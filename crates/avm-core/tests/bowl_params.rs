//! Port of `tests/bowl_model/test_bowl_params.cpp`.

mod common;

use avm_core::bowl::{BowlModel, BowlParams, CALIBRATED_ANCHOR_OUTER_RADIUS};
use common::{
    car_bowl_centered, make_car_bowl, make_default_kb_cameras, mesh_params, wrap_kb_cameras,
    default_bake_cameras, default_intrinsics,
};

#[test]
fn accessors_round_trip() {
    let p = BowlParams::new(4.0, 2.0, 0.5, 0.2, 12.6, 0.09, 2.2, 8.6, [1.5, 0.0]);
    assert_eq!(0.5, p.flat_platform_radius());
    assert_eq!(0.2, p.grid_step());
    assert_eq!(12.6, p.outer_radius());
    assert_eq!(0.09, p.slope_coefficient());
    assert_eq!(2.2, p.height_exponent());
    assert_eq!(8.6, p.max_height());
    assert_eq!(1.5, p.car_center()[0]);
    assert_eq!(0.0, p.car_center()[1]);
    assert!((0.4135312 - p.bowl_height(2.0)).abs() < 1e-6);
    assert_eq!(126, p.grid_segments_per_axis());
}

#[test]
fn flat_region_half_extents_are_car_plus_platform() {
    let p = BowlParams::new(4.0, 2.0, 0.5, 0.2, 12.6, 0.09, 2.2, 8.6, [0.0, 0.0]);
    assert_eq!(0.0, p.distance_from_flat_platform(4.0 / 2.0 + 0.5, 0.0));
    assert_eq!(0.0, p.distance_from_flat_platform(0.0, 2.0 / 2.0 + 0.5));
    assert!(p.distance_from_flat_platform(4.0 / 2.0 + 0.5 + 1.0, 0.0) > 0.0);
    assert!(p.distance_from_flat_platform(0.0, 2.0 / 2.0 + 0.5 + 1.0) > 0.0);
}

#[test]
fn distance_from_flat_platform_matches_hand_computation() {
    let p = BowlParams::new(4.0, 2.0, 1.0, 0.2, 12.6, 0.09, 2.2, 8.6, [0.0, 0.0]);
    assert_eq!(0.0, p.distance_from_flat_platform(3.0, 0.0));
    assert_eq!(2.0, p.distance_from_flat_platform(5.0, 0.0));
    assert!(
        ((2.0f64 * 2.0 + 2.0 * 2.0).sqrt() - p.distance_from_flat_platform(5.0, 4.0)).abs()
            < 1e-7
    );

    let shifted = BowlParams::new(4.0, 2.0, 1.0, 0.2, 12.6, 0.09, 2.2, 8.6, [2.0, 0.0]);
    assert_eq!(2.0, shifted.distance_from_flat_platform(7.0, 0.0));
}

#[test]
fn bowl_height_uses_the_power_law() {
    let p = BowlParams::new(4.0, 2.0, 0.5, 0.2, 12.6, 0.09, 2.2, 8.6, [0.0, 0.0]);
    assert_eq!(0.0, p.bowl_height(0.0));
    assert!((0.4135312 - p.bowl_height(2.0)).abs() < 1e-6);
}

#[test]
fn grid_segments_rounds_the_radius_over_the_step() {
    let p = BowlParams::new(4.0, 2.0, 0.5, 0.2, 12.6, 0.09, 2.2, 8.6, [0.0, 0.0]);
    assert_eq!(126, p.grid_segments_per_axis());
}

#[test]
fn full_constructor_override_keeps_the_original_intact() {
    let original = BowlParams::for_car_size(5.6, 2.2, 2.9);
    let modified = BowlParams::new(
        5.6,
        2.2,
        original.flat_platform_radius(),
        0.0,
        original.outer_radius(),
        original.slope_coefficient(),
        original.height_exponent(),
        original.max_height(),
        original.car_center(),
    );

    assert_eq!(0.0, modified.grid_step());
    assert_eq!(0.2, original.grid_step());
    assert_eq!(2.0, original.flat_platform_radius());
    assert_eq!(12.6, original.outer_radius());
    assert_eq!(0.09, original.slope_coefficient());
    assert_eq!(2.2, original.height_exponent());
    assert_eq!(8.6, original.max_height());
    assert_eq!(1.45, original.car_center()[0]);
    assert_eq!(0.0, original.car_center()[1]);
}

#[test]
fn for_car_size_scales_lengths_linearly_and_slope_inversely() {
    let p1 = BowlParams::for_car_size(5.6, 2.2, 2.9);
    let p2 = BowlParams::for_car_size(11.2, 2.2, 2.9);
    assert!((2.0 - p2.outer_radius() / p1.outer_radius()).abs() < 1e-12);
    assert!((2.0 - p2.grid_step() / p1.grid_step()).abs() < 1e-12);
    assert!((2.0 - p2.flat_platform_radius() / p1.flat_platform_radius()).abs() < 1e-12);
    assert!((2.0 - p2.max_height() / p1.max_height()).abs() < 1e-12);
    assert!((2.0f64.powf(-1.2) - p2.slope_coefficient() / p1.slope_coefficient()).abs() < 1e-12);
}

#[test]
fn block_boat_mesh_structure_is_consistent() {
    let p = mesh_params();
    let mesh = p.generate_block_boat_mesh();

    assert!(!mesh.vertices.is_empty());
    assert_eq!(mesh.vertices.len(), mesh.normals.len());
    assert!(!mesh.triangles.is_empty());
    assert_eq!(0, mesh.triangles.len() % 2);
    assert!(mesh.triangles.len() >= 2);

    for v in &mesh.vertices {
        assert!(v[2] >= 0.0);
        assert!(v[2] <= p.max_height() + 1e-9);
    }
    for n in &mesh.normals {
        let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        assert!((1.0 - len).abs() < 1e-9);
    }
    for tri in &mesh.triangles {
        for idx in tri {
            assert!(*idx < mesh.vertices.len());
        }
    }
}

#[test]
fn block_boat_mesh_clips_to_the_outer_radius() {
    let p = mesh_params();
    let mesh = p.generate_block_boat_mesh();
    assert!(!mesh.vertices.is_empty());
    for v in &mesh.vertices {
        assert!(p.distance_from_flat_platform(v[0], v[1]) <= p.outer_radius() + 1e-9);
    }
}

#[test]
fn block_boat_mesh_matches_the_bowl_model_constructor_path() {
    let p = mesh_params();
    let mesh = p.generate_block_boat_mesh();
    let model = BowlModel::new(&p, &make_default_kb_cameras());

    assert_eq!(mesh.vertices.len(), model.vertex_count());
    assert_eq!(mesh.normals.len(), model.normal_count());
    assert_eq!(mesh.triangles.len(), model.triangle_count());

    for i in 0..model.vertex_count() {
        assert_eq!(&mesh.vertices[i], model.vertex(i), "vertex {i}");
        assert_eq!(&mesh.normals[i], model.normal(i), "normal {i}");
    }
    for t in 0..model.triangle_count() {
        assert_eq!(mesh.triangles[t], model.triangle(t), "triangle {t}");
    }
}

#[test]
fn car_anchor_is_exact_regression_to_old_default() {
    let p = BowlParams::for_car_size(5.6, 2.2, 2.9);
    assert_eq!(12.6, p.outer_radius());
    assert_eq!(2.0, p.flat_platform_radius());
    assert_eq!(0.2, p.grid_step());
    assert_eq!(8.6, p.max_height());
    assert_eq!(0.09, p.slope_coefficient());
    assert_eq!(2.2, p.height_exponent());
    let rd_star = (p.max_height() / p.slope_coefficient()).powf(1.0 / p.height_exponent());
    assert!((7.9454 - rd_star).abs() < 1e-4);
    assert!((0.6306 - rd_star / p.outer_radius()).abs() < 1e-4);
    assert!(rd_star < p.outer_radius());
    assert_eq!(1.45, p.car_center()[0]);
    assert_eq!(0.0, p.car_center()[1]);
}

#[test]
fn linear_similarity_doubling_length_doubles_all_lengths() {
    let p1 = BowlParams::for_car_size(5.6, 2.2, 2.9);
    let p2 = BowlParams::for_car_size(11.2, 2.2, 2.9);
    assert!(p1.slope_coefficient() > 0.0);
    assert!(p1.max_height() > 0.0);
    assert!(p1.grid_step() > 0.0);
    assert!(p1.flat_platform_radius() > 0.0);
    assert!(p1.outer_radius() > 0.0);
    assert!((2.0 - p2.outer_radius() / p1.outer_radius()).abs() < 1e-9);
    assert!((2.0 - p2.flat_platform_radius() / p1.flat_platform_radius()).abs() < 1e-9);
    assert!((2.0 - p2.grid_step() / p1.grid_step()).abs() < 1e-9);
    assert!((2.0 - p2.max_height() / p1.max_height()).abs() < 1e-9);
    assert_eq!(p1.height_exponent(), p2.height_exponent());
    assert_eq!(2.2, p1.height_exponent());
    assert!((2.0f64.powf(-1.2) - p2.slope_coefficient() / p1.slope_coefficient()).abs() < 1e-9);
}

#[test]
fn self_similarity_opening_ratio_is_constant_across_scales() {
    let scales = [5.6, 11.2, 44.8, 211.583];
    let mut ref_ratio = 0.0;
    for l in scales {
        let p = BowlParams::for_car_size(l, 2.2, l / 2.0);
        let rd_star = (p.max_height() / p.slope_coefficient()).powf(1.0 / p.height_exponent());
        let opening = rd_star / p.outer_radius();
        if ref_ratio == 0.0 {
            ref_ratio = opening;
            assert!((0.63 - ref_ratio).abs() < 0.63 * 0.01, "anchor opening");
        } else {
            assert!((ref_ratio - opening).abs() < ref_ratio * 1e-9, "L={l}");
        }
        assert!(rd_star < p.outer_radius(), "L={l}");
    }
}

#[test]
fn isotropic_rim_height_stays_within_15_percent() {
    let cars = [(5.6, 2.2, 2.9), (5.27, 2.17, 2.9), (211.583, 53.453, 116.371)];
    for (length, width, axis_x) in cars {
        let p = BowlParams::for_car_size(length, width, axis_x);
        let rim_z = |rd: f64| (p.slope_coefficient() * rd.powf(p.height_exponent())).min(p.max_height());
        let rd_long = p.outer_radius() - (length / 2.0 + p.flat_platform_radius());
        let rd_lat = p.outer_radius() - (width / 2.0 + p.flat_platform_radius());
        assert!(rd_long > 0.0, "L={length}");
        assert!(rd_lat > 0.0, "L={length}");
        let (z_long, z_lat) = (rim_z(rd_long), rim_z(rd_lat));
        let ratio = z_long.max(z_lat) / z_long.min(z_lat);
        assert!(ratio >= 1.0, "L={length}");
        assert!(ratio <= 1.15, "L={length} z_long={z_long} z_lat={z_lat}");
    }
}

#[test]
fn ship_scale_covers_the_camera_ring() {
    let p = BowlParams::for_car_size(211.583, 53.453, 116.371);
    assert!(p.outer_radius() >= 150.0);
    let rd_star = (p.max_height() / p.slope_coefficient()).powf(1.0 / p.height_exponent());
    assert!(rd_star >= 155.0);
    assert!(rd_star <= p.outer_radius());
    assert_eq!(58.1855, p.car_center()[0]);
    assert_eq!(0.0, p.car_center()[1]);
    assert!(p.grid_step() > 0.0);
    assert!(p.slope_coefficient() > 0.0);
    assert!(p.max_height() > 0.0);
    assert_eq!(2.2, p.height_exponent());
}

#[test]
fn ship_scale_all_eight_cameras_see_bowl_with_weight() {
    let p = BowlParams::for_car_size(211.583, 53.453, 116.371);
    assert!(p.outer_radius() >= 150.0);
    let cam_xy: [[f64; 2]; 8] = [
        [96.71248, 0.0],
        [75.55415, 25.65737],
        [17.36875, 25.65737],
        [-40.81666, 25.65737],
        [75.55415, -25.65737],
        [17.36875, -25.65737],
        [-40.81666, -25.65737],
        [-91.59664, 0.0],
    ];
    let cam_z = [
        19.00911, 30.88980, 30.88980, 30.88980, 30.88980, 30.88980, 30.88980, 28.51366,
    ];
    let intrs = vec![common::real_intrinsics(); 8];
    let extrs: Vec<_> = (0..8)
        .map(|j| {
            let r = cam_xy[j][0].hypot(cam_xy[j][1]);
            let az = cam_xy[j][1].atan2(cam_xy[j][0]) * 180.0 / std::f64::consts::PI;
            common::make_look_at_camera(az, r, cam_z[j], 0.0)
        })
        .collect();

    let model = BowlModel::new(&p, &wrap_kb_cameras(&intrs, &extrs));
    assert!(model.vertex_count() > 0);
    for j in 0..8 {
        let weighted = (0..model.vertex_count())
            .filter(|i| model.baked_vertex(*i).weight[j] > 1e-9)
            .count();
        assert!(weighted > 0, "camera {j} has no weighted vertices");
    }
}

#[test]
fn grid_topology_stays_126_by_126_across_scales() {
    for l in [5.6, 18.0, 211.583, 423.166] {
        let p = BowlParams::for_car_size(l, 2.2, l / 2.0);
        let ratio = p.outer_radius() / p.grid_step();
        assert!((63.0 - ratio).abs() < 1.0, "L={l}");
        let rows = (ratio.round() as i64) * 2;
        assert_eq!(126, rows, "L={l}");
        let max_vertices = (rows + 1) * (rows + 1);
        assert!(max_vertices <= 150_000, "L={l}");
        assert!(max_vertices > 0, "L={l}");
    }
}

#[test]
fn default_car_center_keeps_the_legacy_vertex_count() {
    let p = BowlParams::default();
    // 0/0 -> NaN, which the C++ `lround` cast turns into 0 segments.
    assert_eq!(0, p.grid_segments_per_axis());

    let model = BowlModel::new(
        &p,
        &wrap_kb_cameras(&default_intrinsics(), &default_bake_cameras()),
    );
    assert_eq!(1, model.vertex_count());
    assert_eq!(0, model.triangle_count());
    let count_default = model.vertex_count();

    let p_centered = BowlParams::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, [0.0, 0.0]);
    let model2 = BowlModel::new(
        &p_centered,
        &wrap_kb_cameras(&default_intrinsics(), &default_bake_cameras()),
    );
    assert_eq!(count_default, model2.vertex_count());
}

#[test]
fn car_center_shifts_the_flat_region() {
    let params = BowlParams::new(2.0, 1.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, [2.0, 0.0]);
    assert_eq!(0.0, params.distance_from_flat_platform(3.0, 0.0));

    let default_params = BowlParams::new(2.0, 1.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, [0.0, 0.0]);
    assert!(default_params.distance_from_flat_platform(3.0, 0.0) > 0.0);
}

#[test]
fn view_presets_follow_the_anchor_scale() {
    let p = make_car_bowl();
    assert_eq!(CALIBRATED_ANCHOR_OUTER_RADIUS, 12.6);
    let poses = p.default_view_poses();
    assert_eq!(4, poses.len());
    assert_eq!("3d_front", poses[0].name);
    assert!((p.outer_radius() / CALIBRATED_ANCHOR_OUTER_RADIUS * 5.8 + p.car_center()[0]
        - poses[0].position[0])
        .abs()
        < 1e-12);
    assert!(
        (12.0 * p.outer_radius() / CALIBRATED_ANCHOR_OUTER_RADIUS - p.home_view_distance()).abs()
            < 1e-12
    );
    assert_eq!(
        p.home_view_pose().position[2],
        6.0 * p.outer_radius() / CALIBRATED_ANCHOR_OUTER_RADIUS - p.car_center()[1]
    );
}

#[test]
fn clip_planes_keep_near_below_far() {
    let p = car_bowl_centered();
    let planes = p.clip_planes_for(7.0);
    assert!(planes.near > 0.0);
    assert!(planes.near < planes.far);
    let expected_far = 5.3553572528096565 * (7.0 + p.outer_radius());
    assert!((expected_far - planes.far).abs() < 1e-9);
}
