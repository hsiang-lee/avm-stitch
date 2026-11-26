//! Port of `tests/bowl_model/test_bowl_model.cpp`.

mod common;

use avm_core::bowl::{BowlModel, BowlParams};
use avm_core::camera::{
    make_camera_model, Camera, CameraExtrinsics, CameraModel, CameraModelParams, KbParams,
    LutParams,
};
use common::{
    arr, car_bowl_centered, default_bake_cameras, default_intrinsics, is_vertex_baked_visible,
    is_vertex_visible_to_camera, make_coarse_bowl, make_coarse_bowl_params,
    make_default_kb_cameras, make_kb_model, make_look_at_camera, real_intrinsics,
    side_cameras, tall_bowl_params, vec3, wrap_kb_cameras,
};

fn count_flat_vertices(model: &BowlModel, flat_x: f64, flat_y: f64) -> usize {
    (0..model.vertex_count())
        .filter(|i| {
            let v = model.vertex(*i);
            v[0].abs() < flat_x - 1e-9 && v[1].abs() < flat_y - 1e-9
        })
        .count()
}

fn standard_intr() -> KbParams {
    KbParams {
        width: 640,
        height: 480,
        fx: 300.0,
        fy: 300.0,
        cx: 320.0,
        cy: 240.0,
        ..KbParams::default()
    }
}

#[test]
fn car_center_flat_region_is_centred_at_the_car_center() {
    let derived = BowlParams::for_car_size(5.6, 2.2, 2.9);
    let params = BowlParams::new(
        6.0,
        4.0,
        0.5,
        0.2,
        derived.outer_radius(),
        derived.slope_coefficient(),
        derived.height_exponent(),
        derived.max_height(),
        [1.55, 0.0],
    );
    let flat_x = 6.0 / 2.0 + 0.5;
    let flat_y = 4.0 / 2.0 + 0.5;

    let model = BowlModel::new(&params, &make_default_kb_cameras());

    let mut left_count = 0;
    let mut right_count = 0;
    for i in 0..model.vertex_count() {
        let v = model.vertex(i);
        if v[2] > 1e-9 {
            continue;
        }
        let rel_x = v[0] - params.car_center()[0];
        let rel_y = v[1] - params.car_center()[1];
        if rel_x.abs() < flat_x - 0.1 && rel_y.abs() < flat_y - 0.1 {
            if rel_x < -0.01 {
                left_count += 1;
            } else if rel_x > 0.01 {
                right_count += 1;
            }
        }
    }

    let total = left_count + right_count;
    assert!(total > 0, "no flat-region vertices straddling car_center");
    assert!(
        (left_count as f64 - right_count as f64).abs() <= total as f64 * 0.20,
        "left={left_count} right={right_count}"
    );
}

#[test]
fn car_center_grid_extent_is_symmetric_about_the_car_center() {
    let derived = BowlParams::for_car_size(5.6, 2.2, 2.9);
    let params = BowlParams::new(
        5.6,
        2.2,
        derived.flat_platform_radius(),
        derived.grid_step(),
        derived.outer_radius(),
        derived.slope_coefficient(),
        derived.height_exponent(),
        derived.max_height(),
        [1.45, 0.0],
    );
    let model = BowlModel::new(&params, &make_default_kb_cameras());
    assert!(model.vertex_count() > 0);

    let mut min_x = f64::MAX;
    let mut max_x = -f64::MAX;
    for i in 0..model.vertex_count() {
        let v = model.vertex(i);
        min_x = min_x.min(v[0]);
        max_x = max_x.max(v[0]);
    }

    let cx = 1.45;
    let outer_radius = 12.6;
    let step = 0.2;
    assert!(
        (outer_radius - (max_x - cx)).abs() <= step,
        "front extent (max_x - cx) should be outer_radius, got {}",
        max_x - cx
    );
    assert!(
        (outer_radius - (cx - min_x)).abs() <= step,
        "back extent (cx - min_x) should be outer_radius, got {}",
        cx - min_x
    );

    let mut max_front_axial: f64 = 0.0;
    let mut max_back_axial: f64 = 0.0;
    for i in 0..model.vertex_count() {
        let v = model.vertex(i);
        let rel_y = v[1] - params.car_center()[1];
        if rel_y.abs() > step {
            continue;
        }
        let dx = v[0] - cx;
        let dist = dx.abs();
        if dx > 0.0 {
            max_front_axial = max_front_axial.max(dist);
        } else if dx < 0.0 {
            max_back_axial = max_back_axial.max(dist);
        }
    }
    assert!(max_front_axial > 0.0, "must have front-axial vertices");
    assert!(max_back_axial > 0.0, "must have back-axial vertices");
    assert!(
        (max_front_axial - max_back_axial).abs() <= step,
        "front axial radius ({max_front_axial}) should be about back axial radius ({max_back_axial})"
    );
    assert!((outer_radius - max_front_axial).abs() <= step);
    assert!((outer_radius - max_back_axial).abs() <= step);
}

#[test]
fn car_center_default_is_unchanged() {
    let p = car_bowl_centered();
    let model = BowlModel::new(&p, &make_default_kb_cameras());
    assert!(model.vertex_count() > 0);
    let outer_radius = p.outer_radius();
    for i in 0..model.vertex_count() {
        let v = model.vertex(i);
        assert!(v[0] >= -outer_radius - 1e-9);
        assert!(v[0] <= outer_radius + 1e-9);
        assert!(v[1] >= -outer_radius - 1e-9);
        assert!(v[1] <= outer_radius + 1e-9);
    }
}

#[test]
fn returns_a_non_empty_mesh() {
    let model = BowlModel::new(&car_bowl_centered(), &make_default_kb_cameras());
    assert!(model.vertex_count() > 0);
    assert!(model.triangle_count() > 0);
    assert_eq!(model.vertex_count(), model.normal_count());
}

#[test]
fn grid_size_matches_params() {
    let p = car_bowl_centered();
    let model = BowlModel::new(&p, &make_default_kb_cameras());
    let mut xs: Vec<f64> = (0..model.vertex_count()).map(|i| model.vertex(i)[0]).collect();
    xs.sort_by(|a, b| a.partial_cmp(b).expect("finite x"));
    xs.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
    assert!(xs.len() >= 2);
    assert_eq!(127, xs.len());
    assert!((p.grid_step() - (xs[1] - xs[0])).abs() < 1e-9);
    assert!((2.0 * p.outer_radius() - (xs[xs.len() - 1] - xs[0])).abs() < 1e-9);
}

#[test]
fn vertices_stay_inside_the_xy_bounds() {
    let model = BowlModel::new(&car_bowl_centered(), &make_default_kb_cameras());
    assert!(model.vertex_count() > 0);
    for i in 0..model.vertex_count() {
        let v = model.vertex(i);
        assert!(v[0] >= -12.6 - 1e-9);
        assert!(v[0] <= 12.6 + 1e-9);
        assert!(v[1] >= -12.6 - 1e-9);
        assert!(v[1] <= 12.6 + 1e-9);
    }
}

#[test]
fn vertices_z_stay_within_max_height() {
    let p = car_bowl_centered();
    let model = BowlModel::new(&p, &make_default_kb_cameras());
    assert!(model.vertex_count() > 0);
    for i in 0..model.vertex_count() {
        let v = model.vertex(i);
        assert!(v[2] >= 0.0);
        assert!(v[2] <= p.max_height() + 1e-9);
    }
}

#[test]
fn flat_center_region_has_zero_height() {
    let derived = BowlParams::for_car_size(5.6, 2.2, 2.9);
    let params = BowlParams::new(
        2.0,
        1.0,
        0.5,
        0.2,
        12.6,
        derived.slope_coefficient(),
        derived.height_exponent(),
        derived.max_height(),
        [0.0, 0.0],
    );
    let flat_x = 2.0 / 2.0 + 0.5;
    let flat_y = 1.0 / 2.0 + 0.5;
    let model = BowlModel::new(&params, &make_default_kb_cameras());
    let mut found_flat = false;
    for i in 0..model.vertex_count() {
        let v = model.vertex(i);
        if v[0].abs() < flat_x - 1e-9 && v[1].abs() < flat_y - 1e-9 {
            found_flat = true;
            assert_eq!(0.0, v[2]);
        }
    }
    assert!(found_flat);
}

#[test]
fn height_is_monotonic_in_the_distance_from_the_platform() {
    let params = car_bowl_centered();
    let model = BowlModel::new(&params, &make_default_kb_cameras());
    assert!(model.vertex_count() > 0);

    let mut rd_z: Vec<(f64, f64)> = (0..model.vertex_count())
        .map(|i| {
            let v = model.vertex(i);
            (params.distance_from_flat_platform(v[0], v[1]), v[2])
        })
        .collect();
    rd_z.sort_by(|a, b| a.partial_cmp(b).expect("finite rd"));

    for i in 1..rd_z.len() {
        if rd_z[i].0 > rd_z[i - 1].0 + 1e-12 {
            assert!(
                rd_z[i].1 >= rd_z[i - 1].1 - 1e-9,
                "z not monotonic at rd {}",
                rd_z[i].0
            );
        }
    }
}

#[test]
fn normals_are_unit_length() {
    let model = BowlModel::new(&car_bowl_centered(), &make_default_kb_cameras());
    assert_eq!(model.vertex_count(), model.normal_count());
    for i in 0..model.normal_count() {
        let n = model.normal(i);
        let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        assert!((1.0 - len).abs() < 1e-6, "normal {i}");
    }
}

#[test]
fn flat_center_normals_point_upward() {
    let derived = BowlParams::for_car_size(5.6, 2.2, 2.9);
    let params = BowlParams::new(
        2.0,
        1.0,
        0.5,
        0.2,
        12.6,
        derived.slope_coefficient(),
        derived.height_exponent(),
        derived.max_height(),
        [0.0, 0.0],
    );
    let flat_x = 2.0 / 2.0 + 0.5;
    let flat_y = 1.0 / 2.0 + 0.5;
    let model = BowlModel::new(&params, &make_default_kb_cameras());
    let mut found_flat = false;
    for k in 0..model.vertex_count() {
        let v = model.vertex(k);
        if v[0].abs() < flat_x - 1e-9 && v[1].abs() < flat_y - 1e-9 {
            found_flat = true;
            let n = model.normal(k);
            assert!((0.0 - n[0]).abs() < 1e-6);
            assert!((0.0 - n[1]).abs() < 1e-6);
            assert!((1.0 - n[2]).abs() < 1e-6);
        }
    }
    assert!(found_flat);
}

#[test]
fn triangles_have_valid_indices() {
    let model = BowlModel::new(&car_bowl_centered(), &make_default_kb_cameras());
    assert!(model.triangle_count() > 0);
    for t in 0..model.triangle_count() {
        for idx in model.triangle(t) {
            assert!(idx < model.vertex_count());
        }
    }
}

#[test]
fn different_car_size_changes_the_flat_region() {
    let derived = BowlParams::for_car_size(5.6, 2.2, 2.9);
    let truck_params = BowlParams::new(
        18.0,
        2.5,
        derived.flat_platform_radius(),
        0.5,
        derived.outer_radius(),
        derived.slope_coefficient(),
        derived.height_exponent(),
        derived.max_height(),
        [0.0, 0.0],
    );
    let truck_model = BowlModel::new(&truck_params, &make_default_kb_cameras());
    assert!(truck_model.vertex_count() > 0);

    let default_params = BowlParams::new(
        5.6,
        2.2,
        derived.flat_platform_radius(),
        0.5,
        derived.outer_radius(),
        derived.slope_coefficient(),
        derived.height_exponent(),
        derived.max_height(),
        [0.0, 0.0],
    );
    let default_model = BowlModel::new(&default_params, &make_default_kb_cameras());

    let truck_flat = count_flat_vertices(&truck_model, 18.0 / 2.0 + 2.0, 2.5 / 2.0 + 2.0);
    let default_flat = count_flat_vertices(&default_model, 5.6 / 2.0 + 2.0, 2.2 / 2.0 + 2.0);
    assert!(truck_flat > default_flat);
}

#[test]
fn minimal_params_still_build_a_one_cell_mesh() {
    let params = BowlParams::new(0.0, 0.0, 0.0, 12.6, 12.6, 0.0, 0.0, 0.0, [0.0, 0.0]);
    let model = BowlModel::new(&params, &make_default_kb_cameras());
    assert!(model.vertex_count() > 0);
    for t in 0..model.triangle_count() {
        for idx in model.triangle(t) {
            assert!(idx < model.vertex_count());
        }
    }
}

#[test]
fn bake_produces_one_baked_vertex_per_mesh_vertex() {
    let intrs = vec![real_intrinsics()];
    let extr = CameraExtrinsics::from_rows(
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        [0.0, 0.0, 5.0],
    );
    let model = make_coarse_bowl(5.0, &intrs, &[extr]);
    assert!(model.vertex_count() > 0);
    assert_eq!(1, model.baked_num_cameras());

    let mut seen_visible = 0;
    for i in 0..model.vertex_count() {
        let baked = model.baked_vertex(i);
        if baked.weight[0] > 1e-9f32 {
            seen_visible += 1;
        }
        assert!(baked.uv[0][0] >= 0.0f32);
        assert!(baked.uv[0][0] <= 1.0f32);
        assert!(baked.uv[0][1] >= 0.0f32);
        assert!(baked.uv[0][1] <= 1.0f32);
    }
    assert!(seen_visible > 0);
}

#[test]
fn single_camera_uv_matches_the_projection() {
    let intr = real_intrinsics();
    let extr = CameraExtrinsics::identity();
    let model = make_coarse_bowl(5.0, std::slice::from_ref(&intr), &[extr]);
    assert!(model.vertex_count() > 0);
    assert_eq!(1, model.baked_num_cameras());

    let rep = make_kb_model(
        intr.width, intr.height, intr.fx, intr.fy, intr.cx, intr.cy, intr.k1, intr.k2, intr.k3,
        intr.k4, 0.0,
    );
    let mut checked = 0;
    for i in 0..model.vertex_count() {
        let p = model.vertex(i);
        if !is_vertex_visible_to_camera(p, &intr, &extr) {
            continue;
        }
        checked += 1;
        let uv = rep.project_camera_point(arr(extr.to_camera(vec3(p[0], p[1], p[2]))));
        assert!((f64::from(model.baked_vertex(i).uv[0][0]) - uv[0] / f64::from(rep.width())).abs() < 1e-6);
        assert!((f64::from(model.baked_vertex(i).uv[0][1]) - uv[1] / f64::from(rep.height())).abs() < 1e-6);
        assert!((f64::from(model.baked_vertex(i).weight[0]) - 1.0).abs() < 1e-6);
    }
    assert!(checked > 0);
}

#[test]
fn baked_slots_match_camera_count_for_4_6_and_8() {
    for n in [4usize, 6, 8] {
        let azimuths: Vec<f64> = (0..n).map(|i| i as f64 * 360.0 / n as f64).collect();
        let intrs = vec![real_intrinsics(); n];
        let extrs = side_cameras(&azimuths);
        let model = make_coarse_bowl(12.6, &intrs, &extrs);

        assert_eq!(n, model.baked_num_cameras(), "N={n}");
        assert!(model.vertex_count() > 0, "N={n}");

        let mut last_slot_used = false;
        let mut visible_vertices = 0;
        for i in 0..model.vertex_count() {
            let baked = model.baked_vertex(i);
            assert_eq!(n, baked.uv.len(), "N={n}");
            assert_eq!(n, baked.weight.len(), "N={n}");
            let mut sum = 0.0;
            for j in 0..n {
                sum += f64::from(baked.weight[j]);
                assert!(baked.weight[j] >= 0.0f32, "N={n}");
                assert!((0.0..=1.0).contains(&baked.uv[j][0]), "N={n}");
                assert!((0.0..=1.0).contains(&baked.uv[j][1]), "N={n}");
            }
            if sum > 1e-9 {
                visible_vertices += 1;
                assert!((1.0 - sum).abs() < 1e-6, "N={n}");
            }
            if baked.weight[n - 1] > 0.0f32 {
                last_slot_used = true;
            }
        }
        assert!(visible_vertices > 0, "N={n}");
        assert!(last_slot_used, "N={n}");
    }
}

#[test]
fn baked_vertex_is_sized_by_the_camera_count() {
    let extr = CameraExtrinsics::from_rows(
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        [0.0, 0.0, 5.0],
    );
    let model = make_coarse_bowl(5.0, &[real_intrinsics()], &[extr]);
    assert_eq!(1, model.baked_num_cameras());
    assert!(model.vertex_count() > 0);
    let baked = model.baked_vertex(0);
    assert_eq!(1, baked.uv.len());
    assert_eq!(1, baked.weight.len());
}

#[test]
fn four_camera_weights_sum_to_one_when_visible() {
    let model = make_coarse_bowl(12.6, &default_intrinsics(), &default_bake_cameras());
    let mut visible_vertices = 0;
    for i in 0..model.vertex_count() {
        let baked = model.baked_vertex(i);
        let sum: f64 = baked.weight[..4].iter().copied().map(f64::from).sum();
        if sum > 1e-9 {
            visible_vertices += 1;
            assert!((1.0 - sum).abs() < 1e-6, "vertex {i}");
        }
    }
    assert!(visible_vertices > 0);
}

#[test]
fn four_camera_uvs_stay_in_the_normalized_range() {
    let model = make_coarse_bowl(12.6, &default_intrinsics(), &default_bake_cameras());
    for i in 0..model.vertex_count() {
        let baked = model.baked_vertex(i);
        for j in 0..4 {
            assert!((0.0..=1.0).contains(&baked.uv[j][0]), "vertex {i} camera {j}");
            assert!((0.0..=1.0).contains(&baked.uv[j][1]), "vertex {i} camera {j}");
        }
    }
}

#[test]
fn four_camera_invisible_pair_has_zero_weight() {
    let intrs = default_intrinsics();
    let extrs = default_bake_cameras();
    let model = make_coarse_bowl(12.6, &intrs, &extrs);

    let mut found_invisible_pair = false;
    let mut found_flat_vertex = false;
    for i in 0..model.vertex_count() {
        let p = model.vertex(i);
        if p[2] == 0.0 {
            found_flat_vertex = true;
        }
        for (j, (intr, extr)) in intrs.iter().zip(&extrs).enumerate() {
            if !is_vertex_visible_to_camera(p, intr, extr) {
                found_invisible_pair = true;
                assert_eq!(0.0f32, model.baked_vertex(i).weight[j], "vertex {i} camera {j}");
            }
        }
    }
    assert!(found_invisible_pair);
    assert!(found_flat_vertex);
}

#[test]
fn four_camera_bake_has_both_visible_and_invisible_vertices() {
    let model = make_coarse_bowl(12.6, &default_intrinsics(), &default_bake_cameras());
    let mut visible_vertices = 0;
    let mut invisible_vertices = 0;
    for i in 0..model.vertex_count() {
        let baked = model.baked_vertex(i);
        let sum: f64 = baked.weight[..4].iter().copied().map(f64::from).sum();
        if sum > 1e-9 {
            visible_vertices += 1;
        } else {
            invisible_vertices += 1;
        }
    }
    assert!(visible_vertices > 0);
    assert!(invisible_vertices > 0);
}

#[test]
fn point_in_rect_but_outside_the_fisheye_circle_is_invisible() {
    let intr = standard_intr();
    let extr = CameraExtrinsics::from_rows(
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        [0.0, 0.0, 5.0],
    );

    let model = BowlModel::new(&tall_bowl_params(), &wrap_kb_cameras(std::slice::from_ref(&intr), &[extr]));
    assert!(model.vertex_count() > 0);

    let rep = make_kb_model(
        intr.width, intr.height, intr.fx, intr.fy, intr.cx, intr.cy, 0.0, 0.0, 0.0, 0.0, 0.0,
    );
    let mut inside_i: Option<usize> = None;
    let mut outside_i: Option<usize> = None;
    let mut best_rpx = -1.0;
    for i in 0..model.vertex_count() {
        let p = model.vertex(i);
        let uv = rep.project_camera_point(arr(extr.to_camera(vec3(p[0], p[1], p[2]))));
        if !(uv[0] >= 0.0 && uv[0] < f64::from(rep.width()) && uv[1] >= 0.0 && uv[1] < f64::from(rep.height())) {
            continue;
        }
        let r_px = (uv[0] - rep.cx()).hypot(uv[1] - rep.cy());
        if rep.inside_fisheye_circle(uv[0], uv[1]) {
            if rep.fisheye_circle_radius() - r_px >= 40.0 && r_px > best_rpx {
                best_rpx = r_px;
                inside_i = Some(i);
            }
        } else if outside_i.is_none() {
            outside_i = Some(i);
        }
    }
    let inside_i = inside_i.expect("no vertex inside rect AND safe inside the circle");
    let outside_i = outside_i.expect("no vertex inside rect but OUTSIDE the circle");

    let p_inside = model.vertex(inside_i);
    let uv_inside = rep.project_camera_point(arr(extr.to_camera(vec3(p_inside[0], p_inside[1], p_inside[2]))));
    assert!((f64::from(model.baked_vertex(inside_i).weight[0]) - 1.0).abs() < 1e-6);
    assert!((uv_inside[0] / f64::from(rep.width()) - f64::from(model.baked_vertex(inside_i).uv[0][0])).abs() < 1e-6);
    assert!((uv_inside[1] / f64::from(rep.height()) - f64::from(model.baked_vertex(inside_i).uv[0][1])).abs() < 1e-6);

    assert_eq!(0.0f32, model.baked_vertex(outside_i).weight[0]);
    assert_eq!(0.0f32, model.baked_vertex(outside_i).uv[0][0]);
    assert_eq!(0.0f32, model.baked_vertex(outside_i).uv[0][1]);
}

#[test]
fn all_invisible_vertices_keep_zero_weight_and_report_no_visibility() {
    let intr = standard_intr();
    let extr = CameraExtrinsics::from_rows(
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        [30.0, 0.0, 5.0],
    );
    let model = BowlModel::new(
        &make_coarse_bowl_params(12.6),
        &wrap_kb_cameras(&[intr], &[extr]),
    );
    assert!(model.vertex_count() > 0);

    for i in 0..model.vertex_count() {
        let baked = model.baked_vertex(i);
        assert_eq!(0.0f32, baked.weight[0], "index {i}");
        assert_eq!(0.0f32, baked.uv[0][0], "index {i}");
        assert_eq!(0.0f32, baked.uv[0][1], "index {i}");
    }
    assert!(!model.has_visible_vertices());
}

#[test]
fn single_camera_visible_vertices_normalize_to_weight_one() {
    let intr = standard_intr();
    let extr = CameraExtrinsics::from_rows(
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        [0.0, 0.0, 5.0],
    );
    let model = BowlModel::new(&tall_bowl_params(), &wrap_kb_cameras(&[intr], &[extr]));
    assert!(model.vertex_count() > 0);

    let mut visible = 0;
    for i in 0..model.vertex_count() {
        let w = model.baked_vertex(i).weight[0];
        if w > 1e-9f32 {
            visible += 1;
            assert!((1.0 - f64::from(w)).abs() < 1e-6, "index {i}");
        }
    }
    assert!(visible > 0);
}

#[test]
fn edge_gate_unsafe_camera_is_dropped_while_safe_camera_gets_full_weight() {
    let intr = standard_intr();
    let cam0 = CameraExtrinsics::from_rows(
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        [0.0, 0.0, 5.0],
    );

    let scan = BowlModel::new(&tall_bowl_params(), &wrap_kb_cameras(std::slice::from_ref(&intr), &[cam0]));
    assert!(scan.vertex_count() > 0);
    let rep = make_kb_model(
        intr.width, intr.height, intr.fx, intr.fy, intr.cx, intr.cy, 0.0, 0.0, 0.0, 0.0, 0.0,
    );
    let mut target: Option<usize> = None;
    for i in 0..scan.vertex_count() {
        let p = scan.vertex(i);
        let uv = rep.project_camera_point(arr(cam0.to_camera(vec3(p[0], p[1], p[2]))));
        if !(uv[0] >= 0.0 && uv[0] < f64::from(rep.width()) && uv[1] >= 0.0 && uv[1] < f64::from(rep.height())) {
            continue;
        }
        let r_px = (uv[0] - rep.cx()).hypot(uv[1] - rep.cy());
        if rep.inside_fisheye_circle(uv[0], uv[1]) && rep.fisheye_circle_radius() - r_px < 40.0 {
            target = Some(i);
            break;
        }
    }
    let target = target.expect("no vertex in cam0's unsafe margin band");

    let p = *scan.vertex(target);
    let cam1 = CameraExtrinsics::from_rows(
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        [p[0], p[1], 5.0],
    );
    let model = BowlModel::new(
        &tall_bowl_params(),
        &wrap_kb_cameras(&[intr.clone(), intr.clone()], &[cam0, cam1]),
    );

    let uv0 = rep.project_camera_point(arr(cam0.to_camera(vec3(p[0], p[1], p[2]))));
    let uv1 = rep.project_camera_point(arr(cam1.to_camera(vec3(p[0], p[1], p[2]))));
    assert!((uv0[0] - rep.cx()).hypot(uv0[1] - rep.cy()) < 240.0);
    assert!(rep.fisheye_circle_radius() - (uv0[0] - rep.cx()).hypot(uv0[1] - rep.cy()) < 40.0);
    assert!((uv1[0] - rep.cx()).hypot(uv1[1] - rep.cy()) < 1e-3);

    let baked = model.baked_vertex(target);
    assert_eq!(0.0f32, baked.weight[0]);
    assert!((1.0f32 - baked.weight[1]).abs() < 1e-6);
    assert!((1.0 - f64::from(baked.weight[0]) - f64::from(baked.weight[1])).abs() < 1e-6);
}

#[test]
fn car_center_shifts_the_blend_weight_distribution() {
    let intrs = default_intrinsics();
    let extrs = side_cameras(&[0.0, 90.0, 180.0, -90.0]);
    let cams = wrap_kb_cameras(&intrs, &extrs);

    let base = make_coarse_bowl_params(12.6);
    let shifted = BowlParams::new(
        5.6,
        2.2,
        base.flat_platform_radius(),
        1.0,
        12.6,
        base.slope_coefficient(),
        base.height_exponent(),
        base.max_height(),
        [3.0, 0.0],
    );
    let def = BowlModel::new(&base, &cams);
    let sh = BowlModel::new(&shifted, &cams);

    let find_world_diag = |m: &BowlModel| -> Option<usize> {
        for i in 0..m.vertex_count() {
            let p = m.vertex(i);
            if !(p[0] > 3.0 && p[1] > 3.0 && (p[0] - p[1]).abs() < 1.0) {
                continue;
            }
            if is_vertex_visible_to_camera(p, &intrs[0], &extrs[0])
                && is_vertex_visible_to_camera(p, &intrs[1], &extrs[1])
            {
                return Some(i);
            }
        }
        None
    };

    let di_def = find_world_diag(&def).expect("no world-diagonal vertex visible to front+left");
    let bv_def = def.baked_vertex(di_def);
    let sum_def: f64 = bv_def.weight[..4].iter().copied().map(f64::from).sum();
    assert!(bv_def.weight[0] > 0.4f32);
    assert!(bv_def.weight[1] > 0.4f32);
    assert!((1.0 - sum_def).abs() < 1e-6);

    let di_sh = find_world_diag(&sh).expect("no world-diagonal vertex in shifted bowl");
    let bv_sh = sh.baked_vertex(di_sh);
    let sum_sh: f64 = bv_sh.weight[..4].iter().copied().map(f64::from).sum();
    assert!(bv_sh.weight[1] > bv_sh.weight[0]);
    assert!((1.0 - sum_sh).abs() < 1e-6);
}

#[test]
fn has_visible_vertices_is_true_when_the_rim_is_visible() {
    let model = make_coarse_bowl(12.6, &default_intrinsics(), &default_bake_cameras());
    assert!(model.has_visible_vertices());
}

#[test]
fn fallback_normalization_gives_weight_to_a_visible_neighbour_only() {
    let intr = standard_intr();
    let cam0 = CameraExtrinsics::from_rows(
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        [0.0, 0.0, 5.0],
    );
    let cam1 = make_look_at_camera(0.0, 6.0, 2.0, 0.0);

    let model = BowlModel::new(
        &make_coarse_bowl_params(12.6),
        &wrap_kb_cameras(&[intr.clone(), intr.clone()], &[cam0, cam1]),
    );
    assert!(model.vertex_count() > 0);

    let mut target: Option<usize> = None;
    for i in 0..model.vertex_count() {
        let p = *model.vertex(i);
        if !is_vertex_visible_to_camera(&p, &intr, &cam0)
            && is_vertex_baked_visible(&p, &intr, &cam1)
        {
            target = Some(i);
            break;
        }
    }
    let target = target.expect("no vertex that cam0 misses but cam1 bakes visible");

    let baked = model.baked_vertex(target);
    let sum = f64::from(baked.weight[0]) + f64::from(baked.weight[1]);
    assert!((1.0 - sum).abs() < 1e-6);
    assert_eq!(0.0f32, baked.weight[0]);
    assert!(baked.weight[1] > 0.0f32);
}

#[test]
fn vertex_below_the_camera_still_gets_weight() {
    let intr = standard_intr();
    let extr = CameraExtrinsics::from_rows(
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        [0.0, 0.0, 5.0],
    );
    let model = BowlModel::new(
        &make_coarse_bowl_params(5.0),
        &wrap_kb_cameras(std::slice::from_ref(&intr), &[extr]),
    );
    assert!(model.vertex_count() > 0);

    let mut target: Option<usize> = None;
    for i in 0..model.vertex_count() {
        let p = *model.vertex(i);
        if p[2] >= 5.0 {
            continue;
        }
        let p_cam = extr.to_camera(vec3(p[0], p[1], p[2]));
        let r_xy = p_cam[0].hypot(p_cam[1]);
        if r_xy < avm_core::camera::EPS && is_vertex_visible_to_camera(&p, &intr, &extr) {
            target = Some(i);
            break;
        }
    }
    let target = target.expect("no degenerate centre vertex below the camera");
    assert!(model.baked_vertex(target).weight[0] > 0.0f32);
}

#[test]
fn edge_gate_zeroes_weight_when_all_cameras_are_unsafe() {
    let intr = standard_intr();
    let cam = CameraExtrinsics::from_rows(
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        [0.0, 0.0, 5.0],
    );
    let model = BowlModel::new(&tall_bowl_params(), &wrap_kb_cameras(std::slice::from_ref(&intr), &[cam]));
    assert!(model.vertex_count() > 0);

    let rep = make_kb_model(
        intr.width, intr.height, intr.fx, intr.fy, intr.cx, intr.cy, 0.0, 0.0, 0.0, 0.0, 0.0,
    );
    let mut target: Option<usize> = None;
    for i in 0..model.vertex_count() {
        let p = model.vertex(i);
        let uv = rep.project_camera_point(arr(cam.to_camera(vec3(p[0], p[1], p[2]))));
        if !(uv[0] >= 0.0 && uv[0] < f64::from(rep.width()) && uv[1] >= 0.0 && uv[1] < f64::from(rep.height())) {
            continue;
        }
        let r_px = (uv[0] - rep.cx()).hypot(uv[1] - rep.cy());
        if rep.inside_fisheye_circle(uv[0], uv[1]) && rep.fisheye_circle_radius() - r_px < 40.0 {
            target = Some(i);
            break;
        }
    }
    let target = target.expect("no vertex in the unsafe margin band");
    assert_eq!(0.0f32, model.baked_vertex(target).weight[0]);
}

#[test]
fn lut_camera_bakes_visible_uv() {
    let lut = LutParams {
        points: vec![[0.0, 0.0], [120.0, 4.0]],
        width: 640,
        height: 480,
        cx: 320.0,
        cy: 240.0,
        ..LutParams::default()
    };

    let extr = side_cameras(&[0.0])[0];
    let cam = Camera::new(
        extr,
        make_camera_model("table", &CameraModelParams::Lut(lut)).expect("table model"),
    );

    let params = BowlParams::new(2.0, 1.0, 0.5, 0.2, 4.0, 0.1, 2.2, 4.0, [0.0, 0.0]);
    let model = BowlModel::new(&params, &[cam]);
    assert!(model.vertex_count() > 0);
    assert_eq!(1, model.baked_num_cameras());

    let mut weighted = 0;
    for i in 0..model.vertex_count() {
        let baked = model.baked_vertex(i);
        if baked.weight[0] > 0.0f32 {
            weighted += 1;
            assert!((0.0..=1.0).contains(&baked.uv[0][0]));
            assert!((0.0..=1.0).contains(&baked.uv[0][1]));
        }
    }
    assert!(weighted > 0);
}
