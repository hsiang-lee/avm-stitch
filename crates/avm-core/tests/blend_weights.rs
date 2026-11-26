mod common;

use avm_core::bowl::BlendWeightsCalculator;
use common::{make_default_kb_cameras, wrap_kb_side_cameras};

fn calculator_default() -> BlendWeightsCalculator {
    BlendWeightsCalculator::with_default_pole(&make_default_kb_cameras())
}

#[test]
fn default_pole_matches_explicit_zero_pole() {
    let cams = make_default_kb_cameras();
    let calc_default = BlendWeightsCalculator::with_default_pole(&cams);
    let calc_explicit = BlendWeightsCalculator::new(&cams, [0.0, 0.0]);
    let a = calc_default.weights_for([5.0, 0.0, 0.0]);
    let b = calc_explicit.weights_for([5.0, 0.0, 0.0]);
    assert_eq!(a.len(), b.len());
    for i in 0..a.len() {
        assert!((a[i] - b[i]).abs() < 1e-9, "index {i}");
    }
}

#[test]
fn non_default_pole_shifts_the_origin() {
    let cams = make_default_kb_cameras();
    let calc = BlendWeightsCalculator::new(&cams, [2.0, 0.0]);
    let weights = calc.weights_for([2.0, 1.0, 0.0]);
    assert_eq!(4, weights.len());
    assert!(weights[2] > 0.9);
    assert!(weights[0] < 0.1);
}

#[test]
fn four_standard_cameras_select_the_front_camera() {
    let calc = calculator_default();
    let weights = calc.weights_for([5.0, 0.0, 0.0]);
    assert_eq!(4, weights.len());
    assert!(weights[0] > 0.9);
    assert!(weights[1] < 0.1);
}

#[test]
fn four_standard_cameras_select_the_rear_camera() {
    let calc = calculator_default();
    let weights = calc.weights_for([-5.0, 0.0, 0.0]);
    assert!(weights[1] > 0.9);
    assert!(weights[0] < 0.1);
}

#[test]
fn four_camera_blend_half_angle_is_13_degrees() {
    assert!((13.0 - BlendWeightsCalculator::compute_blend_half_angle(4)).abs() < 1e-9);
}

#[test]
fn four_camera_diagonal_blends_two_cameras_to_one() {
    let calc = calculator_default();
    let weights = calc.weights_for([5.0, 5.0, 0.0]);
    let positive = weights.iter().filter(|w| **w > 0.0).count();
    let sum: f64 = weights.iter().sum();
    assert_eq!(2, positive);
    assert!((1.0 - sum).abs() < 1e-6);
}

#[test]
fn four_camera_cosine_smoothstep_is_half_half_on_the_boundary() {
    let calc = calculator_default();
    let weights = calc.weights_for([5.0, 5.0, 0.0]);
    assert!((0.5 - weights[0]).abs() < 1e-6);
    assert!((0.5 - weights[2]).abs() < 1e-6);
    assert!((0.0 - weights[1]).abs() < 1e-6);
    assert!((0.0 - weights[3]).abs() < 1e-6);
}

#[test]
fn four_camera_point_outside_the_blend_band_has_a_single_owner() {
    let calc = calculator_default();
    let angle = 65.0f64 * std::f64::consts::PI / 180.0;
    let weights = calc.weights_for([angle.cos(), angle.sin(), 0.0]);
    let ones = weights.iter().filter(|w| **w > 0.999).count();
    assert_eq!(1, ones);
}

#[test]
fn eight_camera_blend_half_angle_is_6_5_degrees() {
    assert!((6.5 - BlendWeightsCalculator::compute_blend_half_angle(8)).abs() < 1e-9);
}

#[test]
fn eight_camera_weights_sum_to_one_everywhere() {
    let cams = wrap_kb_side_cameras(&[0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0]);
    let calc = BlendWeightsCalculator::with_default_pole(&cams);
    let points = [
        [5.0, 0.0, 0.0],
        [0.0, 5.0, 0.0],
        [-5.0, 0.0, 0.0],
        [0.0, -5.0, 0.0],
        [5.0, 5.0, 0.0],
        [-5.0, 5.0, 0.0],
        [-5.0, -5.0, 0.0],
        [5.0, -5.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [1.0, 1.0, 0.0],
        [-1.0, 1.0, 0.0],
    ];
    for p in points {
        let sum: f64 = calc.weights_for(p).iter().sum();
        assert!((1.0 - sum).abs() < 1e-6, "point {p:?}");
    }
}

#[test]
fn three_camera_arbitrary_spacing_weights_sum_to_one() {
    let cams = wrap_kb_side_cameras(&[0.0, 120.0, 240.0]);
    let calc = BlendWeightsCalculator::with_default_pole(&cams);
    let points = [
        [5.0, 0.0, 0.0],
        [0.0, 5.0, 0.0],
        [-5.0, 0.0, 0.0],
        [0.0, -5.0, 0.0],
        [5.0, 5.0, 0.0],
    ];
    for p in points {
        let sum: f64 = calc.weights_for(p).iter().sum();
        assert!((1.0 - sum).abs() < 1e-6, "point {p:?}");
    }
}

#[test]
fn all_weights_are_non_negative() {
    let calc = calculator_default();
    let points = [
        [5.0, 0.0, 0.0],
        [0.0, 5.0, 0.0],
        [-5.0, 0.0, 0.0],
        [0.0, -5.0, 0.0],
        [5.0, 5.0, 0.0],
        [-5.0, 5.0, 0.0],
        [-5.0, -5.0, 0.0],
        [5.0, -5.0, 0.0],
    ];
    for p in points {
        for w in calc.weights_for(p) {
            assert!(w >= 0.0, "point {p:?}");
        }
    }
}

#[test]
fn single_camera_always_has_full_weight() {
    let cams = wrap_kb_side_cameras(&[0.0]);
    let calc = BlendWeightsCalculator::with_default_pole(&cams);
    let weights = calc.weights_for([5.0, 0.0, 0.0]);
    assert_eq!(1, weights.len());
    assert!((1.0 - weights[0]).abs() < 1e-9);
}

#[test]
fn empty_camera_list_returns_empty_weights() {
    let calc = BlendWeightsCalculator::with_default_pole(&[]);
    assert!(calc.weights_for([0.0, 0.0, 0.0]).is_empty());
}

#[test]
fn normalize_weights_scales_blend_weights_by_edge_factor() {
    let cams = wrap_kb_side_cameras(&[0.0, 180.0]);
    let calc = BlendWeightsCalculator::with_default_pole(&cams);
    let mut normalized = [0.0f32; 2];
    calc.normalize_weights(&[0.6, 0.4], &[1.0, 0.5], &mut normalized);
    assert!((0.75f32 - normalized[0]).abs() < 1e-6);
    assert!((0.25f32 - normalized[1]).abs() < 1e-6);
}

#[test]
fn normalize_weights_falls_back_to_edge_factors_when_blend_is_zero() {
    let cams = wrap_kb_side_cameras(&[0.0, 180.0]);
    let calc = BlendWeightsCalculator::with_default_pole(&cams);
    let mut normalized = [0.0f32; 2];
    calc.normalize_weights(&[0.0, 0.0], &[0.25, 0.75], &mut normalized);
    assert!((0.25f32 - normalized[0]).abs() < 1e-6);
    assert!((0.75f32 - normalized[1]).abs() < 1e-6);
}

#[test]
fn normalize_weights_drops_a_gated_camera_share() {
    let cams = wrap_kb_side_cameras(&[0.0, 180.0]);
    let calc = BlendWeightsCalculator::with_default_pole(&cams);
    let mut normalized = [0.0f32; 2];
    calc.normalize_weights(&[0.5, 0.5], &[0.0, 1.0], &mut normalized);
    assert_eq!(0.0f32, normalized[0]);
    assert_eq!(1.0f32, normalized[1]);
}

#[test]
fn normalize_weights_leaves_output_untouched_without_visible_camera() {
    let cams = wrap_kb_side_cameras(&[0.0, 180.0]);
    let calc = BlendWeightsCalculator::with_default_pole(&cams);
    let mut normalized = [0.0f32; 2];
    calc.normalize_weights(&[0.0, 0.0], &[0.0, 0.0], &mut normalized);
    assert_eq!(0.0f32, normalized[0]);
    assert_eq!(0.0f32, normalized[1]);
}
