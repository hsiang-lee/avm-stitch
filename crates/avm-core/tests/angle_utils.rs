use avm_core::angle_utils::{normalize_angle_deg, DEG_TO_RAD, PI, RAD_TO_DEG};

#[test]
fn normalize_keeps_in_range_values_unchanged() {
    assert_eq!(0.0, normalize_angle_deg(0.0));
    assert_eq!(180.0, normalize_angle_deg(180.0));
    assert_eq!(357.0, normalize_angle_deg(357.0));
}

#[test]
fn normalize_wraps_exact_multiple_of_360_to_zero() {
    assert_eq!(0.0, normalize_angle_deg(360.0));
    assert_eq!(0.0, normalize_angle_deg(720.0));
}

#[test]
fn normalize_adds_360_to_negative_angles() {
    assert_eq!(315.0, normalize_angle_deg(-45.0));
    assert_eq!(180.0, normalize_angle_deg(-180.0));
}

#[test]
fn normalize_maps_negative_multiple_of_360_to_zero() {
    assert_eq!(0.0, normalize_angle_deg(-360.0));
    assert_eq!(0.0, normalize_angle_deg(-720.0));
}

#[test]
fn normalize_wraps_multiple_turns_into_range() {
    assert_eq!(30.0, normalize_angle_deg(750.0));
    assert_eq!(305.0, normalize_angle_deg(-415.0));
    assert_eq!(0.0, normalize_angle_deg(1080.0));
}

#[test]
fn normalize_preserves_non_integer_degrees() {
    assert_eq!(123.456, normalize_angle_deg(123.456));
    assert_eq!(10.5, normalize_angle_deg(370.5));
}

#[test]
fn constants_have_cpp_values() {
    assert_eq!(0x4009_21FB_5444_2D18u64, PI.to_bits());
    assert_eq!(0.017453292519943295, DEG_TO_RAD);
    assert_eq!(57.29577951308232, RAD_TO_DEG);
}

#[test]
fn constants_convert_symmetrically() {
    assert!((180.0 - RAD_TO_DEG * PI).abs() < 1e-12);
    assert!((PI - DEG_TO_RAD * 180.0).abs() < 1e-12);
}
