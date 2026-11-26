pub const PI: f64 = std::f64::consts::PI;
pub const DEG_TO_RAD: f64 = PI / 180.0;
pub const RAD_TO_DEG: f64 = 180.0 / PI;

/// Normalize an angle in degrees to `[0, 360)`.
pub fn normalize_angle_deg(deg: f64) -> f64 {
    let mut deg = deg % 360.0;
    if deg < 0.0 {
        deg += 360.0;
    }
    deg
}
