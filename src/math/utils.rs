use std::f32::consts::PI;

pub const DEFAULT_EPSILON: f32 = 1.0e-6;

pub fn clamp(value: f32, min: f32, max: f32) -> f32 {
    if min > max {
        return min;
    }

    value.clamp(min, max)
}

pub fn deg_to_rad(degree: f32) -> f32 {
    degree.to_radians()
}

pub fn rad_to_deg(radian: f32) -> f32 {
    radian.to_degrees()
}

pub fn normalize_angle(angle: f32) -> f32 {
    if !angle.is_finite() {
        return 0.0;
    }

    let mut normalized = angle % (2.0 * PI);

    if normalized > PI {
        normalized -= 2.0 * PI;
    } else if normalized < -PI {
        normalized += 2.0 * PI;
    }

    normalized
}

pub fn is_near_zero(
    value: f32,
    epsilon: f32,
) -> bool {
    value.abs() <= epsilon
}

pub fn approximately_equal(
    a: f32,
    b: f32,
    epsilon: f32,
) -> bool {
    (a - b).abs() <= epsilon
}