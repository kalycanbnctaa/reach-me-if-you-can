use std::f32::consts::PI;

use crate::math::utils::{clamp, normalize_angle};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointLimit {
    pub min_angle: f32,
    pub max_angle: f32,
}

impl JointLimit {
    pub fn unrestricted() -> Self {
        Self {
            min_angle: -PI,
            max_angle: PI,
        }
    }

    pub fn new(min_angle: f32, max_angle: f32) -> Self {
        assert!(
            min_angle.is_finite() && max_angle.is_finite(),
            "Joint limits must be finite"
        );

        assert!(
            min_angle <= max_angle,
            "Joint minimum angle must not exceed maximum angle"
        );

        Self {
            min_angle,
            max_angle,
        }
    }

    pub fn contains(&self, angle: f32) -> bool {
        angle.is_finite()
            && angle >= self.min_angle
            && angle <= self.max_angle
    }

    pub fn clamp(&self, angle: f32) -> f32 {
        if !angle.is_finite() {
            return self.min_angle;
        }

        clamp(angle, self.min_angle, self.max_angle)
    }

    pub fn clamp_normalized(&self, angle: f32) -> f32 {
        self.clamp(normalize_angle(angle))
    }

    pub fn range(&self) -> f32 {
        self.max_angle - self.min_angle
    }

    pub fn is_unrestricted(&self) -> bool {
        (self.min_angle + PI).abs() <= f32::EPSILON
            && (self.max_angle - PI).abs() <= f32::EPSILON
    }
}

impl Default for JointLimit {
    fn default() -> Self {
        Self::unrestricted()
    }
}