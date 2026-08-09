use crate::{
    kinematics::jacobian::Jacobian,
    linear_solver::determinant,
};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SingularityReport {
    pub manipulability: f32,
    pub normalized_manipulability: f32,
    pub is_singular: bool,
    pub is_near_singular: bool,
}

pub const SINGULAR_RATIO: f32 = 1.0e-4;
pub const NEAR_SINGULAR_RATIO: f32 = 0.05;

pub fn analyze(jacobian: &Jacobian, arm_scale: f32) -> SingularityReport {
    let j = jacobian.as_matrix();
    let jjt = j.clone() * j.transpose();
    let det = determinant::determinant(&jjt);
    let manipulability = det.max(0.0).sqrt();

    let scale_squared = (arm_scale * arm_scale).max(f32::EPSILON);
    let normalized_manipulability = manipulability / scale_squared;

    let is_singular =
        !manipulability.is_finite() || normalized_manipulability <= SINGULAR_RATIO;

    let is_near_singular =
        !is_singular && normalized_manipulability <= NEAR_SINGULAR_RATIO;

    SingularityReport {
        manipulability,
        normalized_manipulability,
        is_singular,
        is_near_singular,
    }
}