use crate::{
    math::vector2::Vector2,
    robot::arm::RobotArm,
};

#[derive(Debug, Clone, Copy)]
pub struct IkConfig {
    pub max_iterations: usize,
    pub position_tolerance: f32,
    pub step_size: f32,
    pub pseudo_step_size: f32,
    pub max_delta_angle: f32,
    pub dls_lambda_max: f32,
    pub dls_threshold: f32,
    pub dls_step_size: f32,
    pub obstacle_step_size: f32,
}

impl Default for IkConfig {
    fn default() -> Self {
        Self {
            max_iterations: 500,
            position_tolerance: 1.5,
            step_size: 0.00004,
            pseudo_step_size: 1.0,
            max_delta_angle: 0.12,
            dls_lambda_max: 12.0,
            dls_threshold: 0.05,
            dls_step_size: 1.0,
            obstacle_step_size: 0.00003,
        }
    }
}

pub trait IkSolver {
    fn compute_delta(
        &self,
        robot: &RobotArm,
        error_vector: Vector2,
        config: &IkConfig,
    ) -> Vec<f32>;
}