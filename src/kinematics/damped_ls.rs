use nalgebra::DMatrix;

use crate::{
    kinematics::{
        inverse::{IkConfig, IkSolver},
        singularity,
    },
    linear_solver::pseudo,
    math::vector2::Vector2,
    robot::arm::RobotArm,
};

pub struct JacobianDLS;

impl JacobianDLS {
    fn compute_lambda(normalized_manipulability: f32, config: &IkConfig) -> f32 {
        let threshold = config.dls_threshold.max(f32::EPSILON);
        let ratio = (normalized_manipulability / threshold).clamp(0.0, 1.0);
        let lambda_squared =
            (1.0 - ratio * ratio) * config.dls_lambda_max * config.dls_lambda_max;

        lambda_squared.max(0.0).sqrt()
    }
}

impl IkSolver for JacobianDLS {
    fn compute_delta(
        &self,
        robot: &RobotArm,
        error_vector: Vector2,
        config: &IkConfig,
    ) -> Vec<f32> {
        let jacobian = robot.jacobian();
        let joint_count = jacobian.joint_count();
        let matrix = jacobian.as_matrix();

        let arm_scale = robot.total_length().max(1.0);
        let report = singularity::analyze(&jacobian, arm_scale);
        let lambda = Self::compute_lambda(report.normalized_manipulability, config);

        let Some(pseudo_inverse) = pseudo::damped_right_pseudo_inverse(matrix, lambda) else {
            return vec![f32::NAN; joint_count];
        };

        let error_matrix =
            DMatrix::from_vec(2, 1, vec![error_vector.x, error_vector.y]);
        let delta = pseudo_inverse * error_matrix;

        (0..joint_count)
            .map(|index| config.dls_step_size * delta[(index, 0)])
            .collect()
    }
}