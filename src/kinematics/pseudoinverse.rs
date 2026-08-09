use nalgebra::DMatrix;

use crate::{
    kinematics::inverse::{IkConfig, IkSolver},
    linear_solver::pseudo,
    math::vector2::Vector2,
    robot::arm::RobotArm,
};

const STABILITY_LAMBDA: f32 = 8.0;

pub struct JacobianPseudoInverse;

impl IkSolver for JacobianPseudoInverse {
    fn compute_delta(
        &self,
        robot: &RobotArm,
        error_vector: Vector2,
        config: &IkConfig,
    ) -> Vec<f32> {
        let jacobian = robot.jacobian();
        let joint_count = jacobian.joint_count();
        let matrix = jacobian.as_matrix();

        let pseudo_inverse = if robot.state.is_singular_now {
            pseudo::damped_right_pseudo_inverse(matrix, STABILITY_LAMBDA)
        } else if robot.state.near_singular {
            pseudo::damped_right_pseudo_inverse(matrix, STABILITY_LAMBDA * 0.5)
        } else {
            pseudo::right_pseudo_inverse(matrix)
        };

        let Some(pseudo_inverse) = pseudo_inverse else {
            return vec![f32::NAN; joint_count];
        };

        let error_matrix =
            DMatrix::from_vec(2, 1, vec![error_vector.x, error_vector.y]);
        let delta = pseudo_inverse * error_matrix;

        (0..joint_count)
            .map(|index| config.pseudo_step_size * delta[(index, 0)])
            .collect()
    }
}

pub struct JacobianPseudoInverseUndamped;

impl IkSolver for JacobianPseudoInverseUndamped {
    fn compute_delta(
        &self,
        robot: &RobotArm,
        error_vector: Vector2,
        config: &IkConfig,
    ) -> Vec<f32> {
        let jacobian = robot.jacobian();
        let joint_count = jacobian.joint_count();
        let matrix = jacobian.as_matrix();

        let Some(pseudo_inverse) = pseudo::right_pseudo_inverse(matrix) else {
            return vec![f32::NAN; joint_count];
        };

        let error_matrix =
            DMatrix::from_vec(2, 1, vec![error_vector.x, error_vector.y]);
        let delta = pseudo_inverse * error_matrix;

        (0..joint_count)
            .map(|index| config.pseudo_step_size * delta[(index, 0)])
            .collect()
    }
}