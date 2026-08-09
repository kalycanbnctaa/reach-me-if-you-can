use crate::{
    kinematics::inverse::{IkConfig, IkSolver},
    math::vector2::Vector2,
    robot::arm::RobotArm,
};

pub struct JacobianTranspose;

impl IkSolver for JacobianTranspose {
    fn compute_delta(
        &self,
        robot: &RobotArm,
        error_vector: Vector2,
        config: &IkConfig,
    ) -> Vec<f32> {
        let jacobian = robot.jacobian();
        let joint_count = jacobian.joint_count();

        (0..joint_count)
            .map(|joint_index| {
                let jx = jacobian.partial_x(joint_index).unwrap_or(0.0);
                let jy = jacobian.partial_y(joint_index).unwrap_or(0.0);

                let dot = jx * error_vector.x + jy * error_vector.y;

                config.step_size * dot
            })
            .collect()
    }
}