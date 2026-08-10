use crate::{
    collision::{
        intersection,
        obstacle::Obstacle,
        potential_field::{self, PotentialFieldConfig},
    },
    kinematics::{
        inverse::{IkConfig, IkSolver},
        singularity,
    },
    math::vector2::Vector2,
    robot::arm::RobotArm,
};

const LIMIT_BOUNDARY_EPSILON: f32 = 1.0e-3;
const NEAR_SINGULAR_LOG_INTERVAL: usize = 20;

pub fn step(
    robot: &mut RobotArm,
    target: Vector2,
    solver: &dyn IkSolver,
    config: &IkConfig,
    obstacles: &[Obstacle],
    field_config: &PotentialFieldConfig,
) {
    if robot.state.converged || robot.state.unreachable || robot.state.stalled {
        return;
    }

    let solver_name = std::any::type_name_of_val(solver);

    if !robot.is_within_reach(target) {
        let error = Vector2::distance(robot.end_effector(), target);
        println!(
            "[SOLVER {}] unreachable at iteration {}, error={:.4}",
            solver_name, robot.state.iteration, error
        );
        robot.state.mark_unreachable(error, robot.state.iteration);
        return;
    }

    let error_vector = target - robot.end_effector();
    let error = error_vector.length();

    robot.state.colliding =
        intersection::arm_intersects_any(&robot.pose.joint_positions, obstacles);

    robot.state.segment_collision = intersection::arm_segments_intersecting(
        &robot.pose.joint_positions,
        obstacles,
    );

    if error <= config.position_tolerance {
        println!(
            "[SOLVER {}] converged in {} iterations, error={:.4}",
            solver_name, robot.state.iteration, error
        );
        robot.state.mark_converged(error, robot.state.iteration);
        robot.state.push_error(error);
        return;
    }

    if robot.state.iteration >= config.max_iterations {
        let blocked = is_blocked_by_limits(robot);
        println!(
            "[SOLVER {}] stalled at iteration {}, error={:.4}, blocked_by_limits={}",
            solver_name, robot.state.iteration, error, blocked
        );
        robot.state.mark_stalled(error, robot.state.iteration);
        robot.state.set_constraint_blocked(blocked);
        robot.state.push_error(error);
        return;
    }

    let jacobian = robot.jacobian();
    let arm_scale = robot.total_length().max(1.0);
    let report = singularity::analyze(&jacobian, arm_scale);
    robot.state.manipulability = report.manipulability;
    robot.state.near_singular = report.is_near_singular;
    robot.state.is_singular_now = report.is_singular;

    if report.is_near_singular && robot.state.iteration % NEAR_SINGULAR_LOG_INTERVAL == 0 {
        println!(
            "[SOLVER {}] near singular at iteration {}, manipulability={:.6}",
            solver_name, robot.state.iteration, report.manipulability
        );
    }

    let mut delta_theta = solver.compute_delta(robot, error_vector, config);

    if !delta_theta.iter().all(|delta| delta.is_finite()) {
        println!(
            "[SOLVER {}] singular/NaN at iteration {}, error={:.4}",
            solver_name, robot.state.iteration, error
        );
        robot.state.mark_singular(error, robot.state.iteration);
        robot.state.push_error(error);
        return;
    }

    if !obstacles.is_empty() {
        let repulsive_delta = potential_field::compute_delta(robot, obstacles, field_config);

        for (delta, repulsive) in delta_theta.iter_mut().zip(repulsive_delta.iter()) {
            *delta += config.obstacle_step_size * repulsive;
            if !delta.is_finite() {
                *delta = 0.0;
            }
        }
    }

    for delta in delta_theta.iter_mut() {
        *delta = delta.clamp(-config.max_delta_angle, config.max_delta_angle);
    }

    let new_angles: Vec<f32> = robot
        .state
        .joint_angles
        .iter()
        .zip(delta_theta.iter())
        .map(|(angle, delta)| angle + delta)
        .collect();

    if !robot.set_joint_angles(&new_angles) {
        println!(
            "[SOLVER {}] rejected angle update at iteration {}, error={:.4}",
            solver_name, robot.state.iteration, error
        );
        robot.state.mark_singular(error, robot.state.iteration);
        robot.state.push_error(error);
        return;
    }

    robot.state.iteration += 1;
    robot.state.current_error = Vector2::distance(robot.end_effector(), target);
    robot.state.push_error(robot.state.current_error);
}

fn is_blocked_by_limits(robot: &RobotArm) -> bool {
    robot
        .limits
        .iter()
        .zip(robot.state.joint_angles.iter())
        .any(|(limit, &angle)| {
            !limit.is_unrestricted()
                && ((angle - limit.min_angle).abs() <= LIMIT_BOUNDARY_EPSILON
                    || (limit.max_angle - angle).abs() <= LIMIT_BOUNDARY_EPSILON)
        })
}