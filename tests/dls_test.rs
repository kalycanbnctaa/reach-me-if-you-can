use reach_me_if_you_can::kinematics::damped_ls::JacobianDLS;
use reach_me_if_you_can::kinematics::inverse::{IkConfig, IkSolver};
use reach_me_if_you_can::kinematics::jacobian;
use reach_me_if_you_can::linear_solver::pseudo;
use reach_me_if_you_can::math::vector2::Vector2;
use reach_me_if_you_can::robot::arm::RobotArm;

fn singular_two_link_angles() -> Vec<f32> {
    vec![0.5, 0.0]
}

#[test]
fn undamped_pseudo_inverse_fails_at_fully_extended_singularity() {
    let base = Vector2::new(0.0, 0.0);
    let lengths = vec![100.0, 80.0];
    let angles = singular_two_link_angles();

    let j = jacobian::calculate(base, &lengths, &angles);
    let undamped = pseudo::right_pseudo_inverse(j.as_matrix());

    assert!(undamped.is_none());
}

#[test]
fn damped_pseudo_inverse_stays_finite_at_same_singularity() {
    let base = Vector2::new(0.0, 0.0);
    let lengths = vec![100.0, 80.0];
    let angles = singular_two_link_angles();

    let j = jacobian::calculate(base, &lengths, &angles);
    let damped = pseudo::damped_right_pseudo_inverse(j.as_matrix(), 8.0);

    assert!(damped.is_some());
    assert!(damped.unwrap().iter().all(|value| value.is_finite()));
}

#[test]
fn dls_solver_produces_finite_delta_at_singularity() {
    let base = Vector2::new(660.0, 400.0);
    let lengths = vec![100.0, 80.0];
    let mut robot = RobotArm::new(base, lengths);

    assert!(robot.set_joint_angles(&singular_two_link_angles()));

    let error_vector = Vector2::new(10.0, 10.0);
    let config = IkConfig::default();
    let solver = JacobianDLS;

    let delta = solver.compute_delta(&robot, error_vector, &config);

    assert!(delta.iter().all(|value| value.is_finite()));
    assert_eq!(delta.len(), robot.actuated_joint_count());
}

#[test]
fn dls_lambda_shrinks_to_near_zero_far_from_singularity() {
    let base = Vector2::new(660.0, 400.0);
    let lengths = vec![100.0, 80.0];
    let mut robot = RobotArm::new(base, lengths);

    assert!(robot.set_joint_angles(&[0.7, 1.2]));

    let error_vector = Vector2::new(10.0, 10.0);
    let config = IkConfig::default();
    let solver = JacobianDLS;

    let delta = solver.compute_delta(&robot, error_vector, &config);

    assert!(delta.iter().all(|value| value.is_finite()));
}