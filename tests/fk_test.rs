use reach_me_if_you_can::kinematics::forward;
use reach_me_if_you_can::math::vector2::Vector2;

#[test]
fn fk_two_link_zero_angles_extends_along_x_axis() {
    let base = Vector2::new(0.0, 0.0);
    let lengths = vec![100.0, 50.0];
    let angles = vec![0.0, 0.0];

    let pose = forward::forward_kinematics(base, &lengths, &angles);

    assert!((pose.end_effector().x - 150.0).abs() < 1e-4);
    assert!((pose.end_effector().y - 0.0).abs() < 1e-4);
}

#[test]
fn fk_two_link_quarter_turn_first_joint() {
    let base = Vector2::new(0.0, 0.0);
    let lengths = vec![100.0, 50.0];
    let angles = vec![std::f32::consts::FRAC_PI_2, 0.0];

    let pose = forward::forward_kinematics(base, &lengths, &angles);

    assert!((pose.end_effector().x - 0.0).abs() < 1e-3);
    assert!((pose.end_effector().y - 150.0).abs() < 1e-3);
}

#[test]
fn fk_returns_correct_joint_count_and_base_position() {
    let base = Vector2::new(10.0, 20.0);
    let lengths = vec![30.0, 40.0, 50.0];
    let angles = vec![0.1, 0.2, 0.3];

    let pose = forward::forward_kinematics(base, &lengths, &angles);

    assert_eq!(pose.joint_count(), 4);
    assert_eq!(pose.joint_position(0), Some(base));
}

#[test]
fn fk_end_effector_matches_pose_end_effector() {
    let base = Vector2::new(5.0, 5.0);
    let lengths = vec![60.0, 40.0, 30.0, 20.0];
    let angles = vec![0.4, -0.6, 0.9, -0.2];

    let end_effector = forward::end_effector(base, &lengths, &angles);
    let pose = forward::forward_kinematics(base, &lengths, &angles);

    assert_eq!(end_effector, pose.end_effector());
}