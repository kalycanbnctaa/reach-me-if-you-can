use crate::{
    math::{
        transform::Transform,
        vector2::Vector2,
    },
    robot::{
        arm::RobotArm,
        pose::RobotPose,
    },
};

pub fn forward_kinematics(
    base_position: Vector2,
    segment_lengths: &[f32],
    joint_angles: &[f32],
) -> RobotPose {
    assert_eq!(
        segment_lengths.len(),
        joint_angles.len(),
        "Segment count and joint angle count must match"
    );

    assert!(
        !segment_lengths.is_empty(),
        "Robot must contain at least one segment"
    );

    assert!(
        base_position.is_finite(),
        "Base position must be finite"
    );

    assert!(
        segment_lengths
            .iter()
            .all(|length| length.is_finite() && *length > 0.0),
        "All segment lengths must be finite and greater than zero"
    );

    assert!(
        joint_angles.iter().all(|angle| angle.is_finite()),
        "All joint angles must be finite"
    );

    let mut positions = Vec::with_capacity(segment_lengths.len() + 1);
    positions.push(base_position);

    let mut world_transform =
        Transform::translation(base_position);

    for (&length, &joint_angle) in
        segment_lengths.iter().zip(joint_angles.iter())
    {
        let local_translation = Vector2::from_angle(joint_angle) * length;

        world_transform = world_transform
            * Transform::from_rotation_translation(
                joint_angle,
                local_translation,
            );

        positions.push(world_transform.position());
    }

    RobotPose::from_positions(positions)
}

pub fn update_pose(robot: &mut RobotArm) {
    let lengths: Vec<f32> = robot
        .segments
        .iter()
        .map(|segment| segment.length)
        .collect();

    robot.pose = forward_kinematics(
        robot.base_position,
        &lengths,
        &robot.state.joint_angles,
    );
}

pub fn end_effector(
    base_position: Vector2,
    segment_lengths: &[f32],
    joint_angles: &[f32],
) -> Vector2 {
    forward_kinematics(
        base_position,
        segment_lengths,
        joint_angles,
    )
    .end_effector()
}

pub fn joint_positions(
    base_position: Vector2,
    segment_lengths: &[f32],
    joint_angles: &[f32],
) -> Vec<Vector2> {
    forward_kinematics(
        base_position,
        segment_lengths,
        joint_angles,
    )
    .joint_positions
}