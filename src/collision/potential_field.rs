use crate::{
    collision::{distance, obstacle::Obstacle},
    kinematics::jacobian,
    math::vector2::Vector2,
    robot::arm::RobotArm,
};

#[derive(Debug, Clone, Copy)]
pub struct PotentialFieldConfig {
    pub influence_radius: f32,
    pub repulsive_gain: f32,
    pub samples_per_segment: usize,
}

impl Default for PotentialFieldConfig {
    fn default() -> Self {
        Self {
            influence_radius: 60.0,
            repulsive_gain: 4500.0,
            samples_per_segment: 4,
        }
    }
}

pub fn compute_delta(
    robot: &RobotArm,
    obstacles: &[Obstacle],
    config: &PotentialFieldConfig,
) -> Vec<f32> {
    let joint_count = robot.actuated_joint_count();
    let mut delta = vec![0.0_f32; joint_count];

    if obstacles.is_empty() {
        return delta;
    }

    let lengths: Vec<f32> = robot.segments.iter().map(|segment| segment.length).collect();
    let angles = &robot.state.joint_angles;

    for segment_index in 0..joint_count {
        for sample in 0..=config.samples_per_segment {
            let fraction = sample as f32 / config.samples_per_segment as f32;

            let point = jacobian::point_on_segment(
                robot.base_position,
                &lengths,
                angles,
                segment_index,
                fraction,
            );

            for obstacle in obstacles {
                let (signed_distance, direction) =
                    distance::signed_distance_and_direction(point, obstacle);

                if !signed_distance.is_finite() || signed_distance >= config.influence_radius {
                    continue;
                }

                let clamped_distance = signed_distance.max(1.0e-3);

                let magnitude = config.repulsive_gain
                    * (1.0 / clamped_distance - 1.0 / config.influence_radius)
                    * (1.0 / (clamped_distance * clamped_distance));

                if magnitude <= 0.0 {
                    continue;
                }

                let force = direction * magnitude;

                let point_jacobian = jacobian::point_jacobian(
                    robot.base_position,
                    &lengths,
                    angles,
                    segment_index,
                    fraction,
                    joint_count,
                );

                for joint_index in 0..joint_count {
                    let jx = point_jacobian.partial_x(joint_index).unwrap_or(0.0);
                    let jy = point_jacobian.partial_y(joint_index).unwrap_or(0.0);

                    delta[joint_index] += jx * force.x + jy * force.y;
                }
            }
        }
    }

    delta
}

pub fn compute_force_vectors(
    robot: &RobotArm,
    obstacles: &[Obstacle],
    config: &PotentialFieldConfig,
) -> Vec<(Vector2, Vector2)> {
    let mut vectors = Vec::new();

    if obstacles.is_empty() {
        return vectors;
    }

    let joint_count = robot.actuated_joint_count();
    let lengths: Vec<f32> = robot.segments.iter().map(|segment| segment.length).collect();
    let angles = &robot.state.joint_angles;

    for segment_index in 0..joint_count {
        for sample in 0..=config.samples_per_segment {
            let fraction = sample as f32 / config.samples_per_segment as f32;

            let point = jacobian::point_on_segment(
                robot.base_position,
                &lengths,
                angles,
                segment_index,
                fraction,
            );

            let mut total_force = Vector2::ZERO;

            for obstacle in obstacles {
                let (signed_distance, direction) =
                    distance::signed_distance_and_direction(point, obstacle);

                if !signed_distance.is_finite() || signed_distance >= config.influence_radius {
                    continue;
                }

                let clamped_distance = signed_distance.max(1.0e-3);

                let magnitude = config.repulsive_gain
                    * (1.0 / clamped_distance - 1.0 / config.influence_radius)
                    * (1.0 / (clamped_distance * clamped_distance));

                if magnitude <= 0.0 {
                    continue;
                }

                total_force += direction * magnitude;
            }

            if total_force.length() > 0.5 {
                vectors.push((point, total_force));
            }
        }
    }

    vectors
}