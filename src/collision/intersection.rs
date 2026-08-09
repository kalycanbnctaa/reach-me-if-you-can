use crate::{
    collision::{distance, obstacle::Obstacle},
    math::vector2::Vector2,
};

pub fn segment_intersects_obstacle(a: Vector2, b: Vector2, obstacle: &Obstacle) -> bool {
    let (closest, _) = distance::closest_point_on_segment(a, b, obstacle.position);
    let (signed_distance, _) = distance::signed_distance_and_direction(closest, obstacle);

    signed_distance <= 0.0 || obstacle.contains(a) || obstacle.contains(b)
}

pub fn arm_intersects_any(joint_positions: &[Vector2], obstacles: &[Obstacle]) -> bool {
    joint_positions.windows(2).any(|pair| {
        obstacles
            .iter()
            .any(|obstacle| segment_intersects_obstacle(pair[0], pair[1], obstacle))
    })
}