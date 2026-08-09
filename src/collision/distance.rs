use crate::{
    collision::obstacle::{Obstacle, ObstacleShape},
    math::vector2::Vector2,
};

pub fn closest_point_on_segment(a: Vector2, b: Vector2, p: Vector2) -> (Vector2, f32) {
    let ab = b - a;
    let length_squared = ab.length_squared();

    if length_squared <= f32::EPSILON {
        return (a, 0.0);
    }

    let t = (Vector2::dot(p - a, ab) / length_squared).clamp(0.0, 1.0);

    (a + ab * t, t)
}

pub fn closest_point_on_aabb(point: Vector2, center: Vector2, half_extents: Vector2) -> Vector2 {
    let local = point - center;

    let clamped = Vector2::new(
        local.x.clamp(-half_extents.x, half_extents.x),
        local.y.clamp(-half_extents.y, half_extents.y),
    );

    center + clamped
}

pub fn signed_distance_and_direction(point: Vector2, obstacle: &Obstacle) -> (f32, Vector2) {
    match obstacle.shape {
        ObstacleShape::Circle { radius } => {
            let offset = point - obstacle.position;
            let distance = offset.length();
            let direction = offset.try_normalized().unwrap_or(Vector2::X_AXIS);

            (distance - radius, direction)
        }
        ObstacleShape::Aabb { half_extents } => {
            let closest = closest_point_on_aabb(point, obstacle.position, half_extents);
            let offset = point - closest;
            let outside_distance = offset.length();

            if outside_distance > f32::EPSILON {
                (outside_distance, offset / outside_distance)
            } else {
                let local = point - obstacle.position;
                let dx = half_extents.x - local.x.abs();
                let dy = half_extents.y - local.y.abs();

                if dx < dy {
                    (-dx, Vector2::new(local.x.signum(), 0.0))
                } else {
                    (-dy, Vector2::new(0.0, local.y.signum()))
                }
            }
        }
    }
}