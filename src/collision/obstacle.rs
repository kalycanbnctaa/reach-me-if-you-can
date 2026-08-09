use crate::math::vector2::Vector2;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ObstacleShape {
    Circle { radius: f32 },
    Aabb { half_extents: Vector2 },
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Obstacle {
    pub position: Vector2,
    pub shape: ObstacleShape,
}

impl Obstacle {
    pub fn circle(position: Vector2, radius: f32) -> Self {
        Self {
            position,
            shape: ObstacleShape::Circle {
                radius: radius.max(1.0),
            },
        }
    }

    pub fn aabb(position: Vector2, half_extents: Vector2) -> Self {
        Self {
            position,
            shape: ObstacleShape::Aabb {
                half_extents: Vector2::new(
                    half_extents.x.max(1.0),
                    half_extents.y.max(1.0),
                ),
            },
        }
    }

    pub fn contains(&self, point: Vector2) -> bool {
        match self.shape {
            ObstacleShape::Circle { radius } => {
                Vector2::distance(self.position, point) <= radius
            }
            ObstacleShape::Aabb { half_extents } => {
                let local = point - self.position;
                local.x.abs() <= half_extents.x && local.y.abs() <= half_extents.y
            }
        }
    }
}