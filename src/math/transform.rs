use crate::math::{
    matrix3::Matrix3,
    vector2::Vector2,
};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Transform {
    pub matrix: Matrix3,
}

impl Transform {
    pub const IDENTITY: Self = Self {
        matrix: Matrix3::IDENTITY,
    };

    pub const fn identity() -> Self {
        Self::IDENTITY
    }

    pub fn new(matrix: Matrix3) -> Self {
        Self { matrix }
    }

    pub fn rotation(angle: f32) -> Self {
        Self::new(Matrix3::rotation(angle))
    }

    pub fn translation(position: Vector2) -> Self {
        Self::new(Matrix3::from_translation(position))
    }

    pub fn from_rotation_translation(
        angle: f32,
        position: Vector2,
    ) -> Self {
        Self::new(
            Matrix3::from_rotation_translation(angle, position),
        )
    }

    pub fn position(self) -> Vector2 {
        self.matrix.translation_component()
    }

    pub fn rotation_angle(self) -> f32 {
        self.matrix.rotation_angle()
    }

    pub fn transform_point(self, point: Vector2) -> Vector2 {
        self.matrix.transform_point(point)
    }

    pub fn transform_vector(self, vector: Vector2) -> Vector2 {
        self.matrix.transform_vector(vector)
    }

    pub fn then(self, next: Self) -> Self {
        Self::new(self.matrix * next.matrix)
    }

    pub fn precompose(self, previous: Self) -> Self {
        Self::new(previous.matrix * self.matrix)
    }

    pub fn inverse(self) -> Option<Self> {
        self.matrix.inverse().map(Self::new)
    }

    pub fn is_finite(self) -> bool {
        self.matrix.is_finite()
    }
}

impl Default for Transform {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl std::ops::Mul for Transform {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        self.then(rhs)
    }
}