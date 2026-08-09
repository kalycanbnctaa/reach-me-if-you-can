use crate::math::vector2::Vector2;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Matrix3 {
    pub data: [[f32; 3]; 3],
}

impl Matrix3 {
    pub const ZERO: Self = Self {
        data: [
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        ],
    };

    pub const IDENTITY: Self = Self {
        data: [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
    };

    pub const fn new(data: [[f32; 3]; 3]) -> Self {
        Self { data }
    }

    pub const fn zero() -> Self {
        Self::ZERO
    }

    pub const fn identity() -> Self {
        Self::IDENTITY
    }

    pub fn rotation(angle: f32) -> Self {
        let cos = angle.cos();
        let sin = angle.sin();

        Self::new([
            [cos, -sin, 0.0],
            [sin, cos, 0.0],
            [0.0, 0.0, 1.0],
        ])
    }

    pub fn translation(x: f32, y: f32) -> Self {
        Self::new([
            [1.0, 0.0, x],
            [0.0, 1.0, y],
            [0.0, 0.0, 1.0],
        ])
    }

    pub fn from_translation(position: Vector2) -> Self {
        Self::translation(position.x, position.y)
    }

    pub fn from_rotation_translation(
        angle: f32,
        translation: Vector2,
    ) -> Self {
        let cos = angle.cos();
        let sin = angle.sin();

        Self::new([
            [cos, -sin, translation.x],
            [sin, cos, translation.y],
            [0.0, 0.0, 1.0],
        ])
    }

    pub fn multiply(self, rhs: Self) -> Self {
        let mut result = [[0.0; 3]; 3];

        for row in 0..3 {
            for column in 0..3 {
                for k in 0..3 {
                    result[row][column] +=
                        self.data[row][k] * rhs.data[k][column];
                }
            }
        }

        Self::new(result)
    }

    pub fn transform_point(self, point: Vector2) -> Vector2 {
        let x = self.data[0][0] * point.x
            + self.data[0][1] * point.y
            + self.data[0][2];

        let y = self.data[1][0] * point.x
            + self.data[1][1] * point.y
            + self.data[1][2];

        let w = self.data[2][0] * point.x
            + self.data[2][1] * point.y
            + self.data[2][2];

        if w.abs() <= f32::EPSILON {
            Vector2::new(x, y)
        } else {
            Vector2::new(x / w, y / w)
        }
    }

    pub fn transform_vector(self, vector: Vector2) -> Vector2 {
        Vector2::new(
            self.data[0][0] * vector.x
                + self.data[0][1] * vector.y,
            self.data[1][0] * vector.x
                + self.data[1][1] * vector.y,
        )
    }

    pub fn transpose(self) -> Self {
        let mut result = [[0.0; 3]; 3];

        for row in 0..3 {
            for column in 0..3 {
                result[row][column] = self.data[column][row];
            }
        }

        Self::new(result)
    }

    pub fn determinant(self) -> f32 {
        let a = self.data[0][0];
        let b = self.data[0][1];
        let c = self.data[0][2];

        let d = self.data[1][0];
        let e = self.data[1][1];
        let f = self.data[1][2];

        let g = self.data[2][0];
        let h = self.data[2][1];
        let i = self.data[2][2];

        a * (e * i - f * h)
            - b * (d * i - f * g)
            + c * (d * h - e * g)
    }

    pub fn inverse(self) -> Option<Self> {
        let determinant = self.determinant();

        if !determinant.is_finite() || determinant.abs() <= f32::EPSILON {
            return None;
        }

        let a = self.data[0][0];
        let b = self.data[0][1];
        let c = self.data[0][2];

        let d = self.data[1][0];
        let e = self.data[1][1];
        let f = self.data[1][2];

        let g = self.data[2][0];
        let h = self.data[2][1];
        let i = self.data[2][2];

        let inverse_determinant = 1.0 / determinant;

        let result = [
            [
                (e * i - f * h) * inverse_determinant,
                (c * h - b * i) * inverse_determinant,
                (b * f - c * e) * inverse_determinant,
            ],
            [
                (f * g - d * i) * inverse_determinant,
                (a * i - c * g) * inverse_determinant,
                (c * d - a * f) * inverse_determinant,
            ],
            [
                (d * h - e * g) * inverse_determinant,
                (b * g - a * h) * inverse_determinant,
                (a * e - b * d) * inverse_determinant,
            ],
        ];

        let inverse = Self::new(result);

        if inverse.is_finite() {
            Some(inverse)
        } else {
            None
        }
    }

    pub fn translation_component(self) -> Vector2 {
        Vector2::new(
            self.data[0][2],
            self.data[1][2],
        )
    }

    pub fn rotation_angle(self) -> f32 {
        self.data[1][0].atan2(self.data[0][0])
    }

    pub fn is_finite(self) -> bool {
        self.data
            .iter()
            .flatten()
            .all(|value| value.is_finite())
    }
}

impl Default for Matrix3 {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl std::ops::Mul for Matrix3 {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        self.multiply(rhs)
    }
}