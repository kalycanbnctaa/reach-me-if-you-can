use std::ops::{
    Add,
    AddAssign,
    Div,
    DivAssign,
    Mul,
    MulAssign,
    Neg,
    Sub,
    SubAssign,
};

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vector2 {
    pub x: f32,
    pub y: f32,
}

impl Vector2 {
    pub const ZERO: Self = Self {
        x: 0.0,
        y: 0.0,
    };

    pub const ONE: Self = Self {
        x: 1.0,
        y: 1.0,
    };

    pub const X_AXIS: Self = Self {
        x: 1.0,
        y: 0.0,
    };

    pub const Y_AXIS: Self = Self {
        x: 0.0,
        y: 1.0,
    };

    pub const fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    pub fn length_squared(self) -> f32 {
        self.x * self.x + self.y * self.y
    }

    pub fn length(self) -> f32 {
        self.length_squared().sqrt()
    }

    pub fn normalized(self) -> Self {
        let length = self.length();

        if !length.is_finite() || length <= f32::EPSILON {
            Self::ZERO
        } else {
            self / length
        }
    }

    pub fn try_normalized(self) -> Option<Self> {
        let length = self.length();

        if !length.is_finite() || length <= f32::EPSILON {
            None
        } else {
            Some(self / length)
        }
    }

    pub fn distance(a: Self, b: Self) -> f32 {
        (a - b).length()
    }

    pub fn distance_squared(a: Self, b: Self) -> f32 {
        (a - b).length_squared()
    }

    pub fn dot(a: Self, b: Self) -> f32 {
        a.x * b.x + a.y * b.y
    }

    pub fn cross(a: Self, b: Self) -> f32 {
        a.x * b.y - a.y * b.x
    }

    pub fn perp(self) -> Self {
        Self::new(-self.y, self.x)
    }

    pub fn angle(self) -> f32 {
        self.y.atan2(self.x)
    }

    pub fn from_angle(angle: f32) -> Self {
        Self::new(angle.cos(), angle.sin())
    }

    pub fn lerp(a: Self, b: Self, t: f32) -> Self {
        a + (b - a) * t
    }

    pub fn component_min(a: Self, b: Self) -> Self {
        Self::new(a.x.min(b.x), a.y.min(b.y))
    }

    pub fn component_max(a: Self, b: Self) -> Self {
        Self::new(a.x.max(b.x), a.y.max(b.y))
    }

    pub fn abs(self) -> Self {
        Self::new(self.x.abs(), self.y.abs())
    }

    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite()
    }

    pub fn is_near_zero(self, epsilon: f32) -> bool {
        self.length_squared() <= epsilon * epsilon
    }

    pub fn clamp_length(self, max_length: f32) -> Self {
        if !max_length.is_finite() || max_length <= 0.0 {
            return Self::ZERO;
        }

        let length = self.length();

        if !length.is_finite() || length <= max_length || length <= f32::EPSILON {
            self
        } else {
            self * (max_length / length)
        }
    }

    pub fn project_onto(self, direction: Self) -> Self {
        let denominator = direction.length_squared();

        if !denominator.is_finite() || denominator <= f32::EPSILON {
            Self::ZERO
        } else {
            direction * (Self::dot(self, direction) / denominator)
        }
    }
}

impl Add for Vector2 {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl AddAssign for Vector2 {
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

impl Sub for Vector2 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl SubAssign for Vector2 {
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
    }
}

impl Mul<f32> for Vector2 {
    type Output = Self;

    fn mul(self, rhs: f32) -> Self::Output {
        Self::new(self.x * rhs, self.y * rhs)
    }
}

impl MulAssign<f32> for Vector2 {
    fn mul_assign(&mut self, rhs: f32) {
        self.x *= rhs;
        self.y *= rhs;
    }
}

impl Div<f32> for Vector2 {
    type Output = Self;

    fn div(self, rhs: f32) -> Self::Output {
        Self::new(self.x / rhs, self.y / rhs)
    }
}

impl DivAssign<f32> for Vector2 {
    fn div_assign(&mut self, rhs: f32) {
        self.x /= rhs;
        self.y /= rhs;
    }
}

impl Neg for Vector2 {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self::new(-self.x, -self.y)
    }
}