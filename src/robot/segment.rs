#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Segment {
    pub length: f32,
}

impl Segment {
    pub fn new(length: f32) -> Self {
        assert!(
            length.is_finite() && length > 0.0,
            "Segment length must be finite and greater than zero"
        );

        Self { length }
    }
}