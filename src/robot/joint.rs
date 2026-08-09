#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Joint {
    pub id: usize,
}

impl Joint {
    pub fn new(id: usize) -> Self {
        Self { id }
    }
}