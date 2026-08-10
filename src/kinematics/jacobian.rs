use nalgebra::DMatrix;

use crate::{
    kinematics::forward,
    math::vector2::Vector2,
};

#[derive(Debug, Clone, PartialEq)]
pub struct Jacobian {
    pub matrix: DMatrix<f32>,
}

impl Jacobian {
    pub fn new(matrix: DMatrix<f32>) -> Self {
        assert_eq!(
            matrix.nrows(),
            2,
            "Jacobian must have exactly 2 rows"
        );

        assert!(
            matrix.iter().all(|value| value.is_finite()),
            "Jacobian values must be finite"
        );

        Self { matrix }
    }

    pub fn zero(joint_count: usize) -> Self {
        Self {
            matrix: DMatrix::zeros(2, joint_count),
        }
    }

    pub fn rows(&self) -> usize {
        self.matrix.nrows()
    }

    pub fn cols(&self) -> usize {
        self.matrix.ncols()
    }

    pub fn joint_count(&self) -> usize {
        self.matrix.ncols()
    }

    pub fn get(&self, row: usize, column: usize) -> Option<f32> {
        if row >= self.rows() || column >= self.cols() {
            return None;
        }
        Some(self.matrix[(row, column)])
    }

    pub fn partial_x(&self, joint_index: usize) -> Option<f32> {
        self.get(0, joint_index)
    }

    pub fn partial_y(&self, joint_index: usize) -> Option<f32> {
        self.get(1, joint_index)
    }

    pub fn is_finite(&self) -> bool {
        self.matrix.iter().all(|value| value.is_finite())
    }

    pub fn as_matrix(&self) -> &DMatrix<f32> {
        &self.matrix
    }
}

fn validate_inputs(
    base_position: Vector2,
    segment_lengths: &[f32],
    joint_angles: &[f32],
) {
    assert!(
        base_position.is_finite(),
        "Base position must be finite"
    );
    assert!(
        !segment_lengths.is_empty(),
        "Robot must contain at least one segment"
    );
    assert_eq!(
        segment_lengths.len(),
        joint_angles.len(),
        "Segment count and joint angle count must match"
    );
    assert!(
        segment_lengths
            .iter()
            .all(|length| length.is_finite() && *length > 0.0),
        "All segment lengths must be finite and greater than zero"
    );
    assert!(
        joint_angles
            .iter()
            .all(|angle| angle.is_finite()),
        "All joint angles must be finite"
    );
}

fn cumulative_angles(joint_angles: &[f32]) -> Vec<f32> {
    let mut result = Vec::with_capacity(joint_angles.len());
    let mut cumulative = 0.0;
    for &angle in joint_angles {
        cumulative += angle;
        result.push(cumulative);
    }
    result
}

pub fn partial_derivative(
    base_position: Vector2,
    segment_lengths: &[f32],
    joint_angles: &[f32],
    joint_index: usize,
) -> Vector2 {
    validate_inputs(base_position, segment_lengths, joint_angles);
    assert!(
        joint_index < joint_angles.len(),
        "Joint index out of bounds"
    );
    let cumulative = cumulative_angles(joint_angles);
    let mut dx = 0.0;
    let mut dy = 0.0;
    for segment_index in joint_index..segment_lengths.len() {
        let length = segment_lengths[segment_index];
        let angle = cumulative[segment_index];
        dx -= length * angle.sin();
        dy += length * angle.cos();
    }
    Vector2::new(dx, dy)
}

pub fn partial_x(
    base_position: Vector2,
    segment_lengths: &[f32],
    joint_angles: &[f32],
    joint_index: usize,
) -> f32 {
    partial_derivative(base_position, segment_lengths, joint_angles, joint_index).x
}

pub fn partial_y(
    base_position: Vector2,
    segment_lengths: &[f32],
    joint_angles: &[f32],
    joint_index: usize,
) -> f32 {
    partial_derivative(base_position, segment_lengths, joint_angles, joint_index).y
}

pub fn calculate(
    base_position: Vector2,
    segment_lengths: &[f32],
    joint_angles: &[f32],
) -> Jacobian {
    validate_inputs(base_position, segment_lengths, joint_angles);
    let joint_count = joint_angles.len();
    let cumulative = cumulative_angles(joint_angles);
    let mut matrix = DMatrix::<f32>::zeros(2, joint_count);
    for joint_index in 0..joint_count {
        let mut dx = 0.0;
        let mut dy = 0.0;
        for segment_index in joint_index..segment_lengths.len() {
            let length = segment_lengths[segment_index];
            let angle = cumulative[segment_index];
            dx -= length * angle.sin();
            dy += length * angle.cos();
        }
        matrix[(0, joint_index)] = dx;
        matrix[(1, joint_index)] = dy;
    }
    Jacobian::new(matrix)
}

pub fn jacobian(
    base_position: Vector2,
    segment_lengths: &[f32],
    joint_angles: &[f32],
) -> Jacobian {
    calculate(base_position, segment_lengths, joint_angles)
}

pub fn point_on_segment(
    base_position: Vector2,
    segment_lengths: &[f32],
    joint_angles: &[f32],
    segment_index: usize,
    fraction: f32,
) -> Vector2 {
    if !fraction.is_finite() || !(0.0..=1.0).contains(&fraction) {
        return base_position;
    }
    if segment_index >= segment_lengths.len() || segment_index >= joint_angles.len() {
        return base_position;
    }
    if !segment_lengths.iter().all(|&l| l.is_finite() && l > 0.0) {
        return base_position;
    }

    let mut lengths: Vec<f32> = segment_lengths[0..=segment_index].to_vec();
    let last = lengths.len() - 1;
    lengths[last] *= fraction;

    if !lengths.iter().all(|&l| l.is_finite() && l > 0.0) {
        return base_position;
    }

    let angles = &joint_angles[0..=segment_index];
    forward::end_effector(base_position, &lengths, angles)
}

pub fn point_jacobian(
    base_position: Vector2,
    segment_lengths: &[f32],
    joint_angles: &[f32],
    segment_index: usize,
    fraction: f32,
    total_joint_count: usize,
) -> Jacobian {
    if !fraction.is_finite() || !(0.0..=1.0).contains(&fraction) {
        return Jacobian::zero(total_joint_count);
    }
    if segment_index >= segment_lengths.len() || segment_index >= joint_angles.len() {
        return Jacobian::zero(total_joint_count);
    }
    if !segment_lengths.iter().all(|&l| l.is_finite() && l > 0.0) {
        return Jacobian::zero(total_joint_count);
    }

    let mut lengths: Vec<f32> = segment_lengths[0..=segment_index].to_vec();
    let last = lengths.len() - 1;
    lengths[last] *= fraction;

    if !lengths.iter().all(|&l| l.is_finite() && l > 0.0) {
        return Jacobian::zero(total_joint_count);
    }

    let angles = &joint_angles[0..=segment_index];
    let partial = calculate(base_position, &lengths, angles);

    let mut matrix = DMatrix::<f32>::zeros(2, total_joint_count);
    for column in 0..partial.cols() {
        matrix[(0, column)] = partial.get(0, column).unwrap_or(0.0);
        matrix[(1, column)] = partial.get(1, column).unwrap_or(0.0);
    }
    Jacobian::new(matrix)
}