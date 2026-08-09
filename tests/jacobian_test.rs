use reach_me_if_you_can::kinematics::{forward, jacobian};
use reach_me_if_you_can::math::vector2::Vector2;

const FINITE_DIFFERENCE_EPS: f32 = 1e-2;
const FINITE_DIFFERENCE_TOLERANCE: f32 = 3e-2;

fn numerical_jacobian(base: Vector2, lengths: &[f32], angles: &[f32]) -> Vec<Vector2> {
    let mut columns = Vec::with_capacity(angles.len());

    for index in 0..angles.len() {
        let mut plus = angles.to_vec();
        plus[index] += FINITE_DIFFERENCE_EPS;

        let mut minus = angles.to_vec();
        minus[index] -= FINITE_DIFFERENCE_EPS;

        let ee_plus = forward::end_effector(base, lengths, &plus);
        let ee_minus = forward::end_effector(base, lengths, &minus);

        let dx = (ee_plus.x - ee_minus.x) / (2.0 * FINITE_DIFFERENCE_EPS);
        let dy = (ee_plus.y - ee_minus.y) / (2.0 * FINITE_DIFFERENCE_EPS);

        columns.push(Vector2::new(dx, dy));
    }

    columns
}

#[test]
fn jacobian_matches_numerical_finite_difference() {
    let base = Vector2::new(0.0, 0.0);
    let lengths = vec![100.0, 80.0, 60.0];
    let angles = vec![0.3, -0.5, 0.8];

    let analytic = jacobian::calculate(base, &lengths, &angles);
    let numeric = numerical_jacobian(base, &lengths, &angles);

    for index in 0..angles.len() {
        let jx = analytic.partial_x(index).unwrap();
        let jy = analytic.partial_y(index).unwrap();

        assert!(
            (jx - numeric[index].x).abs() < FINITE_DIFFERENCE_TOLERANCE,
            "dx mismatch at joint {index}: analytic={jx}, numeric={}",
            numeric[index].x
        );
        assert!(
            (jy - numeric[index].y).abs() < FINITE_DIFFERENCE_TOLERANCE,
            "dy mismatch at joint {index}: analytic={jy}, numeric={}",
            numeric[index].y
        );
    }
}

#[test]
fn jacobian_has_two_rows_and_n_columns() {
    let base = Vector2::new(0.0, 0.0);
    let lengths = vec![50.0, 50.0, 50.0, 50.0];
    let angles = vec![0.0; 4];

    let j = jacobian::calculate(base, &lengths, &angles);

    assert_eq!(j.rows(), 2);
    assert_eq!(j.cols(), 4);
}

#[test]
fn jacobian_last_joint_column_matches_single_segment_derivative() {
    let base = Vector2::new(0.0, 0.0);
    let lengths = vec![100.0, 80.0];
    let angles = vec![0.2, 0.5];

    let j = jacobian::calculate(base, &lengths, &angles);
    let cumulative = angles[0] + angles[1];

    let expected_dx = -lengths[1] * cumulative.sin();
    let expected_dy = lengths[1] * cumulative.cos();

    assert!((j.partial_x(1).unwrap() - expected_dx).abs() < 1e-4);
    assert!((j.partial_y(1).unwrap() - expected_dy).abs() < 1e-4);
}