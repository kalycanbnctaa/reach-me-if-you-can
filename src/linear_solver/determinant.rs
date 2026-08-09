use nalgebra::DMatrix;

const RELATIVE_EPSILON: f32 = 1.0e-5;

pub fn determinant(a: &DMatrix<f32>) -> f32 {
    assert_eq!(a.nrows(), a.ncols());

    let n = a.nrows();
    let mut m = a.clone();
    let mut sign = 1.0_f32;

    let scale = a.iter().fold(0.0f32, |acc, value| acc.max(value.abs())).max(1.0);
    let threshold = scale * RELATIVE_EPSILON;

    for pivot in 0..n {
        let mut max_row = pivot;
        let mut max_value = m[(pivot, pivot)].abs();

        for row in (pivot + 1)..n {
            let value = m[(row, pivot)].abs();

            if value > max_value {
                max_value = value;
                max_row = row;
            }
        }

        if !max_value.is_finite() || max_value <= threshold {
            return 0.0;
        }

        if max_row != pivot {
            m.swap_rows(pivot, max_row);
            sign = -sign;
        }

        let pivot_value = m[(pivot, pivot)];

        for row in (pivot + 1)..n {
            let factor = m[(row, pivot)] / pivot_value;

            if factor == 0.0 {
                continue;
            }

            for col in pivot..n {
                m[(row, col)] -= factor * m[(pivot, col)];
            }
        }
    }

    let mut det = sign;

    for i in 0..n {
        det *= m[(i, i)];
    }

    if det.is_finite() { det } else { 0.0 }
}