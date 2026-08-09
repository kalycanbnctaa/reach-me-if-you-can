use nalgebra::DMatrix;

const RELATIVE_EPSILON: f32 = 1.0e-5;

pub fn solve(a: &DMatrix<f32>, b: &DMatrix<f32>) -> Option<DMatrix<f32>> {
    assert_eq!(a.nrows(), a.ncols());
    assert_eq!(a.nrows(), b.nrows());

    let n = a.nrows();
    let columns = b.ncols();

    let scale = a.iter().fold(0.0f32, |acc, value| acc.max(value.abs())).max(1.0);
    let threshold = scale * RELATIVE_EPSILON;

    let mut aug = DMatrix::<f32>::zeros(n, n + columns);

    for row in 0..n {
        for col in 0..n {
            aug[(row, col)] = a[(row, col)];
        }
        for col in 0..columns {
            aug[(row, n + col)] = b[(row, col)];
        }
    }

    forward_eliminate(&mut aug, n, threshold)?;
    back_substitute(&aug, n, columns, threshold)
}

fn forward_eliminate(aug: &mut DMatrix<f32>, n: usize, threshold: f32) -> Option<()> {
    for pivot in 0..n {
        let mut max_row = pivot;
        let mut max_value = aug[(pivot, pivot)].abs();

        for row in (pivot + 1)..n {
            let value = aug[(row, pivot)].abs();

            if value > max_value {
                max_value = value;
                max_row = row;
            }
        }

        if !max_value.is_finite() || max_value <= threshold {
            return None;
        }

        if max_row != pivot {
            aug.swap_rows(pivot, max_row);
        }

        let pivot_value = aug[(pivot, pivot)];

        for row in (pivot + 1)..n {
            let factor = aug[(row, pivot)] / pivot_value;

            if factor == 0.0 {
                continue;
            }

            for col in pivot..aug.ncols() {
                aug[(row, col)] -= factor * aug[(pivot, col)];
            }
        }
    }

    Some(())
}

fn back_substitute(
    aug: &DMatrix<f32>,
    n: usize,
    columns: usize,
    threshold: f32,
) -> Option<DMatrix<f32>> {
    let mut result = DMatrix::<f32>::zeros(n, columns);

    for row in (0..n).rev() {
        for col in 0..columns {
            let mut sum = aug[(row, n + col)];

            for k in (row + 1)..n {
                sum -= aug[(row, k)] * result[(k, col)];
            }

            let pivot = aug[(row, row)];

            if !pivot.is_finite() || pivot.abs() <= threshold {
                return None;
            }

            result[(row, col)] = sum / pivot;
        }
    }

    if result.iter().all(|value| value.is_finite()) {
        Some(result)
    } else {
        None
    }
}