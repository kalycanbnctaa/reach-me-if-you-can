use nalgebra::DMatrix;

use crate::linear_solver::inverse;

pub fn right_pseudo_inverse(j: &DMatrix<f32>) -> Option<DMatrix<f32>> {
    assert!(j.nrows() <= j.ncols());

    let jjt = j.clone() * j.transpose();
    let jjt_inv = inverse::inverse(&jjt)?;
    let pseudo = j.transpose() * jjt_inv;

    if pseudo.iter().all(|value| value.is_finite()) {
        Some(pseudo)
    } else {
        None
    }
}

pub fn damped_right_pseudo_inverse(
    j: &DMatrix<f32>,
    lambda: f32,
) -> Option<DMatrix<f32>> {
    assert!(j.nrows() <= j.ncols());

    let rows = j.nrows();
    let jjt = j.clone() * j.transpose();
    let damped = jjt + DMatrix::<f32>::identity(rows, rows) * (lambda * lambda);

    let damped_inv = inverse::inverse(&damped)?;
    let pseudo = j.transpose() * damped_inv;

    if pseudo.iter().all(|value| value.is_finite()) {
        Some(pseudo)
    } else {
        None
    }
}