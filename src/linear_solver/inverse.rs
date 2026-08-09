use nalgebra::DMatrix;

use crate::linear_solver::gaussian;

pub fn inverse(a: &DMatrix<f32>) -> Option<DMatrix<f32>> {
    assert_eq!(a.nrows(), a.ncols());

    let identity = DMatrix::<f32>::identity(a.nrows(), a.nrows());

    gaussian::solve(a, &identity)
}