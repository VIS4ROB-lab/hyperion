import symforce.symbolic as sf

from math import factorial

from codegen.splines import utils


def uniform_z_spline_matrix(n: int) -> sf.M:
    if n == 2:
        return sf.M22(1, -1,
                      0, 1)
    elif n == 4:
        return sf.M44(0, -1, 2, -1,
                      2, 0, -5, 3,
                      0, 1, 4, -3,
                      0, 0, -1, 1) / factorial(2)
    elif n == 6:
        return sf.M66(0, 0, 0, 7, -12, 5,
                      0, -2, -1, -33, 61, -25,
                      0, 16, 16, 66, -124, 50,
                      24, 0, -30, -70, 126, -50,
                      0, -16, 16, 39, -64, 25,
                      0, 2, -1, -9, 13, -5) / factorial(4)
    else:
        raise AssertionError(f"Matrix order {n} either odd or not implemented")


def cumulative_uniform_z_spline_matrix(n: int, compact: bool = True) -> sf.M:
    assert n > 1
    mn = sf.M.ones(n, n).lower_triangle().transpose() * uniform_z_spline_matrix(n)
    return mn[1:, :] if compact else mn


def uniform_z_spline_lambdas(ut: sf.Scalar, n: int, k: int) -> sf.M:
    return cumulative_uniform_z_spline_matrix(n, False) * utils.diff_k_normalized_vectors_n(ut, n, k)


def cumulative_uniform_z_spline_lambdas(ut: sf.Scalar, n: int, k: int) -> sf.M:
    return cumulative_uniform_z_spline_matrix(n, True) * utils.diff_k_normalized_vectors_n(ut, n, k)
