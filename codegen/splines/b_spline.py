import symforce.symbolic as sf

from codegen.splines import utils


def uniform_b_spline_matrix_coefficients(n: int) -> (sf.M, sf.M):
    an = sf.M.zeros(n, n - 1)
    bn = sf.M.zeros(n, n - 1)

    for i in range(n - 1):
        c = sf.Rational(1, n - 1)
        an[i, i] = (i + 1) * c
        an[i + 1, i] = (n - 2 - i) * c
        bn[i, i] = -c
        bn[i + 1, i] = c

    return an, bn


def uniform_b_spline_matrix(n: int) -> sf.M:
    if n == 1:
        return sf.M11.ones(1, 1)
    elif n == 2:
        m = sf.M22()
        m[0, 0] = 1
        m[1, 0] = 0
        m[0, 1] = -1
        m[1, 1] = 1
        return m
    else:
        an, bn = uniform_b_spline_matrix_coefficients(n)
        mn = uniform_b_spline_matrix(n - 1)
        ln = sf.M.zeros(n, n)
        rn = sf.M.zeros(n, n)
        ln[:n, :n - 1] = an * mn
        rn[:n, 1:] = bn * mn
        return ln + rn


def cumulative_uniform_b_spline_matrix(n: int, compact: bool = True) -> sf.M:
    assert n > 1
    mn = sf.M.ones(n, n).lower_triangle().transpose() * uniform_b_spline_matrix(n)
    return mn[1:, :] if compact else mn


def uniform_b_spline_lambdas(ut: sf.Scalar, n: int, k: int) -> sf.M:
    return cumulative_uniform_b_spline_matrix(n, False) * utils.diff_k_normalized_vectors_n(ut, n, k)


def cumulative_uniform_b_spline_lambdas(ut: sf.Scalar, n: int, k: int) -> sf.M:
    return cumulative_uniform_b_spline_matrix(n, True) * utils.diff_k_normalized_vectors_n(ut, n, k)
