import symforce.symbolic as sf
from symforce import typing as T


def as_symmetric_matrix(mat: T.Type[sf.Matrix.MatrixT]) -> sf.Matrix.MatrixT:
    """
    Reinterprets a matrix as symmetric matrix (using the lower triangular entries).
    :param mat: Symbolic matrix (assumed to be symmetric).
    :return: Symmetric interpretation of the matrix.
    """
    assert mat.cols == mat.rows
    tmp = mat.lower_triangle() + mat.lower_triangle().transpose()
    for i in range(mat.cols):
        tmp[i, i] -= mat[i, i]
    return tmp


def remove_less_than_epsilon_entries(mat: T.Type[sf.Matrix.MatrixT], eps: sf.Scalar) -> sf.Matrix.MatrixT:
    """
    Removes all entries below epsilon and replaces them with zeros.
    :param mat: Input matrix.
    :param eps: Epsilon tolerance.
    :return: Matrix without entries below epsilon.
    """
    for i in range(mat.shape[0]):
        for j in range(mat.shape[1]):
            mat[i, j] = mat[i, j] if abs(mat[i, j]) > eps else 0
    return mat


def hat_rot2(v: sf.V1) -> sf.M22:
    """
    Hat operator for Rot2.
    :param v: Input twist vector.
    :return: Skew-symmetric matrix.
    """
    return sf.M22([[0, -v], [v, 0]])


def vee_rot2(x: sf.M22) -> sf.V1:
    """
    Vee operator for Rot2.
    :param x: Skew-symmetric matrix.
    :return: Twist vector.
    """
    return sf.V1(x[1, 0])


def hat_rot3(v: sf.V3) -> sf.M33:
    """
    Hat operator for Rot3.
    :param v: Input twist vector.
    :return: Skew-symmetric matrix.
    """
    return sf.M33([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def vee_rot3(x: sf.M33) -> sf.V3:
    """
    Vee operator for Rot3.
    :param x: Skew-symmetric matrix.
    :return: Twist vector.
    """
    return sf.V3(x[2, 1], x[0, 2], x[1, 0])


def hat_pose2(v: sf.V3) -> sf.M33:
    """
    Hat operator for Pose2.
    :param v: Input twist vector.
    :return: Skew-symmetric matrix.
    """
    hat = sf.M33()
    hat[:2, :2] = hat_rot2(v[0])
    hat[:2, 2] = v[1:]
    return hat


def vee_pose2(x: sf.M33) -> sf.V3:
    """
    Vee operator for Pose2.
    :param x: Skew-symmetric matrix.
    :return: Twist vector.
    """
    return sf.V3(x[1, 0], x[0, 2], x[1, 2])


def hat_pose3(v: sf.V6) -> sf.M44:
    """
    Hat operator for Pose3.
    :param v: Input twist vector.
    :return: Skew-symmetric matrix.
    """
    hat = sf.M44()
    hat[:3, :3] = hat_rot3(v[:3])
    hat[:3, 3] = v[3:]
    return hat


def vee_pose3(x: sf.M44) -> sf.V6:
    """
    Vee operator for Pose3.
    :param x: Skew-symmetric matrix.
    :return: Twist vector.
    """
    vee = sf.V6()
    vee[:3] = vee_rot3(x[:3, :3])
    vee[3:] = x[:3, 3]
    return vee


def adj_rot2(g: sf.Rot2) -> sf.M11:
    """
    Adjoint operator for Rot2 (i.e. adj_g * v = vee(G * hat(v) inv(G))).
    :param g: Rot2 element.
    """
    return sf.M11.eye()


def adj_rot3(g: sf.Rot3) -> sf.M33:
    """
    Adjoint operator for Rot3 (i.e. adj_g * v = vee(G * hat(v) inv(G))).
    :param g: Rot3 element.
    """
    return g.to_rotation_matrix()


def adj_pose2(g: sf.Pose2) -> sf.M33:
    """
    Adjoint operator for Pose2 (i.e. adj_g * v = vee(G * hat(v) inv(G))).
    :param g: Pose2 element.
    """
    adj = sf.M33()
    rot = g.R.to_rotation_matrix()
    adj[0, 0] = 1
    adj[1, 0] = g.t[1]
    adj[2, 0] = -g.t[0]
    adj[1:, 1:] = rot
    return adj


def adj_pose3(g: sf.Pose3) -> sf.M66:
    """
    Adjoint operator for Pose3 (i.e. adj_g * v = vee(G * hat(v) inv(G))).
    :param g: Pose3 element.
    """
    adj = sf.M66()
    rot = g.R.to_rotation_matrix()
    adj[:3, :3] = rot
    adj[3:, :3] = hat_rot3(g.t) * rot
    adj[3:, 3:] = rot
    return adj


def lie_bracket_rot2(x: sf.V1, y: sf.V1) -> sf.V1:
    """
    Lie bracket operator for Rot2 (i.e. [x, y]_rot2 = vee(hat(x) * hat(y) - hat(y) * hat(x))).
    :param x: Element in the tangent space of Rot2.
    :param y: Element in the tangent space of Rot2.
    """
    return sf.V1.zero()


def lie_bracket_rot3(x: sf.V3, y: sf.V3) -> sf.V3:
    """
    Lie bracket operator for Rot3 (i.e. [x, y]_rot3 = vee(hat(x) * hat(y) - hat(y) * hat(x))).
    :param x: Element in the tangent space of Rot3.
    :param y: Element in the tangent space of Rot3.
    """
    return x.cross(y)


def lie_bracket_pose2(x: sf.V3, y: sf.V3) -> sf.V3:
    """
    Lie bracket operator for Pose2 (i.e. [x, y]_pose2 = vee(hat(x) * hat(y) - hat(y) * hat(x))).
    :param x: Element in the tangent space of Pose2.
    :param y: Element in the tangent space of Pose2.
    """
    a = hat_pose2(x)
    b = hat_pose2(y)
    return vee_pose2(a * b - b * a)


def lie_bracket_pose3(x: sf.V6, y: sf.V6) -> sf.V6:
    """
    Lie bracket operator for Pose3 (i.e. [x, y]_pose3 = vee(hat(x) * hat(y) - hat(y) * hat(x))).
    :param x: Element in the tangent space of Pose3.
    :param y: Element in the tangent space of Pose3.
    """
    a = hat_pose3(x)
    b = hat_pose3(y)
    return vee_pose3(a * b - b * a)


def normalized_time(t: sf.Scalar, t0: sf.Scalar, t1: sf.Scalar) -> sf.Scalar:
    """
    Normalizes a query time.
    :param t: Time.
    :param t0: Lower normalization boundary.
    :param t1: Upper normalization boundary.
    :return: Normalized time (i.e. ut).
    """
    return (t - t0) / (t1 - t0)


def normalized_time_vector_n(ut: sf.Scalar, n: int) -> sf.M:
    """
    Evaluates the (stacked) vector of normalized times.
    :param ut: Normalized time.
    :param n: Order of the vector (n x 1).
    :return: (Stacked) Vector of normalized times.
    """
    assert n > 0
    vec = sf.M(n, 1)
    for i in range(n):
        vec[i] = ut ** i
    return vec


def diff_k_normalized_vectors_n(ut: sf.Scalar, n: int, k: int) -> sf.M:
    """
    Evaluates the differentiated (stacked) vectors of normalized times.
    :param ut: Normalized time.
    :param n: Order of the vector (n x 1).
    :param k: Order of the derivative.
    :return: (Stacked) Vectors of normalized times (n x k+1).
    """
    assert k >= 0
    mat = sf.M(n, k + 1)
    mat[:, 0] = normalized_time_vector_n(ut, n)
    for i in range(k):
        mat[:, i + 1] = mat[:, i].diff(ut)
    return mat
