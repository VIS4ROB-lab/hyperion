from codegen.factors.delta import *


def _rn_euclidean_distance_factor(x, x_d_y: sf.V1, y, sqrt_info: sf.M11, epsilon: sf.Scalar) -> sf.V1:
    est_x_d_y = sf.V1((y - x).norm(epsilon))
    return r1_delta_factor(x_d_y, est_x_d_y, sqrt_info, epsilon)


def _lie_angular_distance_factor(x, x_d_y: sf.V1, y, sqrt_info: sf.M11, epsilon: sf.Scalar) -> sf.V1:
    est_x_d_y = sf.V1(sf.Matrix(x.local_coordinates(y, epsilon)).norm(epsilon))
    return r1_delta_factor(x_d_y, est_x_d_y, sqrt_info, epsilon)


def r1_euclidean_distance_factor(x: sf.V1, x_d_y: sf.V1, y: sf.V1, sqrt_info: sf.M11, epsilon: sf.Scalar) -> sf.V1:
    return _rn_euclidean_distance_factor(x, x_d_y, y, sqrt_info, epsilon)


def r2_euclidean_distance_factor(x: sf.V2, x_d_y: sf.V1, y: sf.V2, sqrt_info: sf.M11, epsilon: sf.Scalar) -> sf.V1:
    return _rn_euclidean_distance_factor(x, x_d_y, y, sqrt_info, epsilon)


def r3_euclidean_distance_factor(x: sf.V3, x_d_y: sf.V1, y: sf.V3, sqrt_info: sf.M11, epsilon: sf.Scalar) -> sf.V1:
    return _rn_euclidean_distance_factor(x, x_d_y, y, sqrt_info, epsilon)


def r2_angular_distance_factor(x: sf.V2, x_d_y: sf.V1, y: sf.V2, sqrt_info: sf.M11, epsilon: sf.Scalar) -> sf.V1:
    est_x_d_y = r2_angular_delta(x, y, epsilon)
    return r1_delta_factor(x_d_y, est_x_d_y, sqrt_info, epsilon)


def r3_angular_distance_factor(x: sf.V3, x_d_y: sf.V1, y: sf.V3, sqrt_info: sf.M11, epsilon: sf.Scalar) -> sf.V1:
    est_x_d_y = r3_angular_delta(x, y, epsilon)
    return r1_delta_factor(x_d_y, est_x_d_y, sqrt_info, epsilon)


def rot2_angular_distance_factor(x: sf.Rot2, x_d_y: sf.V1, y: sf.Rot2, sqrt_info: sf.M11, epsilon: sf.Scalar) -> sf.V1:
    return _lie_angular_distance_factor(x, x_d_y, y, sqrt_info, epsilon)


def rot3_angular_distance_factor(x: sf.Rot3, x_d_y: sf.V1, y: sf.Rot3, sqrt_info: sf.M11, epsilon: sf.Scalar) -> sf.V1:
    return _lie_angular_distance_factor(x, x_d_y, y, sqrt_info, epsilon)
