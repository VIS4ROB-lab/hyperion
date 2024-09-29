import symforce.symbolic as sf


def _rn_delta_factor(x, y, sqrt_info, epsilon: sf.Scalar) -> sf.Matrix:
    return sqrt_info * (y - x)


def _lie_delta_factor(x, y, sqrt_info, epsilon: sf.Scalar) -> sf.Matrix:
    return sqrt_info * sf.Matrix(x.local_coordinates(y, epsilon))


def r1_delta_factor(x: sf.V1, y: sf.V1, sqrt_info: sf.M11, epsilon: sf.Scalar) -> sf.V1:
    return sf.V1(_rn_delta_factor(x, y, sqrt_info, epsilon))


def r2_delta_factor(x: sf.V2, y: sf.V2, sqrt_info: sf.M22, epsilon: sf.Scalar) -> sf.V2:
    return sf.V2(_rn_delta_factor(x, y, sqrt_info, epsilon))


def r3_delta_factor(x: sf.V3, y: sf.V3, sqrt_info: sf.M33, epsilon: sf.Scalar) -> sf.V3:
    return sf.V3(_rn_delta_factor(x, y, sqrt_info, epsilon))


def r6_delta_factor(x: sf.V6, y: sf.V6, sqrt_info: sf.M66, epsilon: sf.Scalar) -> sf.V6:
    return sf.V6(_rn_delta_factor(x, y, sqrt_info, epsilon))


def rot2_delta_factor(x: sf.Rot2, y: sf.Rot2, sqrt_info: sf.M11, epsilon: sf.Scalar) -> sf.V1:
    return sf.V1(_lie_delta_factor(x, y, sqrt_info, epsilon))


def rot3_delta_factor(x: sf.Rot3, y: sf.Rot3, sqrt_info: sf.M33, epsilon: sf.Scalar) -> sf.V3:
    return sf.V3(_lie_delta_factor(x, y, sqrt_info, epsilon))


def pose2_delta_factor(x: sf.Pose2, y: sf.Pose2, sqrt_info: sf.M33, epsilon: sf.Scalar) -> sf.V3:
    return sf.V3(_lie_delta_factor(x, y, sqrt_info, epsilon))


def pose3_delta_factor(x: sf.Pose3, y: sf.Pose3, sqrt_info: sf.M66, epsilon: sf.Scalar) -> sf.V6:
    return sf.V6(_lie_delta_factor(x, y, sqrt_info, epsilon))


def r2_angular_delta(x: sf.V2, y: sf.V2, epsilon: sf.Scalar) -> sf.V1:
    return sf.V1(sf.atan2(x[0] * y[1] - x[1] * y[0], x.dot(y)))


def r3_angular_delta(x: sf.V3, y: sf.V3, epsilon: sf.Scalar) -> sf.V1:
    return sf.V1(sf.atan2(x.cross(y).norm(epsilon), x.dot(y)))


def r2_angular_delta_factor(x: sf.V2, y: sf.V2, sqrt_info: sf.M11, epsilon: sf.Scalar) -> sf.V1:
    return sqrt_info * r2_angular_delta(x, y, epsilon)


def r3_angular_delta_factor(x: sf.V3, y: sf.V3, sqrt_info: sf.M11, epsilon: sf.Scalar) -> sf.V1:
    return sqrt_info * r3_angular_delta(x, y, epsilon)
