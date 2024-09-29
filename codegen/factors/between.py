from codegen.factors.delta import *


def r1_between_factor(x: sf.V1, x_T_y: sf.V1, y: sf.V1, sqrt_info: sf.M11, epsilon: sf.Scalar) -> sf.V1:
    return r1_delta_factor(x_T_y, x - y, sqrt_info, epsilon)


def r2_between_factor(x: sf.V2, x_T_y: sf.V2, y: sf.V2, sqrt_info: sf.M22, epsilon: sf.Scalar) -> sf.V2:
    return r2_delta_factor(x_T_y, x - y, sqrt_info, epsilon)


def r3_between_factor(x: sf.V3, x_T_y: sf.V3, y: sf.V3, sqrt_info: sf.M33, epsilon: sf.Scalar) -> sf.V3:
    return r3_delta_factor(x_T_y, x - y, sqrt_info, epsilon)


def rot2_between_factor(x: sf.Rot2, x_T_y: sf.Rot2, y: sf.Rot2, sqrt_info: sf.M11, epsilon: sf.Scalar) -> sf.V1:
    return rot2_delta_factor(x_T_y, x.between(y), sqrt_info, epsilon)


def rot3_between_factor(x: sf.Rot3, x_T_y: sf.Rot3, y: sf.Rot3, sqrt_info: sf.M33, epsilon: sf.Scalar) -> sf.V3:
    return rot3_delta_factor(x_T_y, x.between(y), sqrt_info, epsilon)


def pose2_between_factor(x: sf.Pose2, x_T_y: sf.Pose2, y: sf.Pose2, sqrt_info: sf.M33, epsilon: sf.Scalar) -> sf.V3:
    return pose2_delta_factor(x_T_y, x.between(y), sqrt_info, epsilon)


def pose3_between_factor(x: sf.Pose3, x_T_y: sf.Pose3, y: sf.Pose3, sqrt_info: sf.M66, epsilon: sf.Scalar) -> sf.V6:
    return pose3_delta_factor(x_T_y, x.between(y), sqrt_info, epsilon)
