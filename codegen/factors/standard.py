from symforce.cam.linear_camera_cal import LinearCameraCal
from codegen.factors.distance import *


# ===============================
# Delta factors
# ===============================

def pose2_sensor_delta_factor(x: sf.Pose2, x_T_y: sf.Pose2, y: sf.Pose2, sqrt_info: sf.M33,
                              epsilon: sf.Scalar) -> sf.V3:
    est_x_T_y = x.between(y)
    return pose2_delta_factor(x_T_y, est_x_T_y, sqrt_info, epsilon)


def pose2_r2_sensor_delta_factor(x: sf.Pose2, x_T_y: sf.V2, y: sf.Pose2, sqrt_info: sf.M22,
                                 epsilon: sf.Scalar) -> sf.V2:
    est_x_T_y = x.between(y)
    return r2_delta_factor(x_T_y, est_x_T_y.position(), sqrt_info, epsilon)


def pose2_rot2_sensor_delta_factor(x: sf.Pose2, x_T_y: sf.Rot2, y: sf.Pose2, sqrt_info: sf.M11,
                                   epsilon: sf.Scalar) -> sf.V1:
    est_x_T_y = x.between(y)
    return rot2_delta_factor(x_T_y, est_x_T_y.rotation(), sqrt_info, epsilon)


def pose3_sensor_delta_factor(x: sf.Pose3, x_T_y: sf.Pose3, y: sf.Pose3, sqrt_info: sf.M66,
                              epsilon: sf.Scalar) -> sf.V6:
    est_x_T_y = x.between(y)
    return pose3_delta_factor(x_T_y, est_x_T_y, sqrt_info, epsilon)


def pose3_r3_sensor_delta_factor(x: sf.Pose3, x_T_y: sf.V3, y: sf.Pose3, sqrt_info: sf.M33,
                                 epsilon: sf.Scalar) -> sf.V3:
    est_x_T_y = x.between(y)
    return r3_delta_factor(x_T_y, est_x_T_y.position(), sqrt_info, epsilon)


def pose3_rot3_sensor_delta_factor(x: sf.Pose3, x_T_y: sf.Rot3, y: sf.Pose3, sqrt_info: sf.M33,
                                   epsilon: sf.Scalar) -> sf.V3:
    est_x_T_y = x.between(y)
    return rot3_delta_factor(x_T_y, est_x_T_y.rotation(), sqrt_info, epsilon)


# ===============================
# Between factors
# ===============================

def pose2_sensor_between_factor(x: sf.Pose2, x_T_s: sf.Pose2, s_T_y: sf.Pose2, y: sf.Pose2, sqrt_info: sf.M33,
                                epsilon: sf.Scalar) -> sf.V3:
    est_s_T_y = x.compose(x_T_s).between(y)
    return pose2_delta_factor(s_T_y, est_s_T_y, sqrt_info, epsilon)


def pose2_r2_sensor_between_factor(x: sf.Pose2, x_T_s: sf.Pose2, s_T_y: sf.V2, y: sf.Pose2, sqrt_info: sf.M22,
                                   epsilon: sf.Scalar) -> sf.V2:
    est_s_T_y = x.compose(x_T_s).between(y)
    return r2_delta_factor(s_T_y, est_s_T_y.position(), sqrt_info, epsilon)


def pose2_rot2_sensor_between_factor(x: sf.Pose2, x_T_s: sf.Pose2, s_T_y: sf.Rot2, y: sf.Pose2, sqrt_info: sf.M11,
                                     epsilon: sf.Scalar) -> sf.V1:
    est_s_T_y = x.compose(x_T_s).between(y)
    return rot2_delta_factor(s_T_y, est_s_T_y.rotation(), sqrt_info, epsilon)


def pose3_sensor_between_factor(x: sf.Pose3, x_T_s: sf.Pose3, s_T_y: sf.Pose3, y: sf.Pose3, sqrt_info: sf.M66,
                                epsilon: sf.Scalar) -> sf.V6:
    est_s_T_y = x.compose(x_T_s).between(y)
    return pose3_delta_factor(s_T_y, est_s_T_y, sqrt_info, epsilon)


def pose3_r3_sensor_between_factor(x: sf.Pose3, x_T_s: sf.Pose3, s_T_y: sf.V3, y: sf.Pose3, sqrt_info: sf.M33,
                                   epsilon: sf.Scalar) -> sf.V3:
    est_s_T_y = x.compose(x_T_s).between(y)
    return r3_delta_factor(s_T_y, est_s_T_y.position(), sqrt_info, epsilon)


def pose3_rot3_sensor_between_factor(x: sf.Pose3, x_T_s: sf.Pose3, s_T_y: sf.Rot3, y: sf.Pose3, sqrt_info: sf.M33,
                                     epsilon: sf.Scalar) -> sf.V3:
    est_s_T_y = x.compose(x_T_s).between(y)
    return rot3_delta_factor(s_T_y, est_s_T_y.rotation(), sqrt_info, epsilon)


# ===============================
# Relative between factors
# ===============================

def pose2_sensor_relative_between_factor(x: sf.Pose2, x_T_a: sf.Pose2, a_T_b: sf.Pose2, y: sf.Pose2, y_T_b: sf.Pose2,
                                         sqrt_info: sf.M33, epsilon: sf.Scalar) -> sf.V3:
    est_a_T_b = x.compose(x_T_a).between(y.compose(y_T_b))
    return pose2_delta_factor(a_T_b, est_a_T_b, sqrt_info, epsilon)


def pose2_r2_sensor_relative_between_factor(x: sf.Pose2, x_T_a: sf.Pose2, a_T_b: sf.V2, y: sf.Pose2, y_T_b: sf.Pose2,
                                            sqrt_info: sf.M22, epsilon: sf.Scalar) -> sf.V2:
    est_a_T_b = x.compose(x_T_a).between(y.compose(y_T_b))
    return r2_delta_factor(a_T_b, est_a_T_b.position(), sqrt_info, epsilon)


def pose2_rot2_sensor_relative_between_factor(x: sf.Pose2, x_T_a: sf.Pose2, a_T_b: sf.Rot2, y: sf.Pose2,
                                              y_T_b: sf.Pose2, sqrt_info: sf.M11, epsilon: sf.Scalar) -> sf.V1:
    est_a_T_b = x.compose(x_T_a).between(y.compose(y_T_b))
    return rot2_delta_factor(a_T_b, est_a_T_b.rotation(), sqrt_info, epsilon)


def pose3_sensor_relative_between_factor(x: sf.Pose3, x_T_a: sf.Pose3, a_T_b: sf.Pose3, y: sf.Pose3, y_T_b: sf.Pose3,
                                         sqrt_info: sf.M66, epsilon: sf.Scalar) -> sf.V6:
    est_a_T_b = x.compose(x_T_a).between(y.compose(y_T_b))
    return pose3_delta_factor(a_T_b, est_a_T_b, sqrt_info, epsilon)


def pose3_r3_sensor_relative_between_factor(x: sf.Pose3, x_T_a: sf.Pose3, a_T_b: sf.V3, y: sf.Pose3, y_T_b: sf.Pose3,
                                            sqrt_info: sf.M33, epsilon: sf.Scalar) -> sf.V3:
    est_a_T_b = x.compose(x_T_a).between(y.compose(y_T_b))
    return r3_delta_factor(a_T_b, est_a_T_b.position(), sqrt_info, epsilon)


def pose3_rot3_sensor_relative_between_factor(x: sf.Pose3, x_T_a: sf.Pose3, a_T_b: sf.Rot3, y: sf.Pose3,
                                              y_T_b: sf.Pose3, sqrt_info: sf.M33, epsilon: sf.Scalar) -> sf.V3:
    est_a_T_b = x.compose(x_T_a).between(y.compose(y_T_b))
    return rot3_delta_factor(a_T_b, est_a_T_b.rotation(), sqrt_info, epsilon)


# ===============================
# Distance between factors
# ===============================

def pose2_sensor_euclidean_distance_factor(x: sf.Pose2, x_T_s: sf.Pose2, s_d_y: sf.V1, y: sf.Pose2,
                                           sqrt_info: sf.M11, epsilon: sf.Scalar) -> sf.V1:
    s = x.compose(x_T_s)
    return r2_euclidean_distance_factor(s.position(), s_d_y, y.position(), sqrt_info, epsilon)


def pose3_sensor_euclidean_distance_factor(x: sf.Pose3, x_T_s: sf.Pose3, s_d_y: sf.V1, y: sf.Pose3,
                                           sqrt_info: sf.M11, epsilon: sf.Scalar) -> sf.V1:
    s = x.compose(x_T_s)
    return r3_euclidean_distance_factor(s.position(), s_d_y, y.position(), sqrt_info, epsilon)


# ===============================
# Relative distance between factors
# ===============================

def pose2_sensor_relative_euclidean_distance_factor(x: sf.Pose2, x_T_a: sf.Pose2, a_d_b: sf.V1, y: sf.Pose2,
                                                    y_T_b: sf.Pose2, sqrt_info: sf.M11, epsilon: sf.Scalar) -> sf.V1:
    a = x.compose(x_T_a)
    b = y.compose(y_T_b)
    return r2_euclidean_distance_factor(a.position(), a_d_b, b.position(), sqrt_info, epsilon)


def pose3_sensor_relative_euclidean_distance_factor(x: sf.Pose3, x_T_a: sf.Pose3, a_d_b: sf.V1, y: sf.Pose3,
                                                    y_T_b: sf.Pose3, sqrt_info: sf.M11, epsilon: sf.Scalar) -> sf.V1:
    a = x.compose(x_T_a)
    b = y.compose(y_T_b)
    return r3_euclidean_distance_factor(a.position(), a_d_b, b.position(), sqrt_info, epsilon)


# ===============================
# Angle between factors
# ===============================

def pose2_sensor_angle_between_factor(x: sf.Pose2, x_T_s: sf.Pose2, s_d_y: sf.V1, y: sf.Pose2, sqrt_info: sf.M11,
                                      epsilon: sf.Scalar) -> sf.V1:
    s = x.compose(x_T_s)
    return rot2_angular_distance_factor(s.rotation(), s_d_y, y.rotation(), sqrt_info, epsilon)


def pose3_sensor_angle_between_factor(x: sf.Pose3, x_T_s: sf.Pose3, s_d_y: sf.V1, y: sf.Pose3, sqrt_info: sf.M11,
                                      epsilon: sf.Scalar) -> sf.V1:
    s = x.compose(x_T_s)
    return rot3_angular_distance_factor(s.rotation(), s_d_y, y.rotation(), sqrt_info, epsilon)


# ===============================
# Relative angle between factors
# ===============================

def pose2_sensor_relative_angle_between_factor(x: sf.Pose2, x_T_a: sf.Pose2, a_d_b: sf.V1, y: sf.Pose2,
                                               y_T_b: sf.Pose2, sqrt_info: sf.M11, epsilon: sf.Scalar) -> sf.V1:
    a = x.compose(x_T_a)
    b = y.compose(y_T_b)
    return rot2_angular_distance_factor(a.rotation(), a_d_b, b.rotation(), sqrt_info, epsilon)


def pose3_sensor_relative_angle_between_factor(x: sf.Pose3, x_T_a: sf.Pose3, a_d_b: sf.V1, y: sf.Pose3,
                                               y_T_b: sf.Pose3, sqrt_info: sf.M11, epsilon: sf.Scalar) -> sf.V1:
    a = x.compose(x_T_a)
    b = y.compose(y_T_b)
    return rot3_angular_distance_factor(a.rotation(), a_d_b, b.rotation(), sqrt_info, epsilon)


# ===============================
# Camera factors
# ===============================

def pose3_camera_linear_reprojection_factor(w_T_b: sf.Pose3, b_T_c: sf.Pose3, calibration: LinearCameraCal, l_w: sf.V3,
                                            pixel: sf.V2, sqrt_info: sf.M22, epsilon: sf.Scalar) -> sf.V2:
    w_T_c = w_T_b.compose(b_T_c)
    l_c = w_T_c.inverse() * l_w
    est_pixel, valid = calibration.pixel_from_camera_point(l_c, epsilon)
    return valid * r2_delta_factor(pixel, est_pixel, sqrt_info, epsilon)


def pose3_camera_linear_bearing_factor(w_T_b: sf.Pose3, b_T_c: sf.Pose3, calibration: LinearCameraCal, l_w: sf.V3,
                                       pixel: sf.V2, sqrt_info: sf.M11, epsilon: sf.Scalar) -> sf.V1:
    v_c, valid = calibration.camera_ray_from_pixel(pixel, epsilon)
    w_T_c = w_T_b.compose(b_T_c)
    l_c = w_T_c.inverse() * l_w
    return valid * r3_angular_distance_factor(v_c, sf.V1(), l_c, sqrt_info, epsilon)
