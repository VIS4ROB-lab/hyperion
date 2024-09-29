from codegen.factors.standard import *
from codegen.splines.evaluator import *
from codegen.splines.spline3 import *


# ===============================
# Delta factors
# ===============================

def spline3_r3_delta_factor(
        lambdas: sf.M31, x0: sf.V3, x1: sf.V3, x2: sf.V3, x3: sf.V3,
        y: sf.V3, sqrt_info: sf.M33, epsilon: sf.Scalar) -> sf.V3:
    estimate = spline_value(lambdas, [x0, x1, x2, x3], epsilon)
    return r3_delta_factor(y, estimate, sqrt_info, epsilon)


def spline3_rot2_delta_factor(
        lambdas: sf.M31, x0: sf.Rot2, x1: sf.Rot2, x2: sf.Rot2, x3: sf.Rot2,
        y: sf.Rot2, sqrt_info: sf.M11, epsilon: sf.Scalar) -> sf.V1:
    estimate = spline_value(lambdas, [x0, x1, x2, x3], epsilon)
    return rot2_delta_factor(y, estimate, sqrt_info, epsilon)


def spline3_rot3_delta_factor(
        lambdas: sf.M31, x0: sf.Rot3, x1: sf.Rot3, x2: sf.Rot3, x3: sf.Rot3,
        y: sf.Rot3, sqrt_info: sf.M33, epsilon: sf.Scalar) -> sf.V3:
    estimate = spline_value(lambdas, [x0, x1, x2, x3], epsilon)
    return rot3_delta_factor(y, estimate, sqrt_info, epsilon)


def spline3_pose2_delta_factor(
        lambdas: sf.M31, x0: sf.Pose2, x1: sf.Pose2, x2: sf.Pose2, x3: sf.Pose2,
        y: sf.Pose2, sqrt_info: sf.M33, epsilon: sf.Scalar) -> sf.V3:
    estimate = spline_value(lambdas, [x0, x1, x2, x3], epsilon)
    return pose2_delta_factor(y, estimate, sqrt_info, epsilon)


def spline3_pose2_r2_delta_factor(
        lambdas: sf.M31, x0: sf.Pose2, x1: sf.Pose2, x2: sf.Pose2, x3: sf.Pose2,
        y: sf.V2, sqrt_info: sf.M22, epsilon: sf.Scalar) -> sf.V2:
    estimate = spline_value(lambdas, [x0, x1, x2, x3], epsilon)
    return r2_delta_factor(y, estimate.position(), sqrt_info, epsilon)


def spline3_pose2_rot2_delta_factor(
        lambdas: sf.M31, x0: sf.Pose2, x1: sf.Pose2, x2: sf.Pose2, x3: sf.Pose2,
        y: sf.Rot2, sqrt_info: sf.M11, epsilon: sf.Scalar) -> sf.V1:
    estimate = spline_value(lambdas, [x0, x1, x2, x3], epsilon)
    return rot2_delta_factor(y, estimate.rotation(), sqrt_info, epsilon)


def spline3_pose3_delta_factor(
        lambdas: sf.M31, x0: sf.Pose3, x1: sf.Pose3, x2: sf.Pose3, x3: sf.Pose3,
        y: sf.Pose3, sqrt_info: sf.M66, epsilon: sf.Scalar) -> sf.V6:
    estimate = spline_value(lambdas, [x0, x1, x2, x3], epsilon)
    return pose3_delta_factor(y, estimate, sqrt_info, epsilon)


def spline3_pose3_r3_delta_factor(
        lambdas: sf.M31, x0: sf.Pose3, x1: sf.Pose3, x2: sf.Pose3, x3: sf.Pose3,
        y: sf.V3, sqrt_info: sf.M33, epsilon: sf.Scalar) -> sf.V3:
    estimate = spline_value(lambdas, [x0, x1, x2, x3], epsilon)
    return r3_delta_factor(y, estimate.position(), sqrt_info, epsilon)


def spline3_pose3_rot3_delta_factor(
        lambdas: sf.M31, x0: sf.Pose3, x1: sf.Pose3, x2: sf.Pose3, x3: sf.Pose3,
        y: sf.Rot3, sqrt_info: sf.M33, epsilon: sf.Scalar) -> sf.V3:
    estimate = spline_value(lambdas, [x0, x1, x2, x3], epsilon)
    return rot3_delta_factor(y, estimate.rotation(), sqrt_info, epsilon)


def spline3_pose2_sensor_delta_factor(
        lambdas: sf.M31, x0: sf.Pose2, x1: sf.Pose2, x2: sf.Pose2, x3: sf.Pose2,
        x_T_y: sf.Pose2, y: sf.Pose2, sqrt_info: sf.M33, epsilon: sf.Scalar) -> sf.V3:
    x = spline_value(lambdas, [x0, x1, x2, x3], epsilon)
    return pose2_sensor_delta_factor(x, x_T_y, y, sqrt_info, epsilon)


def spline3_pose2_r2_sensor_delta_factor(
        lambdas: sf.M31, x0: sf.Pose2, x1: sf.Pose2, x2: sf.Pose2, x3: sf.Pose2,
        x_T_y: sf.V2, y: sf.Pose2, sqrt_info: sf.M22, epsilon: sf.Scalar) -> sf.V2:
    x = spline_value(lambdas, [x0, x1, x2, x3], epsilon)
    return pose2_r2_sensor_delta_factor(x, x_T_y, y, sqrt_info, epsilon)


def spline3_pose2_rot2_sensor_delta_factor(
        lambdas: sf.M31, x0: sf.Pose2, x1: sf.Pose2, x2: sf.Pose2, x3: sf.Pose2,
        x_T_y: sf.Rot2, y: sf.Pose2, sqrt_info: sf.M11, epsilon: sf.Scalar) -> sf.V1:
    x = spline_value(lambdas, [x0, x1, x2, x3], epsilon)
    return pose2_rot2_sensor_delta_factor(x, x_T_y, y, sqrt_info, epsilon)


def spline3_pose3_sensor_delta_factor(
        lambdas: sf.M31, x0: sf.Pose3, x1: sf.Pose3, x2: sf.Pose3, x3: sf.Pose3,
        x_T_y: sf.Pose3, y: sf.Pose3, sqrt_info: sf.M66, epsilon: sf.Scalar) -> sf.V6:
    x = spline_value(lambdas, [x0, x1, x2, x3], epsilon)
    return pose3_sensor_delta_factor(x, x_T_y, y, sqrt_info, epsilon)


def spline3_pose3_r3_sensor_delta_factor(
        lambdas: sf.M31, x0: sf.Pose3, x1: sf.Pose3, x2: sf.Pose3, x3: sf.Pose3,
        x_T_y: sf.V3, y: sf.Pose3, sqrt_info: sf.M33, epsilon: sf.Scalar) -> sf.V3:
    x = spline_value(lambdas, [x0, x1, x2, x3], epsilon)
    return pose3_r3_sensor_delta_factor(x, x_T_y, y, sqrt_info, epsilon)


def spline3_pose3_rot3_sensor_delta_factor(
        lambdas: sf.M31, x0: sf.Pose3, x1: sf.Pose3, x2: sf.Pose3, x3: sf.Pose3,
        x_T_y: sf.Rot3, y: sf.Pose3, sqrt_info: sf.M33, epsilon: sf.Scalar) -> sf.V3:
    x = spline_value(lambdas, [x0, x1, x2, x3], epsilon)
    return pose3_rot3_sensor_delta_factor(x, x_T_y, y, sqrt_info, epsilon)


# ===============================
# Between factors
# ===============================

def spline3_pose2_sensor_between_factor(
        lambdas: sf.M31, x0: sf.Pose2, x1: sf.Pose2, x2: sf.Pose2, x3: sf.Pose2,
        x_T_s: sf.Pose2, s_T_y: sf.Pose2, y: sf.Pose2,
        sqrt_info: sf.M33, epsilon: sf.Scalar) -> sf.V3:
    x = spline_value(lambdas, [x0, x1, x2, x3], epsilon)
    return pose2_sensor_between_factor(x, x_T_s, s_T_y, y, sqrt_info, epsilon)


def spline3_pose2_r2_sensor_between_factor(
        lambdas: sf.M31, x0: sf.Pose2, x1: sf.Pose2, x2: sf.Pose2, x3: sf.Pose2,
        x_T_s: sf.Pose2, s_T_y: sf.V2, y: sf.Pose2,
        sqrt_info: sf.M22, epsilon: sf.Scalar) -> sf.V2:
    x = spline_value(lambdas, [x0, x1, x2, x3], epsilon)
    return pose2_r2_sensor_between_factor(x, x_T_s, s_T_y, y, sqrt_info, epsilon)


def spline3_pose2_rot2_sensor_between_factor(
        lambdas: sf.M31, x0: sf.Pose2, x1: sf.Pose2, x2: sf.Pose2, x3: sf.Pose2,
        x_T_s: sf.Pose2, s_T_y: sf.Rot2, y: sf.Pose2,
        sqrt_info: sf.M11, epsilon: sf.Scalar) -> sf.V1:
    x = spline_value(lambdas, [x0, x1, x2, x3], epsilon)
    return pose2_rot2_sensor_between_factor(x, x_T_s, s_T_y, y, sqrt_info, epsilon)


def spline3_pose3_sensor_between_factor(
        lambdas: sf.M31, x0: sf.Pose3, x1: sf.Pose3, x2: sf.Pose3, x3: sf.Pose3,
        x_T_s: sf.Pose3, s_T_y: sf.Pose3, y: sf.Pose3,
        sqrt_info: sf.M66, epsilon: sf.Scalar) -> sf.V6:
    x = spline_value(lambdas, [x0, x1, x2, x3], epsilon)
    return pose3_sensor_between_factor(x, x_T_s, s_T_y, y, sqrt_info, epsilon)


def spline3_pose3_r3_sensor_between_factor(
        lambdas: sf.M31, x0: sf.Pose3, x1: sf.Pose3, x2: sf.Pose3, x3: sf.Pose3,
        x_T_s: sf.Pose3, s_T_y: sf.V3, y: sf.Pose3,
        sqrt_info: sf.M33, epsilon: sf.Scalar) -> sf.V3:
    x = spline_value(lambdas, [x0, x1, x2, x3], epsilon)
    return pose3_r3_sensor_between_factor(x, x_T_s, s_T_y, y, sqrt_info, epsilon)


def spline3_pose3_rot3_sensor_between_factor(
        lambdas: sf.M31, x0: sf.Pose3, x1: sf.Pose3, x2: sf.Pose3, x3: sf.Pose3,
        x_T_s: sf.Pose3, s_T_y: sf.Rot3, y: sf.Pose3,
        sqrt_info: sf.M33, epsilon: sf.Scalar) -> sf.V3:
    x = spline_value(lambdas, [x0, x1, x2, x3], epsilon)
    return pose3_rot3_sensor_between_factor(x, x_T_s, s_T_y, y, sqrt_info, epsilon)


# ===============================
# Relative between factors
# ===============================

def spline3_pose2_sensor_relative_between_factor(
        lambdas_x: sf.M31, x0: sf.Pose2, x1: sf.Pose2, x2: sf.Pose2, x3: sf.Pose2, x_T_a: sf.Pose2,
        a_T_b: sf.Pose2, lambdas_y: sf.M31, y0: sf.Pose2, y1: sf.Pose2, y2: sf.Pose2, y3: sf.Pose2, y_T_b: sf.Pose2,
        sqrt_info: sf.M33, epsilon: sf.Scalar) -> sf.V3:
    x = spline_value(lambdas_x, [x0, x1, x2, x3], epsilon)
    y = spline_value(lambdas_y, [y0, y1, y2, y3], epsilon)
    return pose2_sensor_relative_between_factor(x, x_T_a, a_T_b, y, y_T_b, sqrt_info, epsilon)


def spline3_pose2_r2_sensor_relative_between_factor(
        lambdas_x: sf.M31, x0: sf.Pose2, x1: sf.Pose2, x2: sf.Pose2, x3: sf.Pose2, x_T_a: sf.Pose2,
        a_T_b: sf.V2, lambdas_y: sf.M31, y0: sf.Pose2, y1: sf.Pose2, y2: sf.Pose2, y3: sf.Pose2, y_T_b: sf.Pose2,
        sqrt_info: sf.M22, epsilon: sf.Scalar) -> sf.V2:
    x = spline_value(lambdas_x, [x0, x1, x2, x3], epsilon)
    y = spline_value(lambdas_y, [y0, y1, y2, y3], epsilon)
    return pose2_r2_sensor_relative_between_factor(x, x_T_a, a_T_b, y, y_T_b, sqrt_info, epsilon)


def spline3_pose2_rot2_sensor_relative_between_factor(
        lambdas_x: sf.M31, x0: sf.Pose2, x1: sf.Pose2, x2: sf.Pose2, x3: sf.Pose2, x_T_a: sf.Pose2,
        a_T_b: sf.Rot2, lambdas_y: sf.M31, y0: sf.Pose2, y1: sf.Pose2, y2: sf.Pose2, y3: sf.Pose2, y_T_b: sf.Pose2,
        sqrt_info: sf.M11, epsilon: sf.Scalar) -> sf.V1:
    x = spline_value(lambdas_x, [x0, x1, x2, x3], epsilon)
    y = spline_value(lambdas_y, [y0, y1, y2, y3], epsilon)
    return pose2_rot2_sensor_relative_between_factor(x, x_T_a, a_T_b, y, y_T_b, sqrt_info, epsilon)


def spline3_pose3_sensor_relative_between_factor(
        lambdas_x: sf.M31, x0: sf.Pose3, x1: sf.Pose3, x2: sf.Pose3, x3: sf.Pose3, x_T_a: sf.Pose3,
        a_T_b: sf.Pose3, lambdas_y: sf.M31, y0: sf.Pose3, y1: sf.Pose3, y2: sf.Pose3, y3: sf.Pose3, y_T_b: sf.Pose3,
        sqrt_info: sf.M66, epsilon: sf.Scalar) -> sf.V6:
    x = spline_value(lambdas_x, [x0, x1, x2, x3], epsilon)
    y = spline_value(lambdas_y, [y0, y1, y2, y3], epsilon)
    return pose3_sensor_relative_between_factor(x, x_T_a, a_T_b, y, y_T_b, sqrt_info, epsilon)


def spline3_pose3_r3_sensor_relative_between_factor(
        lambdas_x: sf.M31, x0: sf.Pose3, x1: sf.Pose3, x2: sf.Pose3, x3: sf.Pose3, x_T_a: sf.Pose3,
        a_T_b: sf.V3, lambdas_y: sf.M31, y0: sf.Pose3, y1: sf.Pose3, y2: sf.Pose3, y3: sf.Pose3, y_T_b: sf.Pose3,
        sqrt_info: sf.M33, epsilon: sf.Scalar) -> sf.V3:
    x = spline_value(lambdas_x, [x0, x1, x2, x3], epsilon)
    y = spline_value(lambdas_y, [y0, y1, y2, y3], epsilon)
    return pose3_r3_sensor_relative_between_factor(x, x_T_a, a_T_b, y, y_T_b, sqrt_info, epsilon)


def spline3_pose3_rot3_sensor_relative_between_factor(
        lambdas_x: sf.M31, x0: sf.Pose3, x1: sf.Pose3, x2: sf.Pose3, x3: sf.Pose3, x_T_a: sf.Pose3,
        a_T_b: sf.Rot3, lambdas_y: sf.M31, y0: sf.Pose3, y1: sf.Pose3, y2: sf.Pose3, y3: sf.Pose3, y_T_b: sf.Pose3,
        sqrt_info: sf.M33, epsilon: sf.Scalar) -> sf.V3:
    x = spline_value(lambdas_x, [x0, x1, x2, x3], epsilon)
    y = spline_value(lambdas_y, [y0, y1, y2, y3], epsilon)
    return pose3_rot3_sensor_relative_between_factor(x, x_T_a, a_T_b, y, y_T_b, sqrt_info, epsilon)


# ===============================
# Camera factors
# ===============================

def spline3_pose3_camera_linear_reprojection_factor(
        lambdas: sf.M31, x0: sf.Pose3, x1: sf.Pose3, x2: sf.Pose3, x3: sf.Pose3, b_T_c: sf.Pose3,
        calibration: LinearCameraCal, l_w: sf.V3, pixel: sf.V2, sqrt_info: sf.M22, epsilon: sf.Scalar) -> sf.V2:
    x = spline_value(lambdas, [x0, x1, x2, x3], epsilon)
    return pose3_camera_linear_reprojection_factor(x, b_T_c, calibration, l_w, pixel, sqrt_info, epsilon)


def spline3_pose3_camera_linear_bearing_factor(
        lambdas: sf.M31, x0: sf.Pose3, x1: sf.Pose3, x2: sf.Pose3, x3: sf.Pose3, b_T_c: sf.Pose3,
        calibration: LinearCameraCal, l_w: sf.V3, pixel: sf.V2, sqrt_info: sf.M11, epsilon: sf.Scalar) -> sf.V1:
    x = spline_value(lambdas, [x0, x1, x2, x3], epsilon)
    return pose3_camera_linear_bearing_factor(x, b_T_c, calibration, l_w, pixel, sqrt_info, epsilon)


# ===============================
# Velocity factors
# ===============================

def spline3_r3_velocity_factor(
        frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M32, x0: sf.V3, x1: sf.V3, x2: sf.V3, x3: sf.V3,
        velocity: sf.V3, sqrt_info: sf.M33, epsilon: sf.Scalar) -> sf.V3:
    estimate = spline3_r3_velocity(frame_type, dt, lambdas, x0, x1, x2, x3, epsilon)
    return r3_delta_factor(velocity, estimate, sqrt_info, epsilon)


def spline3_rot2_velocity_factor(
        frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M32, x0: sf.Rot2, x1: sf.Rot2, x2: sf.Rot2, x3: sf.Rot2,
        velocity: sf.V1, sqrt_info: sf.M11, epsilon: sf.Scalar) -> sf.V1:
    estimate = spline3_rot2_velocity(frame_type, dt, lambdas, x0, x1, x2, x3, epsilon)
    return r1_delta_factor(velocity, estimate, sqrt_info, epsilon)


def spline3_rot3_velocity_factor(
        frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M32, x0: sf.Rot3, x1: sf.Rot3, x2: sf.Rot3, x3: sf.Rot3,
        velocity: sf.V3, sqrt_info: sf.M33, epsilon: sf.Scalar) -> sf.V3:
    estimate = spline3_rot3_velocity(frame_type, dt, lambdas, x0, x1, x2, x3, epsilon)
    return r3_delta_factor(velocity, estimate, sqrt_info, epsilon)


def spline3_pose2_velocity_factor(
        frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M32, x0: sf.Pose2, x1: sf.Pose2, x2: sf.Pose2, x3: sf.Pose2,
        velocity: sf.V3, sqrt_info: sf.M33, epsilon: sf.Scalar) -> sf.V3:
    estimate = spline3_pose2_velocity(frame_type, dt, lambdas, x0, x1, x2, x3, epsilon)
    return r3_delta_factor(velocity, estimate, sqrt_info, epsilon)


def spline3_pose3_velocity_factor(
        frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M32, x0: sf.Pose3, x1: sf.Pose3, x2: sf.Pose3, x3: sf.Pose3,
        velocity: sf.V6, sqrt_info: sf.M66, epsilon: sf.Scalar) -> sf.V6:
    estimate = spline3_pose3_velocity(frame_type, dt, lambdas, x0, x1, x2, x3, epsilon)
    return r6_delta_factor(velocity, estimate, sqrt_info, epsilon)


def spline3_pose2_sensor_velocity_factor(
        frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M32, x0: sf.Pose2, x1: sf.Pose2, x2: sf.Pose2, x3: sf.Pose2,
        x_T_s: sf.Pose2, velocity: sf.V3, sqrt_info: sf.M33, epsilon: sf.Scalar) -> sf.V3:
    estimate = spline3_pose2_sensor_velocity(frame_type, dt, lambdas, x0, x1, x2, x3, x_T_s, epsilon)
    return r3_delta_factor(velocity, estimate, sqrt_info, epsilon)


def spline3_pose3_sensor_velocity_factor(
        frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M32, x0: sf.Pose3, x1: sf.Pose3, x2: sf.Pose3, x3: sf.Pose3,
        x_T_s: sf.Pose3, velocity: sf.V6, sqrt_info: sf.M66, epsilon: sf.Scalar) -> sf.V6:
    estimate = spline3_pose3_sensor_velocity(frame_type, dt, lambdas, x0, x1, x2, x3, x_T_s, epsilon)
    return r6_delta_factor(velocity, estimate, sqrt_info, epsilon)


# ===============================
# Acceleration factors
# ===============================

def spline3_r3_acceleration_factor(
        frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M33, x0: sf.V3, x1: sf.V3, x2: sf.V3, x3: sf.V3,
        acceleration: sf.V3, sqrt_info: sf.M33, epsilon: sf.Scalar) -> sf.V3:
    estimate = spline3_r3_acceleration(frame_type, dt, lambdas, x0, x1, x2, x3, epsilon)
    return r3_delta_factor(acceleration, estimate, sqrt_info, epsilon)


def spline3_rot2_acceleration_factor(
        frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M33, x0: sf.Rot2, x1: sf.Rot2, x2: sf.Rot2, x3: sf.Rot2,
        acceleration: sf.V1, sqrt_info: sf.M11, epsilon: sf.Scalar) -> sf.V1:
    estimate = spline3_rot2_acceleration(frame_type, dt, lambdas, x0, x1, x2, x3, epsilon)
    return r1_delta_factor(acceleration, estimate, sqrt_info, epsilon)


def spline3_rot3_acceleration_factor(
        frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M33, x0: sf.Rot3, x1: sf.Rot3, x2: sf.Rot3, x3: sf.Rot3,
        acceleration: sf.V3, sqrt_info: sf.M33, epsilon: sf.Scalar) -> sf.V3:
    estimate = spline3_rot3_acceleration(frame_type, dt, lambdas, x0, x1, x2, x3, epsilon)
    return r3_delta_factor(acceleration, estimate, sqrt_info, epsilon)


def spline3_pose2_acceleration_factor(
        frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M33, x0: sf.Pose2, x1: sf.Pose2, x2: sf.Pose2, x3: sf.Pose2,
        acceleration: sf.V3, sqrt_info: sf.M33, epsilon: sf.Scalar) -> sf.V3:
    estimate = spline3_pose2_acceleration(frame_type, dt, lambdas, x0, x1, x2, x3, epsilon)
    return r3_delta_factor(acceleration, estimate, sqrt_info, epsilon)


def spline3_pose3_acceleration_factor(
        frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M33, x0: sf.Pose3, x1: sf.Pose3, x2: sf.Pose3, x3: sf.Pose3,
        acceleration: sf.V6, sqrt_info: sf.M66, epsilon: sf.Scalar) -> sf.V6:
    estimate = spline3_pose3_acceleration(frame_type, dt, lambdas, x0, x1, x2, x3, epsilon)
    return r6_delta_factor(acceleration, estimate, sqrt_info, epsilon)


def spline3_pose2_sensor_acceleration_factor(
        frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M33, x0: sf.Pose2, x1: sf.Pose2, x2: sf.Pose2, x3: sf.Pose2,
        x_T_s: sf.Pose2, acceleration: sf.V3, sqrt_info: sf.M33, epsilon: sf.Scalar) -> sf.V3:
    estimate = spline3_pose2_sensor_acceleration(frame_type, dt, lambdas, x0, x1, x2, x3, x_T_s, epsilon)
    return r3_delta_factor(acceleration, estimate, sqrt_info, epsilon)


def spline3_pose3_sensor_acceleration_factor(
        frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M33, x0: sf.Pose3, x1: sf.Pose3, x2: sf.Pose3, x3: sf.Pose3,
        x_T_s: sf.Pose3, acceleration: sf.V6, sqrt_info: sf.M66, epsilon: sf.Scalar) -> sf.V6:
    estimate = spline3_pose3_sensor_acceleration(frame_type, dt, lambdas, x0, x1, x2, x3, x_T_s, epsilon)
    return r6_delta_factor(acceleration, estimate, sqrt_info, epsilon)


# ===============================
# Inertial factors
# ===============================

def spline3_pose3_imu_factor(
        dt: sf.Scalar, lambdas: sf.M33, x0: sf.Pose3, x1: sf.Pose3, x2: sf.Pose3, x3: sf.Pose3,
        x_T_s: sf.Pose3, m: sf.V6, b_g: sf.V3, b_a: sf.V3, g_w: sf.V3, sqrt_info: sf.M66, epsilon: sf.Scalar) -> sf.V6:
    # Evaluate spline value, velocity and acceleration.
    # w_T_s = spline3_pose3_value(FrameType.LOCAL, ut, x0, x1, x2, x3, epsilon).compose(x_T_s)
    # s_v_ws = spline3_pose3_sensor_velocity(spline_type, FrameType.LOCAL, ut, dt, x0, x1, x2, x3, x_T_s, epsilon)
    # s_a_ws = spline3_pose3_sensor_acceleration(spline_type, FrameType.LOCAL, ut, dt, x0, x1, x2, x3, x_T_s, epsilon)

    # Compute the residual.
    res = sf.V6
    # res[:3] = s_v_ws[:3] + b_g + m[:3]
    # res[3:] = s_a_ws[3:] + w_T_s.rotation().inverse() * g_w + b_a + m[3:]
    return sqrt_info * res
