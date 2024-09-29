from codegen.factors.standard import *
from codegen.splines.evaluator import *
from codegen.splines.spline4 import *


# ===============================
# Delta factors
# ===============================

def spline4_rot3_delta_factor(
        lambdas: sf.M41, x0: sf.Rot3, x1: sf.Rot3, x2: sf.Rot3, x3: sf.Rot3, x4: sf.Rot3,
        y: sf.Rot3, sqrt_info: sf.M33, epsilon: sf.Scalar) -> sf.V3:
    estimate = spline_value(lambdas, [x0, x1, x2, x3, x4], epsilon)
    return rot3_delta_factor(y, estimate, sqrt_info, epsilon)


def spline4_pose3_delta_factor(
        lambdas: sf.M41, x0: sf.Pose3, x1: sf.Pose3, x2: sf.Pose3, x3: sf.Pose3, x4: sf.Pose3,
        y: sf.Pose3, sqrt_info: sf.M66, epsilon: sf.Scalar) -> sf.V6:
    estimate = spline_value(lambdas, [x0, x1, x2, x3, x4], epsilon)
    return pose3_delta_factor(y, estimate, sqrt_info, epsilon)


# ===============================
# Velocity factors
# ===============================

def spline4_rot3_velocity_factor(
        frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M42, x0: sf.Rot3, x1: sf.Rot3, x2: sf.Rot3, x3: sf.Rot3,
        x4: sf.Rot3, velocity: sf.V3, sqrt_info: sf.M33, epsilon: sf.Scalar) -> sf.V3:
    estimate = spline4_rot3_velocity(frame_type, dt, lambdas, x0, x1, x2, x3, x4, epsilon)
    return r3_delta_factor(velocity, estimate, sqrt_info, epsilon)


def spline4_pose3_velocity_factor(
        frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M42, x0: sf.Pose3, x1: sf.Pose3, x2: sf.Pose3, x3: sf.Pose3,
        x4: sf.Pose3, velocity: sf.V6, sqrt_info: sf.M66, epsilon: sf.Scalar) -> sf.V6:
    estimate = spline4_pose3_velocity(frame_type, dt, lambdas, x0, x1, x2, x3, x4, epsilon)
    return r6_delta_factor(velocity, estimate, sqrt_info, epsilon)


# ===============================
# Acceleration factors
# ===============================

def spline4_rot3_acceleration_factor(
        frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M43, x0: sf.Rot3, x1: sf.Rot3, x2: sf.Rot3, x3: sf.Rot3,
        x4: sf.Rot3, acceleration: sf.V3, sqrt_info: sf.M33, epsilon: sf.Scalar) -> sf.V3:
    estimate = spline4_rot3_acceleration(frame_type, dt, lambdas, x0, x1, x2, x3, x4, epsilon)
    return r3_delta_factor(acceleration, estimate, sqrt_info, epsilon)


def spline4_pose3_acceleration_factor(
        frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M43, x0: sf.Pose3, x1: sf.Pose3, x2: sf.Pose3, x3: sf.Pose3,
        x4: sf.Pose3, acceleration: sf.V6, sqrt_info: sf.M66, epsilon: sf.Scalar) -> sf.V6:
    estimate = spline4_pose3_acceleration(frame_type, dt, lambdas, x0, x1, x2, x3, x4, epsilon)
    return r6_delta_factor(acceleration, estimate, sqrt_info, epsilon)
