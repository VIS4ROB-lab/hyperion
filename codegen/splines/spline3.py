import codegen.splines.evaluator as evaluator

from codegen.splines.evaluator import FrameType
from codegen.splines.utils import *


def spline3_r1_value(lambdas: sf.M31, x0: sf.V1, x1: sf.V1, x2: sf.V1, x3: sf.V1, epsilon: sf.Scalar) -> sf.V1:
    return evaluator.spline_value(lambdas, [x0, x1, x2, x3], epsilon)


def spline3_r2_value(lambdas: sf.M31, x0: sf.V2, x1: sf.V2, x2: sf.V2, x3: sf.V2, epsilon: sf.Scalar) -> sf.V2:
    return evaluator.spline_value(lambdas, [x0, x1, x2, x3], epsilon)


def spline3_r3_value(lambdas: sf.M31, x0: sf.V3, x1: sf.V3, x2: sf.V3, x3: sf.V3, epsilon: sf.Scalar) -> sf.V3:
    return evaluator.spline_value(lambdas, [x0, x1, x2, x3], epsilon)


def spline3_rot2_value(lambdas: sf.M31, x0: sf.Rot2, x1: sf.Rot2, x2: sf.Rot2, x3: sf.Rot2,
                       epsilon: sf.Scalar) -> sf.Rot2:
    return evaluator.spline_value(lambdas, [x0, x1, x2, x3], epsilon)


def spline3_rot3_value(lambdas: sf.M31, x0: sf.Rot3, x1: sf.Rot3, x2: sf.Rot3, x3: sf.Rot3,
                       epsilon: sf.Scalar) -> sf.Rot3:
    return evaluator.spline_value(lambdas, [x0, x1, x2, x3], epsilon)


def spline3_pose2_value(lambdas: sf.M31, x0: sf.Pose2, x1: sf.Pose2, x2: sf.Pose2, x3: sf.Pose2,
                        epsilon: sf.Scalar) -> sf.Pose2:
    return evaluator.spline_value(lambdas, [x0, x1, x2, x3], epsilon)


def spline3_pose3_value(lambdas: sf.M31, x0: sf.Pose3, x1: sf.Pose3, x2: sf.Pose3, x3: sf.Pose3,
                        epsilon: sf.Scalar) -> sf.Pose3:
    return evaluator.spline_value(lambdas, [x0, x1, x2, x3], epsilon)


def spline3_r1_velocity(frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M32, x0: sf.V1, x1: sf.V1, x2: sf.V1,
                        x3: sf.V1, epsilon: sf.Scalar) -> sf.V1:
    return evaluator.spline_velocity(frame_type, dt, lambdas, [x0, x1, x2, x3], epsilon)


def spline3_r2_velocity(frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M32, x0: sf.V2, x1: sf.V2, x2: sf.V2,
                        x3: sf.V2, epsilon: sf.Scalar) -> sf.V2:
    return evaluator.spline_velocity(frame_type, dt, lambdas, [x0, x1, x2, x3], epsilon)


def spline3_r3_velocity(frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M32, x0: sf.V3, x1: sf.V3, x2: sf.V3,
                        x3: sf.V3, epsilon: sf.Scalar) -> sf.V3:
    return evaluator.spline_velocity(frame_type, dt, lambdas, [x0, x1, x2, x3], epsilon)


def spline3_rot2_velocity(frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M32, x0: sf.Rot2, x1: sf.Rot2, x2: sf.Rot2,
                          x3: sf.Rot2, epsilon: sf.Scalar) -> sf.V1:
    return evaluator.spline_velocity(frame_type, dt, lambdas, [x0, x1, x2, x3], epsilon)


def spline3_rot3_velocity(frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M32, x0: sf.Rot3, x1: sf.Rot3, x2: sf.Rot3,
                          x3: sf.Rot3, epsilon: sf.Scalar) -> sf.V3:
    return evaluator.spline_velocity(frame_type, dt, lambdas, [x0, x1, x2, x3], epsilon)


def spline3_pose2_velocity(frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M32, x0: sf.Pose2, x1: sf.Pose2,
                           x2: sf.Pose2, x3: sf.Pose2, epsilon: sf.Scalar) -> sf.V3:
    return evaluator.spline_velocity(frame_type, dt, lambdas, [x0, x1, x2, x3], epsilon)


def spline3_pose3_velocity(frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M32, x0: sf.Pose3, x1: sf.Pose3,
                           x2: sf.Pose3, x3: sf.Pose3, epsilon: sf.Scalar) -> sf.V6:
    return evaluator.spline_velocity(frame_type, dt, lambdas, [x0, x1, x2, x3], epsilon)


def spline3_r1_acceleration(frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M33, x0: sf.V1, x1: sf.V1, x2: sf.V1,
                            x3: sf.V1, epsilon: sf.Scalar) -> sf.V1:
    return evaluator.spline_acceleration(frame_type, dt, lambdas, [x0, x1, x2, x3], epsilon)


def spline3_r2_acceleration(frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M33, x0: sf.V2, x1: sf.V2, x2: sf.V2,
                            x3: sf.V2, epsilon: sf.Scalar) -> sf.V2:
    return evaluator.spline_acceleration(frame_type, dt, lambdas, [x0, x1, x2, x3], epsilon)


def spline3_r3_acceleration(frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M33, x0: sf.V3, x1: sf.V3, x2: sf.V3,
                            x3: sf.V3, epsilon: sf.Scalar) -> sf.V3:
    return evaluator.spline_acceleration(frame_type, dt, lambdas, [x0, x1, x2, x3], epsilon)


def spline3_rot2_acceleration(frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M33, x0: sf.Rot2, x1: sf.Rot2,
                              x2: sf.Rot2, x3: sf.Rot2, epsilon: sf.Scalar) -> sf.V1:
    return evaluator.spline_acceleration(frame_type, dt, lambdas, [x0, x1, x2, x3], epsilon)


def spline3_rot3_acceleration(frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M33, x0: sf.Rot3, x1: sf.Rot3,
                              x2: sf.Rot3, x3: sf.Rot3, epsilon: sf.Scalar) -> sf.V3:
    return evaluator.spline_acceleration(frame_type, dt, lambdas, [x0, x1, x2, x3], epsilon)


def spline3_pose2_acceleration(frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M33, x0: sf.Pose2, x1: sf.Pose2,
                               x2: sf.Pose2, x3: sf.Pose2, epsilon: sf.Scalar) -> sf.V3:
    return evaluator.spline_acceleration(frame_type, dt, lambdas, [x0, x1, x2, x3], epsilon)


def spline3_pose3_acceleration(frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M33, x0: sf.Pose3, x1: sf.Pose3,
                               x2: sf.Pose3, x3: sf.Pose3, epsilon: sf.Scalar) -> sf.V6:
    return evaluator.spline_acceleration(frame_type, dt, lambdas, [x0, x1, x2, x3], epsilon)


def spline3_pose2_sensor_velocity(frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M32, x0: sf.Pose2, x1: sf.Pose2,
                                  x2: sf.Pose2, x3: sf.Pose2, x_T_s: sf.Pose2, epsilon: sf.Scalar) -> sf.V3:
    v = evaluator.spline_velocity(frame_type, dt, lambdas, [x0, x1, x2, x3], epsilon)
    return v if frame_type == FrameType.GLOBAL else adj_pose2(x_T_s.inverse()) * v


def spline3_pose3_sensor_velocity(frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M32, x0: sf.Pose3, x1: sf.Pose3,
                                  x2: sf.Pose3, x3: sf.Pose3, x_T_s: sf.Pose3, epsilon: sf.Scalar) -> sf.V6:
    v = evaluator.spline_velocity(frame_type, dt, lambdas, [x0, x1, x2, x3], epsilon)
    return v if frame_type == FrameType.GLOBAL else adj_pose3(x_T_s.inverse()) * v


def spline3_pose2_sensor_acceleration(frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M33, x0: sf.Pose2, x1: sf.Pose2,
                                      x2: sf.Pose2, x3: sf.Pose2, x_T_s: sf.Pose2, epsilon: sf.Scalar) -> sf.V3:
    v = evaluator.spline_acceleration(frame_type, dt, lambdas, [x0, x1, x2, x3], epsilon)
    return v if frame_type == FrameType.GLOBAL else adj_pose2(x_T_s.inverse()) * v


def spline3_pose3_sensor_acceleration(frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M33, x0: sf.Pose3, x1: sf.Pose3,
                                      x2: sf.Pose3, x3: sf.Pose3, x_T_s: sf.Pose3, epsilon: sf.Scalar) -> sf.V6:
    v = evaluator.spline_acceleration(frame_type, dt, lambdas, [x0, x1, x2, x3], epsilon)
    return v if frame_type == FrameType.GLOBAL else adj_pose3(x_T_s.inverse()) * v
