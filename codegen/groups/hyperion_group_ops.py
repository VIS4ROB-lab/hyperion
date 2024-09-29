import symforce.symbolic as sf


def delta_rot2(x: sf.Rot2, y: sf.Rot2, epsilon: sf.Scalar) -> sf.V1:
    return sf.V1.from_flat_list(x.local_coordinates(y, epsilon))


def retract_rot2(value: sf.Rot2, tau: sf.V1, epsilon: sf.Scalar) -> sf.Rot2:
    return value.retract(tau.to_storage(), epsilon)


def delta_rot3(x: sf.Rot3, y: sf.Rot3, epsilon: sf.Scalar) -> sf.V3:
    return sf.V3.from_flat_list(x.local_coordinates(y, epsilon))


def retract_rot3(value: sf.Rot3, tau: sf.V3, epsilon: sf.Scalar) -> sf.Rot3:
    return value.retract(tau.to_storage(), epsilon)


def delta_pose2(x: sf.Pose2, y: sf.Pose2, epsilon: sf.Scalar) -> sf.V3:
    return sf.V3.from_flat_list(x.local_coordinates(y, epsilon))


def retract_pose2(value: sf.Pose2, tau: sf.V3, epsilon: sf.Scalar) -> sf.Pose2:
    return value.retract(tau.to_storage(), epsilon)


def delta_pose3(x: sf.Pose3, y: sf.Pose3, epsilon: sf.Scalar) -> sf.V6:
    return sf.V6.from_flat_list(x.local_coordinates(y, epsilon))


def retract_pose3(value: sf.Pose3, tau: sf.V6, epsilon: sf.Scalar) -> sf.Pose3:
    return value.retract(tau.to_storage(), epsilon)
