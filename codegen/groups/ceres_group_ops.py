import symforce.symbolic as sf


def storage_d_tangent_rot2(value: sf.Rot2) -> sf.M21:
    return value.storage_D_tangent()


def tangent_d_storage_rot2(value: sf.Rot2) -> sf.M12:
    return value.tangent_D_storage()


def storage_d_tangent_rot3(value: sf.Rot3) -> sf.M43:
    return value.storage_D_tangent()


def tangent_d_storage_rot3(value: sf.Rot3) -> sf.M34:
    return value.tangent_D_storage()


def storage_d_tangent_pose2(value: sf.Pose2) -> sf.M43:
    return sf.M43(value.storage_D_tangent())


def tangent_d_storage_pose2(value: sf.Pose2) -> sf.M34:
    return sf.M34(value.tangent_D_storage())


def storage_d_tangent_pose3(value: sf.Pose3) -> sf.M76:
    return sf.M76(value.storage_D_tangent())


def tangent_d_storage_pose3(value: sf.Pose3) -> sf.M67:
    return sf.M67(value.tangent_D_storage())
