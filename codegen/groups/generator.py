from symforce.codegen import Codegen, CppConfig
from pathlib import Path

from codegen.groups.ceres_group_ops import *
from codegen.groups.hyperion_group_ops import *


def generate_ceres(output_dir: Path):
    funcs = [
        storage_d_tangent_rot2,
        tangent_d_storage_rot2,
        storage_d_tangent_rot3,
        tangent_d_storage_rot3,
        storage_d_tangent_pose2,
        tangent_d_storage_pose2,
        storage_d_tangent_pose3,
        tangent_d_storage_pose3
    ]

    for func in funcs:
        codegen = Codegen.function(func, config=CppConfig())
        codegen.generate_function(output_dir=output_dir, skip_directory_nesting=True)
        print("Generated:", func.__name__, '(Ceres)')


def generate_hyperion(output_dir: Path):
    funcs = [
        (delta_rot2, ["x", "y"]),
        (retract_rot2, ["value", "tau"]),
        (delta_rot3, ["x", "y"]),
        (retract_rot3, ["value", "tau"]),
        (delta_pose2, ["x", "y"]),
        (retract_pose2, ["value", "tau"]),
        (delta_pose3, ["x", "y"]),
        (retract_pose3, ["value", "tau"])
    ]

    for func, which_args in funcs:
        codegen = Codegen.function(func, config=CppConfig())
        codegen = codegen.with_jacobians(name=func.__name__ + '_with_jacobians', which_args=which_args)
        codegen.generate_function(output_dir=output_dir, skip_directory_nesting=True)
        print("Generated:", func.__name__, '(Hyperion)')
