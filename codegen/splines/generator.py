from pathlib import Path

from symforce.codegen import Codegen, CppConfig

from codegen.splines.partial import *
from codegen.splines.b_spline import *
from codegen.splines.z_spline import *
from codegen.splines.evaluator import *
from codegen.splines.spline3 import *
from codegen.splines.spline4 import *
from codegen.splines.spline5 import *


def generate_b_spline_matrices(output_dir: Path):
    funcs = []
    for order in [4, 5, 6]:
        # funcs.append(partial(uniform_b_spline_matrix, n=order))
        funcs.append(partial(cumulative_uniform_b_spline_matrix, n=order))
        for k in range(order):
            # funcs.append(partial(uniform_b_spline_lambdas, n=order, k=k))
            funcs.append(partial(cumulative_uniform_b_spline_lambdas, n=order, k=k))

    for func in funcs:
        name = get_name_from_potential_partial(func) + get_suffix_from_potential_partial(func)
        codegen = Codegen.function(func, config=CppConfig(), name=name)
        codegen.generate_function(output_dir=output_dir, skip_directory_nesting=True)
        print("Generated:", name)


def generate_z_spline_matrices(output_dir: Path):
    funcs = []
    for order in [4, 6]:
        # funcs.append(partial(uniform_z_spline_matrix, n=order))
        funcs.append(partial(cumulative_uniform_z_spline_matrix, n=order))
        for k in range(order):
            # funcs.append(partial(uniform_z_spline_lambdas, n=order, k=k))
            funcs.append(partial(cumulative_uniform_z_spline_lambdas, n=order, k=k))

    for func in funcs:
        name = get_name_from_potential_partial(func) + get_suffix_from_potential_partial(func)
        codegen = Codegen.function(func, config=CppConfig(), name=name)
        codegen.generate_function(output_dir=output_dir, skip_directory_nesting=True)
        print("Generated:", name)


def generate_value_func(output_dir: Path, func):
    codegen = Codegen.function(func, config=CppConfig())
    codegen.generate_function(output_dir=output_dir, skip_directory_nesting=True)
    print("Generated:", codegen.name)


def generate_derivative_func(output_dir: Path, func, frame_type: FrameType):
    partial_func = partial(func, frame_type)
    name = '_'.join((func.__name__, get_frame_type_str(frame_type)))
    codegen = Codegen.function(partial_func, config=CppConfig(), name=name)
    codegen.generate_function(output_dir=output_dir, skip_directory_nesting=True)
    print("Generated:", codegen.name)


def generate_spline3(output_dir: Path):
    value_funcs = [
        spline3_r1_value,
        spline3_r2_value,
        spline3_r3_value,
        spline3_rot2_value,
        spline3_rot3_value,
        spline3_pose2_value,
        spline3_pose3_value,
    ]

    for func in value_funcs:
        generate_value_func(output_dir, func)

    derivative_funcs = [
        spline3_r1_velocity,
        spline3_r2_velocity,
        spline3_r3_velocity,
        spline3_rot2_velocity,
        spline3_rot3_velocity,
        spline3_pose2_velocity,
        spline3_pose3_velocity,
        spline3_pose2_sensor_velocity,
        spline3_pose3_sensor_velocity,

        spline3_r1_acceleration,
        spline3_r2_acceleration,
        spline3_r3_acceleration,
        spline3_rot2_acceleration,
        spline3_rot3_acceleration,
        spline3_pose2_acceleration,
        spline3_pose3_acceleration,
        spline3_pose2_sensor_acceleration,
        spline3_pose3_sensor_acceleration,
    ]

    for func in derivative_funcs:
        for frame_type in FrameType:
            generate_derivative_func(output_dir, func, frame_type)


def generate_spline4(output_dir: Path):
    value_funcs = [
        spline4_r1_value,
        spline4_r2_value,
        spline4_r3_value,
        spline4_rot2_value,
        spline4_rot3_value,
        spline4_pose2_value,
        spline4_pose3_value,
    ]

    for func in value_funcs:
        generate_value_func(output_dir, func)

    derivative_funcs = [
        spline4_r1_velocity,
        spline4_r2_velocity,
        spline4_r3_velocity,
        spline4_rot2_velocity,
        spline4_rot3_velocity,
        spline4_pose2_velocity,
        spline4_pose3_velocity,
        spline4_pose2_sensor_velocity,
        spline4_pose3_sensor_velocity,

        spline4_r1_acceleration,
        spline4_r2_acceleration,
        spline4_r3_acceleration,
        spline4_rot2_acceleration,
        spline4_rot3_acceleration,
        spline4_pose2_acceleration,
        spline4_pose3_acceleration,
        spline4_pose2_sensor_acceleration,
        spline4_pose3_sensor_acceleration,
    ]

    for func in derivative_funcs:
        for frame_type in FrameType:
            generate_derivative_func(output_dir, func, frame_type)


def generate_spline5(output_dir: Path):
    value_funcs = [
        spline5_r1_value,
        spline5_r2_value,
        spline5_r3_value,
        spline5_rot2_value,
        spline5_rot3_value,
        spline5_pose2_value,
        spline5_pose3_value,
    ]

    for func in value_funcs:
        generate_value_func(output_dir, func)

    derivative_funcs = [
        spline5_r1_velocity,
        spline5_r2_velocity,
        spline5_r3_velocity,
        spline5_rot2_velocity,
        spline5_rot3_velocity,
        spline5_pose2_velocity,
        spline5_pose3_velocity,
        spline5_pose2_sensor_velocity,
        spline5_pose3_sensor_velocity,

        spline5_r1_acceleration,
        spline5_r2_acceleration,
        spline5_r3_acceleration,
        spline5_rot2_acceleration,
        spline5_rot3_acceleration,
        spline5_pose2_acceleration,
        spline5_pose3_acceleration,
        spline5_pose2_sensor_acceleration,
        spline5_pose3_sensor_acceleration,
    ]

    for func in derivative_funcs:
        for frame_type in FrameType:
            generate_derivative_func(output_dir, func, frame_type)


def generate(output_dir: Path):
    generate_b_spline_matrices(output_dir)
    generate_z_spline_matrices(output_dir)
    generate_spline3(output_dir)
    generate_spline4(output_dir)
    generate_spline5(output_dir)
