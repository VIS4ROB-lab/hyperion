import codegen.cost_functions.patches as patches

from functools import partial
from multiprocessing import Pool
from pathlib import Path
from symforce.codegen import Codegen

from codegen.cost_functions.cost_function import CostFunction
from codegen.factors.between import *
from codegen.factors.spline3 import *
from codegen.factors.spline4 import *
from codegen.factors.spline5 import *


def get_standard_cost_functions(namespace: str) -> list[CostFunction]:
    # Arguments.
    delta_args = {'sqrt_info': 'sqrtInfo'}
    prior_args = {'y': 'prior', 'sqrt_info': 'sqrtInfo'}

    between_args = {'x_T_y': 'xTy', 'sqrt_info': 'sqrtInfo'}
    sensor_between_args = {'s_T_y': 'sTy', 'sqrt_info': 'sqrtInfo'}
    sensor_relative_between_args = {'a_T_b': 'aTb', 'sqrt_info': 'sqrtInfo'}

    distance_args = {'x_d_y': 'xdy', 'sqrt_info': 'sqrtInfo'}
    sensor_distance_args = {'s_d_y': 'sdy', 'sqrt_info': 'sqrtInfo'}
    sensor_relative_distance_args = {'a_d_b': 'adb', 'sqrt_info': 'sqrtInfo'}

    visual_args = {'pixel': 'pixel', 'sqrt_info': 'sqrtInfo'}
    specialized_visual_args = ['b_T_c', 'calibration']

    # Cost functions.
    cost_functions: list[CostFunction] = [
        # Delta factors.
        CostFunction(r1_delta_factor, namespace, member_args=delta_args),
        CostFunction(r2_delta_factor, namespace, member_args=delta_args),
        CostFunction(r3_delta_factor, namespace, member_args=delta_args),
        CostFunction(rot2_delta_factor, namespace, member_args=delta_args),
        CostFunction(rot3_delta_factor, namespace, member_args=delta_args),
        CostFunction(pose2_delta_factor, namespace, member_args=delta_args),
        CostFunction(pose3_delta_factor, namespace, member_args=delta_args),
        CostFunction(r2_angular_delta_factor, namespace, member_args=delta_args),
        CostFunction(r3_angular_delta_factor, namespace, member_args=delta_args),

        # Prior factors.
        CostFunction(r1_delta_factor, namespace, custom_name='r1_prior', member_args=prior_args),
        CostFunction(r2_delta_factor, namespace, custom_name='r2_prior', member_args=prior_args),
        CostFunction(r3_delta_factor, namespace, custom_name='r3_prior', member_args=prior_args),
        CostFunction(rot2_delta_factor, namespace, custom_name='rot2_prior', member_args=prior_args),
        CostFunction(rot3_delta_factor, namespace, custom_name='rot3_prior', member_args=prior_args),
        CostFunction(pose2_delta_factor, namespace, custom_name='pose2_prior', member_args=prior_args),
        CostFunction(pose3_delta_factor, namespace, custom_name='pose3_prior', member_args=prior_args),
        CostFunction(r2_angular_delta_factor, namespace, custom_name='r2_angular_prior', member_args=prior_args),
        CostFunction(r3_angular_delta_factor, namespace, custom_name='r3_angular_prior', member_args=prior_args),

        # Between factors.
        CostFunction(r1_between_factor, namespace, member_args=between_args),
        CostFunction(r2_between_factor, namespace, member_args=between_args),
        CostFunction(r3_between_factor, namespace, member_args=between_args),
        CostFunction(rot2_between_factor, namespace, member_args=between_args),
        CostFunction(rot3_between_factor, namespace, member_args=between_args),
        CostFunction(pose2_between_factor, namespace, member_args=between_args),
        CostFunction(pose3_between_factor, namespace, member_args=between_args),

        # Distance factors.
        CostFunction(r1_euclidean_distance_factor, namespace, member_args=distance_args),
        CostFunction(r2_euclidean_distance_factor, namespace, member_args=distance_args),
        CostFunction(r3_euclidean_distance_factor, namespace, member_args=distance_args),
        CostFunction(r2_angular_distance_factor, namespace, member_args=distance_args),
        CostFunction(r3_angular_distance_factor, namespace, member_args=distance_args),
        CostFunction(rot2_angular_distance_factor, namespace, member_args=distance_args),
        CostFunction(rot3_angular_distance_factor, namespace, member_args=distance_args),

        # Sensor prior factors.
        CostFunction(pose2_sensor_delta_factor, namespace, custom_name='pose2_sensor_prior',
                     member_args=prior_args),
        CostFunction(pose2_r2_sensor_delta_factor, namespace, custom_name='pose2_r2_sensor_prior',
                     member_args=prior_args),
        CostFunction(pose2_rot2_sensor_delta_factor, namespace, custom_name='pose2_rot2_sensor_prior',
                     member_args=prior_args),
        CostFunction(pose3_sensor_delta_factor, namespace, custom_name='pose3_sensor_prior',
                     member_args=prior_args),
        CostFunction(pose3_r3_sensor_delta_factor, namespace, custom_name='pose3_r3_sensor_prior',
                     member_args=prior_args),
        CostFunction(pose3_rot3_sensor_delta_factor, namespace, custom_name='pose3_rot3_sensor_prior',
                     member_args=prior_args),

        # Sensor between factors.
        CostFunction(pose2_sensor_between_factor, namespace, member_args=sensor_between_args),
        CostFunction(pose2_r2_sensor_between_factor, namespace, member_args=sensor_between_args),
        CostFunction(pose2_rot2_sensor_between_factor, namespace, member_args=sensor_between_args),
        CostFunction(pose3_sensor_between_factor, namespace, member_args=sensor_between_args),
        CostFunction(pose3_r3_sensor_between_factor, namespace, member_args=sensor_between_args),
        CostFunction(pose3_rot3_sensor_between_factor, namespace, member_args=sensor_between_args),
        CostFunction(pose2_sensor_relative_between_factor, namespace, member_args=sensor_relative_between_args),
        CostFunction(pose2_r2_sensor_relative_between_factor, namespace, member_args=sensor_relative_between_args),
        CostFunction(pose2_rot2_sensor_relative_between_factor, namespace, member_args=sensor_relative_between_args),
        CostFunction(pose3_sensor_relative_between_factor, namespace, member_args=sensor_relative_between_args),
        CostFunction(pose3_r3_sensor_relative_between_factor, namespace, member_args=sensor_relative_between_args),
        CostFunction(pose3_rot3_sensor_relative_between_factor, namespace, member_args=sensor_relative_between_args),

        # Sensor distance factors.
        CostFunction(pose2_sensor_euclidean_distance_factor, namespace,
                     member_args=sensor_distance_args),
        CostFunction(pose3_sensor_euclidean_distance_factor, namespace,
                     member_args=sensor_distance_args),
        CostFunction(pose2_sensor_relative_euclidean_distance_factor, namespace,
                     member_args=sensor_relative_distance_args),
        CostFunction(pose3_sensor_relative_euclidean_distance_factor, namespace,
                     member_args=sensor_relative_distance_args),
        CostFunction(pose2_sensor_angle_between_factor, namespace,
                     member_args=sensor_distance_args),
        CostFunction(pose3_sensor_angle_between_factor, namespace,
                     member_args=sensor_distance_args),
        CostFunction(pose2_sensor_relative_angle_between_factor, namespace,
                     member_args=sensor_relative_distance_args),
        CostFunction(pose3_sensor_relative_angle_between_factor, namespace,
                     member_args=sensor_relative_distance_args),

        # Camera factors.
        CostFunction(pose3_camera_linear_reprojection_factor, namespace, member_args=visual_args,
                     specialized_args=specialized_visual_args),
        CostFunction(pose3_camera_linear_bearing_factor, namespace, member_args=visual_args,
                     specialized_args=specialized_visual_args),
    ]

    return cost_functions


def get_spline3_cost_functions(namespace: str) -> list[CostFunction]:
    # Arguments.
    delta_args = {'lambdas': 'lambdas', 'sqrt_info': 'sqrtInfo'}
    prior_args = {'lambdas': 'lambdas', 'y': 'prior', 'sqrt_info': 'sqrtInfo'}
    between_args = {'lambdas': 'lambdas', 'x_T_y': 'xTy', 'sqrt_info': 'sqrtInfo'}
    sensor_between_args = {'lambdas': 'lambdas', 's_T_y': 'sTy', 'sqrt_info': 'sqrtInfo'}
    sensor_relative_between_args = {'lambdas_x': 'lambdas_x', 'lambdas_y': 'lambdas_y', 'a_T_b': 'aTb',
                                    'sqrt_info': 'sqrtInfo'}
    visual_args = {'lambdas': 'lambdas', 'pixel': 'pixel', 'sqrt_info': 'sqrtInfo'}
    velocity_args = {'dt': 'dt', 'lambdas': 'lambdas', 'velocity': 'velocity', 'sqrt_info': 'sqrtInfo'}
    acceleration_args = {'dt': 'dt', 'lambdas': 'lambdas', 'acceleration': 'acceleration', 'sqrt_info': 'sqrtInfo'}

    # Specializations.
    specialized_reprojection_args = ['b_T_c', 'calibration']

    # Cost functions.
    cost_functions: list[CostFunction] = [
        # Prior factors.
        CostFunction(spline3_r3_delta_factor, namespace, custom_name='spline3_r3_prior',
                     member_args=prior_args),
        CostFunction(spline3_rot2_delta_factor, namespace, custom_name='spline3_rot2_prior',
                     member_args=prior_args),
        CostFunction(spline3_rot3_delta_factor, namespace, custom_name='spline3_rot3_prior',
                     member_args=prior_args),
        CostFunction(spline3_pose2_delta_factor, namespace, custom_name='spline3_pose2_prior',
                     member_args=prior_args),
        CostFunction(spline3_pose2_r2_delta_factor, namespace, custom_name='spline3_pose2_r2_prior',
                     member_args=prior_args),
        CostFunction(spline3_pose2_rot2_delta_factor, namespace, custom_name='spline3_pose2_rot2_prior',
                     member_args=prior_args),
        CostFunction(spline3_pose3_delta_factor, namespace, custom_name='spline3_pose3_prior',
                     member_args=prior_args),
        CostFunction(spline3_pose3_r3_delta_factor, namespace, custom_name='spline3_pose3_r3_prior',
                     member_args=prior_args),
        CostFunction(spline3_pose3_rot3_delta_factor, namespace, custom_name='spline3_pose3_rot3_prior',
                     member_args=prior_args),

        # Sensor prior factors.
        CostFunction(spline3_pose2_sensor_delta_factor, namespace, custom_name='spline3_pose2_sensor_prior',
                     member_args=prior_args),
        CostFunction(spline3_pose2_r2_sensor_delta_factor, namespace, custom_name='spline3_pose2_r2_sensor_prior',
                     member_args=prior_args),
        CostFunction(spline3_pose2_rot2_sensor_delta_factor, namespace, custom_name='spline3_pose2_rot2_sensor_prior',
                     member_args=prior_args),
        CostFunction(spline3_pose3_sensor_delta_factor, namespace, custom_name='spline3_pose3_sensor_prior',
                     member_args=prior_args),
        CostFunction(spline3_pose3_r3_sensor_delta_factor, namespace, custom_name='spline3_pose3_r3_sensor_prior',
                     member_args=prior_args),
        CostFunction(spline3_pose3_rot3_sensor_delta_factor, namespace, custom_name='spline3_pose3_rot3_sensor_prior',
                     member_args=prior_args),

        # Sensor between factors.
        CostFunction(spline3_pose2_sensor_between_factor, namespace,
                     member_args=sensor_between_args),
        CostFunction(spline3_pose2_r2_sensor_between_factor, namespace,
                     member_args=sensor_between_args),
        CostFunction(spline3_pose2_rot2_sensor_between_factor, namespace,
                     member_args=sensor_between_args),
        CostFunction(spline3_pose3_sensor_between_factor, namespace,
                     member_args=sensor_between_args),
        CostFunction(spline3_pose3_r3_sensor_between_factor, namespace,
                     member_args=sensor_between_args),
        CostFunction(spline3_pose3_rot3_sensor_between_factor, namespace,
                     member_args=sensor_between_args),
        CostFunction(spline3_pose2_sensor_relative_between_factor, namespace,
                     member_args=sensor_relative_between_args),
        CostFunction(spline3_pose2_r2_sensor_relative_between_factor, namespace,
                     member_args=sensor_relative_between_args),
        CostFunction(spline3_pose2_rot2_sensor_relative_between_factor, namespace,
                     member_args=sensor_relative_between_args),
        CostFunction(spline3_pose3_sensor_relative_between_factor, namespace,
                     member_args=sensor_relative_between_args),
        CostFunction(spline3_pose3_r3_sensor_relative_between_factor, namespace,
                     member_args=sensor_relative_between_args),
        CostFunction(spline3_pose3_rot3_sensor_relative_between_factor, namespace,
                     member_args=sensor_relative_between_args),

        # Camera factors.
        CostFunction(spline3_pose3_camera_linear_reprojection_factor, namespace,
                     member_args=visual_args, specialized_args=specialized_reprojection_args),
        CostFunction(spline3_pose3_camera_linear_bearing_factor, namespace,
                     member_args=visual_args, specialized_args=specialized_reprojection_args),
    ]

    for frame_type in FrameType:
        cost_functions.extend([
            # Velocity factors.
            CostFunction(partial(spline3_r3_velocity_factor, frame_type), namespace,
                         member_args=velocity_args, suffix=get_frame_type_str(frame_type)),
            CostFunction(partial(spline3_rot2_velocity_factor, frame_type), namespace,
                         member_args=velocity_args, suffix=get_frame_type_str(frame_type)),
            CostFunction(partial(spline3_rot3_velocity_factor, frame_type), namespace,
                         member_args=velocity_args, suffix=get_frame_type_str(frame_type)),
            CostFunction(partial(spline3_pose2_velocity_factor, frame_type), namespace,
                         member_args=velocity_args, suffix=get_frame_type_str(frame_type)),
            CostFunction(partial(spline3_pose3_velocity_factor, frame_type), namespace,
                         member_args=velocity_args, suffix=get_frame_type_str(frame_type)),

            # Sensor velocity factors.
            CostFunction(partial(spline3_pose2_sensor_velocity_factor, frame_type), namespace,
                         member_args=velocity_args, suffix=get_frame_type_str(frame_type)),
            CostFunction(partial(spline3_pose3_sensor_velocity_factor, frame_type), namespace,
                         member_args=velocity_args, suffix=get_frame_type_str(frame_type)),

            # Acceleration factors.
            CostFunction(partial(spline3_r3_acceleration_factor, frame_type), namespace,
                         member_args=acceleration_args, suffix=get_frame_type_str(frame_type)),
            CostFunction(partial(spline3_rot2_acceleration_factor, frame_type), namespace,
                         member_args=acceleration_args, suffix=get_frame_type_str(frame_type)),
            CostFunction(partial(spline3_rot3_acceleration_factor, frame_type), namespace,
                         member_args=acceleration_args, suffix=get_frame_type_str(frame_type)),
            CostFunction(partial(spline3_pose2_acceleration_factor, frame_type), namespace,
                         member_args=acceleration_args, suffix=get_frame_type_str(frame_type)),
            CostFunction(partial(spline3_pose3_acceleration_factor, frame_type), namespace,
                         member_args=acceleration_args, suffix=get_frame_type_str(frame_type)),

            # Sensor acceleration factors.
            CostFunction(partial(spline3_pose2_sensor_acceleration_factor, frame_type), namespace,
                         member_args=acceleration_args, suffix=get_frame_type_str(frame_type)),
            CostFunction(partial(spline3_pose3_sensor_acceleration_factor, frame_type), namespace,
                         member_args=acceleration_args, suffix=get_frame_type_str(frame_type)),
        ])

    return cost_functions


def get_spline4_cost_functions(namespace: str) -> list[CostFunction]:
    prior_args = {'lambdas': 'lambdas', 'y': 'prior', 'sqrt_info': 'sqrtInfo'}
    velocity_args = {'dt': 'dt', 'lambdas': 'lambdas', 'velocity': 'velocity', 'sqrt_info': 'sqrtInfo'}
    acceleration_args = {'dt': 'dt', 'lambdas': 'lambdas', 'acceleration': 'acceleration', 'sqrt_info': 'sqrtInfo'}

    cost_functions: list[CostFunction] = [
        # Prior factors.
        CostFunction(spline4_rot3_delta_factor, namespace, custom_name='spline4_rot3_prior',
                     member_args=prior_args),
        CostFunction(spline4_pose3_delta_factor, namespace, custom_name='spline4_pose3_prior',
                     member_args=prior_args),
    ]

    for frame_type in FrameType:
        cost_functions.extend([
            # Velocity factors.
            CostFunction(partial(spline4_rot3_velocity_factor, frame_type), namespace,
                         member_args=velocity_args, suffix=get_frame_type_str(frame_type)),
            CostFunction(partial(spline4_pose3_velocity_factor, frame_type), namespace,
                         member_args=velocity_args, suffix=get_frame_type_str(frame_type)),

            # Acceleration factors.
            CostFunction(partial(spline4_rot3_acceleration_factor, frame_type), namespace,
                         member_args=acceleration_args, suffix=get_frame_type_str(frame_type)),
            CostFunction(partial(spline4_pose3_acceleration_factor, frame_type), namespace,
                         member_args=acceleration_args, suffix=get_frame_type_str(frame_type)),
        ])

    return cost_functions


def get_spline5_cost_functions(namespace: str) -> list[CostFunction]:
    prior_args = {'lambdas': 'lambdas', 'y': 'prior', 'sqrt_info': 'sqrtInfo'}
    velocity_args = {'dt': 'dt', 'lambdas': 'lambdas', 'velocity': 'velocity', 'sqrt_info': 'sqrtInfo'}
    acceleration_args = {'dt': 'dt', 'lambdas': 'lambdas', 'acceleration': 'acceleration', 'sqrt_info': 'sqrtInfo'}

    cost_functions: list[CostFunction] = [
        # Prior factors.
        CostFunction(spline5_rot3_delta_factor, namespace, custom_name='spline5_rot3_prior',
                     member_args=prior_args),
        CostFunction(spline5_pose3_delta_factor, namespace, custom_name='spline5_pose3_prior',
                     member_args=prior_args),
    ]

    for frame_type in FrameType:
        cost_functions.extend([
            # Velocity factors.
            CostFunction(partial(spline5_rot3_velocity_factor, frame_type), namespace,
                         member_args=velocity_args, suffix=get_frame_type_str(frame_type)),
            CostFunction(partial(spline5_pose3_velocity_factor, frame_type), namespace,
                         member_args=velocity_args, suffix=get_frame_type_str(frame_type)),

            # Acceleration factors.
            CostFunction(partial(spline5_rot3_acceleration_factor, frame_type), namespace,
                         member_args=acceleration_args, suffix=get_frame_type_str(frame_type)),
            CostFunction(partial(spline5_pose3_acceleration_factor, frame_type), namespace,
                         member_args=acceleration_args, suffix=get_frame_type_str(frame_type)),
        ])

    return cost_functions


def get_cost_functions() -> list[CostFunction]:
    namespace = 'hyperion'
    return get_standard_cost_functions(namespace) + \
        get_spline3_cost_functions(namespace) + \
        get_spline4_cost_functions(namespace) + \
        get_spline5_cost_functions(namespace)


def generate_ceres(output_dir: Path):
    # Do monkey patch.
    Codegen.original_with_jacobians = Codegen.with_jacobians
    Codegen.with_jacobians = patches.codegen_with_ceres_jacobians_patch

    # Generate Ceres cost functions.
    with Pool(processes=8) as pool:
        pool.starmap(CostFunction.generate,
                     [(cost_function, output_dir, True) for cost_function in get_cost_functions()])

    # Undo monkey patch.
    Codegen.with_jacobians = Codegen.original_with_jacobians


def generate_hyperion(output_dir: Path):
    # Generate Hyperion cost functions.
    with Pool(processes=8) as pool:
        pool.starmap(CostFunction.generate,
                     [(cost_function, output_dir, False) for cost_function in get_cost_functions()])
