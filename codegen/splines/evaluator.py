import symforce.symbolic as sf

from enum import IntEnum

from symforce import typing as T

from codegen.splines import utils


class FrameType(IntEnum):
    GLOBAL = 0
    LOCAL = 1


class SplineType(IntEnum):
    B_SPLINE = 0
    Z_SPLINE = 1


class ReturnType(IntEnum):
    VALUE = 0
    VELOCITY = 1
    ACCELERATION = 2


def get_spline_type_str(spline_type: SplineType):
    match spline_type:
        case SplineType.B_SPLINE:
            return 'b'
        case SplineType.Z_SPLINE:
            return 'z'


def get_frame_type_str(frame_type: FrameType):
    match frame_type:
        case FrameType.GLOBAL:
            return 'global'
        case FrameType.LOCAL:
            return 'local'


def _evaluate_spline(frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M, vals: T.List[T.Any],
                     epsilon: sf.Scalar) -> T.Any:
    assert lambdas.shape == (len(vals) - 1, 3,)

    value_type = type(vals[0])
    if frame_type == FrameType.GLOBAL:
        val = value_type()
        vel = sf.Matrix(val.tangent_dim(), 1)
        acc = sf.Matrix(val.tangent_dim(), 1)

        for i in reversed(range(lambdas.rows)):
            s_i = lambdas[i, 0]
            v_i = lambdas[i, 1]
            a_i = lambdas[i, 2]
            match value_type:
                case sf.Rot2:
                    d_i = sf.V1.from_flat_list(vals[i].local_coordinates(vals[i + 1], epsilon))
                    val_i = sf.Rot2.from_tangent(s_i * d_i, epsilon)
                    val = val_i * val
                    vel += v_i * d_i
                    acc += a_i * d_i
                case sf.Rot3:
                    d_i = sf.V3.from_flat_list(vals[i].local_coordinates(vals[i + 1], epsilon))
                    val_i = sf.Rot3.from_tangent(s_i * d_i, epsilon)
                    val = val_i * val
                    vel = val_i * vel
                    acc = vel.cross(-v_i * d_i) + val_i * acc + a_i * d_i
                    vel += v_i * d_i
                case sf.Pose2:
                    d_i = sf.V3.from_flat_list(vals[i].local_coordinates(vals[i + 1], epsilon))
                    val_i = sf.Pose2.from_tangent(s_i * d_i, epsilon)
                    val = sf.Pose2(val_i.R * val.R, val_i.t + val.t)
                    vel[:1] += v_i * d_i[:1]
                    vel[1:] += v_i * d_i[1:]
                    acc[:1] += a_i * d_i[:1]
                    acc[1:] += a_i * d_i[1:]
                case sf.Pose3:
                    d_i = sf.V6.from_flat_list(vals[i].local_coordinates(vals[i + 1], epsilon))
                    val_i = sf.Pose3.from_tangent(s_i * d_i, epsilon)
                    val = sf.Pose3(val_i.R * val.R, val_i.t + val.t)
                    vel[:3] = val_i.R * vel[:3]
                    acc[:3] = vel[:3].cross(-v_i * d_i[:3]) + val_i.R * acc[:3] + a_i * d_i[:3]
                    acc[3:] += a_i * d_i[3:]
                    vel[:3] += v_i * d_i[:3]
                    vel[3:] += v_i * d_i[3:]
                case _:
                    d_i = vals[i + 1] - vals[i]
                    val += s_i * d_i
                    vel += v_i * d_i
                    acc += a_i * d_i

        match value_type:
            case sf.Rot2:
                val = vals[0] * val
            case sf.Rot3:
                val = vals[0] * val
                vel = vals[0] * vel
                acc = vals[0] * acc
            case sf.Pose2:
                val = sf.Pose2(vals[0].R * val.R, vals[0].t + val.t)
                acc[1:] -= utils.hat_rot2(vel[:1]) * vel[1:] + utils.hat_rot2(acc[:1]) * val.t
                vel[1:] -= utils.hat_rot2(vel[:1]) * val.t
            case sf.Pose3:
                val = sf.Pose3(vals[0].R * val.R, vals[0].t + val.t)
                vel[:3] = vals[0].R * vel[:3]
                acc[:3] = vals[0].R * acc[:3]
                acc[3:] -= utils.hat_rot3(vel[:3]) * vel[3:] + utils.hat_rot3(acc[:3]) * val.t
                vel[3:] -= utils.hat_rot3(vel[:3]) * val.t
            case _:
                val = vals[0] + val
    else:
        val = vals[0]
        vel = sf.Matrix(val.tangent_dim(), 1)
        acc = sf.Matrix(val.tangent_dim(), 1)

        for i in range(lambdas.rows):
            s_i = lambdas[i, 0]
            v_i = lambdas[i, 1]
            a_i = lambdas[i, 2]
            match value_type:
                case sf.Rot2:
                    d_i = sf.V1.from_flat_list(vals[i].local_coordinates(vals[i + 1], epsilon))
                    val_i = sf.Rot2.from_tangent(s_i * d_i, epsilon)
                    val = val.compose(val_i)
                    vel += v_i * d_i
                    acc += a_i * d_i
                case sf.Rot3:
                    d_i = sf.V3.from_flat_list(vals[i].local_coordinates(vals[i + 1], epsilon))
                    val_i = sf.Rot3.from_tangent(s_i * d_i, epsilon)
                    inv_i = val_i.inverse()
                    val = val.compose(val_i)
                    vel = inv_i * vel
                    acc = vel.cross(v_i * d_i) + inv_i * acc + a_i * d_i
                    vel += v_i * d_i
                case sf.Pose2:
                    d_i = sf.V3.from_flat_list(vals[i].local_coordinates(vals[i + 1], epsilon))
                    val_i = sf.Pose2.from_tangent(s_i * d_i, epsilon)
                    val = sf.Pose2(val.R.compose(val_i.R), val.t + val_i.t)
                    vel[:1] += v_i * d_i[:1]
                    vel[1:] += v_i * d_i[1:]
                    acc[:1] += a_i * d_i[:1]
                    acc[1:] += a_i * d_i[1:]
                case sf.Pose3:
                    d_i = sf.V6.from_flat_list(vals[i].local_coordinates(vals[i + 1], epsilon))
                    val_i = sf.Pose3.from_tangent(s_i * d_i, epsilon)
                    inv_i = val_i.R.inverse()
                    val = sf.Pose3(val.R.compose(val_i.R), val.t + val_i.t)
                    vel[:3] = inv_i * vel[:3]
                    acc[:3] = vel[:3].cross(v_i * d_i[:3]) + inv_i * acc[:3] + a_i * d_i[:3]
                    acc[3:] += a_i * d_i[3:]
                    vel[:3] += v_i * d_i[:3]
                    vel[3:] += v_i * d_i[3:]
                case _:
                    d_i = vals[i + 1] - vals[i]
                    val += s_i * d_i
                    vel += v_i * d_i
                    acc += a_i * d_i

        match value_type:
            case sf.Pose2:
                inv = val.R.inverse()
                vel[1:] = inv * vel[1:]
                acc[1:] = inv * acc[1:] - utils.hat_rot2(vel[:1]) * vel[1:]
            case sf.Pose3:
                inv = val.R.inverse()
                vel[3:] = inv * vel[3:]
                acc[3:] = inv * acc[3:] - utils.hat_rot3(vel[:3]) * vel[3:]
            case _:
                pass

    return val, vel / dt, acc / (dt * dt)


def spline_value(lambdas: sf.M, vals: T.List[T.Any], epsilon: sf.Scalar) -> T.Any:
    num_rows = len(vals) - 1
    assert lambdas.shape == (num_rows, 1,)
    lambdas = lambdas.row_join(sf.M.zeros(num_rows, 2))  # Zero padding.
    val, _, _ = _evaluate_spline(FrameType.GLOBAL, 1, lambdas, vals, epsilon)  # Independent of dt.
    return val


def spline_velocity(frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M, vals: T.List[T.Any],
                    epsilon: sf.Scalar) -> T.Any:
    num_rows = len(vals) - 1
    assert lambdas.shape == (num_rows, 2,)
    lambdas = lambdas.row_join(sf.M.zeros(num_rows, 1))  # Zero padding.
    _, vel, _ = _evaluate_spline(frame_type, dt, lambdas, vals, epsilon)
    return vel


def spline_acceleration(frame_type: FrameType, dt: sf.Scalar, lambdas: sf.M, vals: T.List[T.Any],
                        epsilon: sf.Scalar) -> T.Any:
    num_rows = len(vals) - 1
    assert lambdas.shape == (num_rows, 3,)
    # lambdas = lambdas.row_join(sf.M.zeros(num_rows, 0))  # Zero padding.
    _, _, acc = _evaluate_spline(frame_type, dt, lambdas, vals, epsilon)
    return acc
