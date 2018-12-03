import numpy as np
import pprint


def compute_ramps(ramp_angles, ramp_length=20.0, x_pos=0.0, y_pos=0.0, z_pos=0.0):
    """ Computes a list of ramp parameters from a set of ramp angles.

    :param ramp_angles: a list of ramp angles about the y-axis
    :param ramp_length: the length of each ramp
    :param x_pos: the x-position for the initial ramp
    :param y_pos: the x-position for the initial ramp
    :param z_pos: the x-position for the initial ramp
    :return: a set of ramp parameters
    """
    n = len(ramp_angles)

    ramp_params = np.zeros((n, 6))
    ramp_params[:, 4] = ramp_angles
    ramp_params[0, :3] = (x_pos, y_pos, z_pos)
    for i in range(1, len(ramp_params)):
        (x, y, z, u, v, w) = ramp_params[i - 1]
        trans_mat1 = np.array([
            [1.0, 0.0, 0.0, ramp_length],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])
        rot_mat = np.array([
            [np.cos(-v),    0.0, -np.sin(-v),   0.0],
            [0.0,           1.0, 0.0,           0.0],
            [np.sin(-v),    0.0, np.cos(-v),    0.0],
            [0.0,           0.0, 0.0,           1.0]
        ])
        trans_mat2 = np.array([
            [1.0, 0.0, 0.0, x],
            [0.0, 1.0, 0.0, y],
            [0.0, 0.0, 1.0, z],
            [0.0, 0.0, 0.0, 1.0]
        ])
        pos1 = np.array((0.0, 0.0, 0.0, 1.0))
        pos2 = np.dot(trans_mat1, pos1)
        pos3 = np.dot(rot_mat, pos2)
        pos4 = np.dot(trans_mat2, pos3)
        ramp_params[i, :3] = pos4[:3]

    return ramp_params

ramp_angles = [0.15, -0.30, 0.15, 0.15, -0.15]
ramp_params = compute_ramps(ramp_angles)

np.set_printoptions(precision=3, suppress=True)
pp = pprint.PrettyPrinter(indent=4)
pp.pprint(ramp_params)
