# -*- coding: utf-8 -*-

import numpy as np
from enki.core.interface import EnkiEvoROSExecutable

PID_SETTINGS = [0.2, 0.0, 0.001, 0.15]
MAX_ANGLE = np.pi / 6


class AutoRallyInclineExecutable(EnkiEvoROSExecutable):
    """AutoRallyInclineExecutable class

    For conducting experiments with variable inclines on an AutoRally platform with a fixed PID setting.
    """
    @property
    def input_definition(self):
        """Defines the boundaries the executable inputs.

        :return: the parameter definition of an encoded input
        """
        return {
            'incline1': [-MAX_ANGLE, MAX_ANGLE],
            'incline2': [-MAX_ANGLE, MAX_ANGLE],
            'incline3': [-MAX_ANGLE, MAX_ANGLE],
            'incline4': [-MAX_ANGLE, MAX_ANGLE],
            'incline5': [-MAX_ANGLE, MAX_ANGLE]
        }

    @property
    def output_definition(self):
        """Defines the boundaries of an the observed system behavior.

        :return: the parameter definition of an encoded system behavior
        """
        return {
            'error': [0.0, 10.0]
        }

    def convert_to_evoros_input(self, enki_input):
        """Converts an executable input from Enki into a format for Evo-ROS.

        :param enki_input: an input from Enki for execution
        :return: an input for Evo-ROS for execution
        """
        # ramp inclines
        ramp_inclines = [
            enki_input['incline1'],
            enki_input['incline2'],
            enki_input['incline3'],
            enki_input['incline4'],
            enki_input['incline5']
        ]

        # convert to Evo-ROS input
        evoros_input = {
            'genome': PID_SETTINGS,
            'enki_genome': ramp_inclines
        }
        return evoros_input

    def convert_from_evoros_result(self, evoros_result):
        """Converts an execution result from Evo-ROS into a format for Enki.

        :param evoros_result: a result from an Evo-ROS execution
        :return: a result for Enki
        """
        error = np.abs(np.array(evoros_result['Actual Speed']) - np.array(evoros_result['Goal Speed']))

        # convert to Enki result
        enki_result = {
            'error': error
        }
        return enki_result


def compute_ramps(ramp_angles, ramp_length=20.0, x_pos=0.0, y_pos=0.0, z_pos=0.0):
    """Computes a list of ramp parameters from a set of ramp angles.

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
            [np.cos(-v),    0.0,    -np.sin(-v),   0.0],
            [0.0,           1.0,    0.0,           0.0],
            [np.sin(-v),    0.0,    np.cos(-v),    0.0],
            [0.0,           0.0,    0.0,           1.0]
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
