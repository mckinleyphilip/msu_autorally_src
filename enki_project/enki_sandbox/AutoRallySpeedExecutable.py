# -*- coding: utf-8 -*-

import argparse
import matplotlib.pyplot as plt
import numpy as np
import os
import pprint
import sys
from enki.core.io import ArchiveFileReader
from enki.core.interface import EnkiEvoROSExecutable
from sklearn.metrics import mean_squared_error

DEFAULT_IP_ADDRESS = '35.9.26.204 '
DEFAULT_SENDER_PORT = 5023
DEFAULT_RECEIVER_PORT = 5033

PID_SETTINGS = [0.2, 0.0, 0.001, 0.15]
MIN_SPEED = 0.0
MAX_SPEED = 10.0
TIME_START = 0.0
TIME_END = 60.0
TIME_STEP = 0.1


class AutoRallySpeedExecutable(EnkiEvoROSExecutable):
    """AutoRallySpeedExecutable class

    For conducting experiments with variable speed signals on an AutoRally platform with a fixed PID setting.
    """
    @property
    def input_parameters(self):
        """Defines the boundaries the executable inputs.

        :return: the parameter definition of an encoded input
        """
        return {
            'speed_f1_type': ['linear', 'cosine'],
            'speed_f1_min': [MIN_SPEED, MAX_SPEED],
            'speed_f1_max': [MIN_SPEED, MAX_SPEED],
            'speed_f2_type': ['linear', 'cosine'],
            'speed_f2_min': [MIN_SPEED, MAX_SPEED],
            'speed_f2_max': [MIN_SPEED, MAX_SPEED],
            'speed_f3_type': ['linear', 'cosine'],
            'speed_f3_min': [MIN_SPEED, MAX_SPEED],
            'speed_f3_max': [MIN_SPEED, MAX_SPEED],
            'speed_f4_type': ['linear', 'cosine'],
            'speed_f4_min': [MIN_SPEED, MAX_SPEED],
            'speed_f4_max': [MIN_SPEED, MAX_SPEED],
            'speed_f5_type': ['linear', 'cosine'],
            'speed_f5_min': [MIN_SPEED, MAX_SPEED],
            'speed_f5_max': [MIN_SPEED, MAX_SPEED],
            'speed_f6_type': ['linear', 'cosine'],
            'speed_f6_min': [MIN_SPEED, MAX_SPEED],
            'speed_f6_max': [MIN_SPEED, MAX_SPEED]
        }

    @property
    def output_parameters(self):
        """Defines the boundaries of an the observed system behavior.

        :return: the parameter definition of an encoded system behavior
        """
        return {
            'error': [0.0, 10.0]
        }

    def __init__(self):
        """Initializes an instance of the executable.

        Uses the base EnkiEvoROSExecutable class to execute the individual on a remote system running EvoROS.
        """
        super().__init__(ip_address=DEFAULT_IP_ADDRESS, sender_port=DEFAULT_SENDER_PORT, receiver_port=DEFAULT_RECEIVER_PORT)

    def convert_to_evoros_input(self, enki_input):
        """Converts an executable input from Enki into a format for EvoROS.

        :param enki_input: an input from Enki for execution
        :return: an input for EvoROS for execution
        """
        # create and sample piecewise speed function
        f_segments = [
            [enki_input['speed_f1_type'], enki_input['speed_f1_min'], enki_input['speed_f1_max']],
            [enki_input['speed_f2_type'], enki_input['speed_f2_min'], enki_input['speed_f2_max']],
            [enki_input['speed_f3_type'], enki_input['speed_f3_min'], enki_input['speed_f3_max']],
            [enki_input['speed_f4_type'], enki_input['speed_f4_min'], enki_input['speed_f4_max']],
            [enki_input['speed_f5_type'], enki_input['speed_f5_min'], enki_input['speed_f5_max']],
            [enki_input['speed_f6_type'], enki_input['speed_f6_min'], enki_input['speed_f6_max']],
        ]
        speed = sample_piecewise_function(f_segments, TIME_START, TIME_END, TIME_STEP)

        # convert to EvoROS input
        evoros_input = {
            'genome': PID_SETTINGS,
            'enki_genome': speed
        }
        return evoros_input

    def convert_from_evoros_result(self, evoros_result):
        """Converts an execution result from EvoROS into a format for Enki.

        :param evoros_result: a result from an EvoROS execution
        :return: a result for Enki
        """
        # get actual speed from EvoROS
        actual_speed = np.array(evoros_result['Actual Speed'])

        # get goal speed from EvoROS
        goal_speed = np.array(evoros_result['Goal Speed'])

        # compute the error
        error = np.abs(actual_speed - goal_speed)

        # convert to Enki result
        enki_result = {
            'actual_speed': actual_speed,
            'goal_speed': goal_speed,
            'error': error
        }
        return enki_result


def sample_piecewise_function(f_segments, t_start, t_end, t_step):
    """Samples values from a piecewise function formed from the given specification.

    :param f_segments: specifies segments of the piecewise function
    :param t_start: the start time for sampling
    :param t_end: the end time for sampling
    :param t_step: the step size for sampling
    :return: a list of sampled values
    """
    result = list()

    # determine the length of each piecewise segment
    segment_length = (t_end - t_start) / len(f_segments)

    # iterate and sample values for each time step
    t = t_start
    while t < t_end:
        # determine which piecewise segment applies
        i = int(t / segment_length)

        # get the specification for the current piecewise segment
        (f_type, f_min, f_max) = f_segments[i]

        # determine the bounds of the piecewise segment
        t_min = i * segment_length
        t_max = i * segment_length + segment_length

        # compute the current value from the piecewise segment
        if f_type is 'linear':
            # use a linear function
            x = f_max + (t - t_max) * (f_max - f_min) / (t_max - t_min)
        elif f_type is 'cosine':
            # use a cosine function
            amp = np.abs(f_max - f_min) / 2
            h_shift = t_min
            v_shift = amp + min(f_min, f_max)
            x = amp * np.cos((t + h_shift) * 2.0 * np.pi / segment_length) + v_shift
        else:
            # default value is zero
            x = 0.0

        # append the value to the result and go to the next time step
        result.append(x)
        t += t_step

    return result


def main(args):
    """

    :param args:
    :return:
    """
    print('Enki Archive Plotter')

    if not os.path.exists(args.archive_path):
        print('ERROR: Invalid archive path.')
        sys.exit(1)
    else:
        # read archive object from file
        reader = ArchiveFileReader(args.archive_path)
        interface = reader.read_interface()
        archive = reader.read(interface)

        # check if archive is empty
        if len(archive.generations) == 0:
            print('ERROR: Archive is empty.')
            sys.exit(1)
        else:
            pp = pprint.PrettyPrinter(indent=3)

            # create output path
            if not os.path.exists(args.output_path):
                print('Creating output directory {}...'.format(args.output_path))
                os.makedirs(args.output_path)

            # get individuals from the last generation in the archive and iterate through each
            individuals = archive.generations[0]
            for i in range(len(individuals)):
                # get input from individual
                try:
                    print('Reading individual input values...')
                    enki_input = individuals[i].genome.values
                    pp.pprint(enki_input)

                    print('Converting to speed signal...')
                    f_segments = [
                        [enki_input['speed_f1_type'], enki_input['speed_f1_min'], enki_input['speed_f1_max']],
                        [enki_input['speed_f2_type'], enki_input['speed_f2_min'], enki_input['speed_f2_max']],
                        [enki_input['speed_f3_type'], enki_input['speed_f3_min'], enki_input['speed_f3_max']],
                        [enki_input['speed_f4_type'], enki_input['speed_f4_min'], enki_input['speed_f4_max']],
                        [enki_input['speed_f5_type'], enki_input['speed_f5_min'], enki_input['speed_f5_max']],
                        [enki_input['speed_f6_type'], enki_input['speed_f6_min'], enki_input['speed_f6_max']],
                    ]
                    input_speed = sample_piecewise_function(f_segments, TIME_START, TIME_END, TIME_STEP)
                except KeyError:
                    print('ERROR: Incompatible archive file.')
                    sys.exit(1)

                # get output from individual
                try:
                    print('Reading individual output values...')
                    enki_output = individuals[i].phenome.values

                    actual_speed = None
                    if 'actual_speed' in enki_output:
                        actual_speed = enki_output['actual_speed']

                    goal_speed = None
                    if 'goal_speed' in enki_output:
                        goal_speed = enki_output['goal_speed']

                    error = None
                    if 'error' in enki_output:
                        error = enki_output['error']
                except KeyError:
                    print('ERROR: Incompatible archive file.')
                    sys.exit(1)

                # plot the results
                file_path = os.path.join(args.output_path, '{:03d}.jpg'.format(i))
                print('Plotting to {}...'.format(file_path))
                fig = plt.figure(figsize=(6.4, 4.8), dpi=100)
                ax = fig.gca()
                if actual_speed is not None and goal_speed is not None:
                    mse = mean_squared_error(actual_speed, goal_speed)
                    ax.set_title('Individual {:03d}, MSE={:0.4f}'.format(i, mse))
                else:
                    ax.set_title('Individual {:03d}'.format(i))
                ax.set_ylim(MIN_SPEED, MAX_SPEED)
                if actual_speed is not None:
                    ax.plot(np.arange(len(actual_speed)), actual_speed, label='Actual Speed')
                if goal_speed is not None:
                    ax.plot(np.arange(len(goal_speed)), goal_speed, label='Goal Speed')
                if error is not None:
                    ax.plot(np.arange(len(error)), error, label='Error')
                ax.legend()
                fig.savefig(file_path)
                plt.close(fig)


def read_commandline():
    """Reads command-line arguments.

    :return: command-line arguments
    """
    parser = argparse.ArgumentParser(
        description='Generates plots from Enki data.'
    )
    parser.add_argument(
        'archive_path',
        help='File path to an archive file.',
        type=str
    )
    parser.add_argument(
        '--output-path',
        metavar='',
        help='directory to save plots to',
        default=os.getcwd(),
        type=str
    )
    return parser.parse_args()


if __name__ == '__main__':
    main(read_commandline())
