# -*- coding: utf-8 -*-

import argparse
import numpy as np
import json
import os
import pickle
import matplotlib.pyplot as plt
from enki.core.errors import EnkiError
from enki.core.errors import EnkiFileError
from enki.core.io import ArchiveFileReader
from enki.sandbox.AutoRallySpeedExecutable import TIME_START
from enki.sandbox.AutoRallySpeedExecutable import TIME_END
from enki.sandbox.AutoRallySpeedExecutable import TIME_STEP
from enki.sandbox.AutoRallySpeedExecutable import MIN_SPEED
from enki.sandbox.AutoRallySpeedExecutable import MAX_SPEED
from enki.sandbox.AutoRallySpeedExecutable import PID_SETTINGS
from enki.sandbox.AutoRallySpeedExecutable import sample_piecewise_function
from sklearn.metrics import mean_squared_error


def main(args):
    """

    :param args:
    :return:
    """
    archive_path = args.archive_path
    output_path = args.output_path
    if archive_path is not None and output_path is None:
        archive_name = os.path.splitext(archive_path)[0]
        output_path = '{}_signals.pickle'.format(archive_name)

    try:
        _process_archive(archive_path, output_path, top=args.top, show_plots=args.plot)
    except EnkiError as e:
        print('ERROR: {}'.format(e))
    except KeyboardInterrupt:
        print('\nAborting.')


def _process_archive(archive_path, output_path, top=None, show_plots=False):
    """

    :param archive_path:
    :param output_path:
    :param top:
    :param show_plots:
    :return:
    """
    if not os.path.exists(archive_path):
        raise EnkiFileError('Invalid location for archive file.')
    else:
        # read archive object from file
        reader = ArchiveFileReader(archive_path)
        interface = reader.read_interface()
        archive = reader.read(interface)

        # check if archive is empty
        if len(archive.generations) == 0:
            raise EnkiFileError('Archive file is empty.')
        else:
            speed_records = list()

            archive_individuals = archive.generations[-1]
            for i in range(len(archive_individuals)):
                # get input and output data from individual
                enki_input = archive_individuals[i].genome.values
                enki_output = archive_individuals[i].phenome.values

                # get metadata from output
                pid_settings = enki_output['pid_settings'] if 'pid_settings' in enki_output else PID_SETTINGS
                time_start = enki_output['time_start'] if 'time_start' in enki_output else TIME_START
                time_end = enki_output['time_end'] if 'time_end' in enki_output else TIME_END
                time_step = enki_output['time_step'] if 'time_step' in enki_output else TIME_STEP
                min_speed = enki_output['min_speed'] if 'min_speed' in enki_output else MIN_SPEED
                max_speed = enki_output['max_speed'] if 'max_speed' in enki_output else MAX_SPEED

                # generate speed signal
                f_segments = [
                    [enki_input['speed_f1_type'], enki_input['speed_f1_min'], enki_input['speed_f1_max']],
                    [enki_input['speed_f2_type'], enki_input['speed_f2_min'], enki_input['speed_f2_max']],
                    [enki_input['speed_f3_type'], enki_input['speed_f3_min'], enki_input['speed_f3_max']],
                    [enki_input['speed_f4_type'], enki_input['speed_f4_min'], enki_input['speed_f4_max']],
                    [enki_input['speed_f5_type'], enki_input['speed_f5_min'], enki_input['speed_f5_max']],
                    [enki_input['speed_f6_type'], enki_input['speed_f6_min'], enki_input['speed_f6_max']],
                ]
                speed = sample_piecewise_function(f_segments, time_start, time_end, time_step)

                # compute mse
                mse = None
                if 'actual_speed' in enki_output and 'goal_speed' in enki_output:
                    mse = mean_squared_error(enki_output['actual_speed'], enki_output['goal_speed'])

                # skip signals where the car never moved
                skip_record = False
                if 'actual_speed' in enki_output:
                    actual_speed_min = np.min(enki_output['actual_speed'])
                    actual_speed_max = np.max(enki_output['actual_speed'])
                    actual_speed_range = np.abs(actual_speed_max - actual_speed_min)
                    if actual_speed_range == 0.0:
                        skip_record = True

                # add record
                if not skip_record:
                    speed_record = {
                        'index': i,
                        'speed': speed,
                        'mse': mse
                    }
                    speed_records.append(speed_record)

            # sort speed records by MSE
            speed_records.sort(key=lambda x: x['mse'])
            speed_records = speed_records[::-1]

            # prune top signals
            if top is not None:
                speed_records = speed_records[:top]

            # process each speed record
            export_data = list()
            for speed_record in speed_records:
                # add to json data
                export_data.append(speed_record['speed'])

                # show message
                if speed_record['mse'] is not None:
                    message = 'Individual #{:03d} (MSE = {:0.4f})'.format(speed_record['index'], speed_record['mse'])
                else:
                    message = 'Individual #{:03d}'.format(speed_record['index'])
                print(message)

                # show plot
                if show_plots:
                    plt.title(message)
                    plt.plot(speed_record['speed'])
                    plt.show()

            # write json file
            print('Writing data to {}...'.format(output_path))
            file_type = os.path.splitext(output_path)[1]
            if file_type == '.json':
                with open(output_path, 'w') as f:
                    json.dump(export_data, f, indent=3)
            elif file_type == '.pickle':
                with open(output_path, 'wb') as f:
                    pickle.dump(export_data, f)
            else:
                print('Unsupported file type.')


def _read_commandline():
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
        help='path to save json to',
        default=None,
        type=str
    )
    parser.add_argument(
        '--top',
        metavar='',
        help='top number of speed signals to retrieve',
        default=None,
        type=int
    )
    parser.add_argument(
        '--plot',
        action='store_true'
    )
    return parser.parse_args()


if __name__ == '__main__':
    main(_read_commandline())
