# -*- coding: utf-8 -*-

import argparse
import numpy as np
import os
import sys
from enki.core.interface import EnkiEvoROSExecutable

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

    def convert_to_evoros_input(self, enki_input):
        """Converts an executable input from Enki into a format for Evo-ROS.

        :param enki_input: an input from Enki for execution
        :return: an input for Evo-ROS for execution
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

        # convert to Evo-ROS input
        evoros_input = {
            'genome': PID_SETTINGS,
            'enki_genome': speed
        }
        return evoros_input

    def convert_from_evoros_result(self, evoros_result):
        """Converts an execution result from Evo-ROS into a format for Enki.

        :param evoros_result: a result from an Evo-ROS execution
        :return: a result for Enki
        """
        # get actual speed from Evo-ROS
        actual_speed = np.array(evoros_result['Actual Speed'])

        # get goal speed from Evo-ROS
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
    _process_archive(args.archive_path, args.output_path)


def _process_archive(archive_path, output_path):
    """

    :param archive_path:
    :param output_path:
    :return:
    """
    print('Enki Archive Plotter')

    if not os.path.exists(archive_path):
        print('ERROR: Invalid archive path.')
        sys.exit(1)
    else:
        archive_name = os.path.splitext(os.path.basename(archive_path))[0]

        # read archive object from file
        reader = ArchiveFileReader(archive_path)
        interface = reader.read_interface()
        archive = reader.read(interface)

        # check if archive is empty
        if len(archive.generations) == 0:
            print('ERROR: Archive is empty.')
            sys.exit(1)
        else:
            # create output path
            if not os.path.exists(output_path):
                print('Creating output directory {}...'.format(output_path))
                os.makedirs(output_path)

            # plot novelty curve
            file_path = os.path.join(output_path, '{}_novelty_curve.png'.format(archive_name))
            print('Plotting to {}...'.format(file_path))
            novelties = list()
            for generation in archive.generations:
                gen_novelties = [individual.novelty for individual in generation]
                if gen_novelties:
                    novelties.append((
                        np.min(gen_novelties),
                        np.mean(gen_novelties),
                        np.max(gen_novelties),
                    ))
            img_stream = plot.novelty_curve(novelties)
            img = Image.open(img_stream)
            img.save(file_path)

            # plot each individual in the last generation of the archive
            data = _read_data(archive.generations[-1])
            for record in data:
                file_path = os.path.join(output_path, '{}_individual_{:03d}.png'.format(archive_name, record['index']))
                print('Plotting to {}...'.format(file_path))
                fig = plt.figure(figsize=(6.4, 4.8), dpi=100)
                ax = fig.gca()
                if 'mse' in record:
                    ax.set_title('Individual {:03d}, MSE={:0.4f}'.format(record['index'], record['mse']))
                else:
                    ax.set_title('Individual {:03d}'.format(record['index']))
                ax.set_ylim(MIN_SPEED, MAX_SPEED)
                if 'actual_speed' in record:
                    ax.plot(np.arange(len(record['actual_speed'])), record['actual_speed'], label='Actual Speed')
                if 'goal_speed' in record:
                    ax.plot(np.arange(len(record['goal_speed'])), record['goal_speed'], label='Goal Speed')
                if 'error' in record:
                    ax.plot(np.arange(len(record['error'])), record['error'], label='Error')
                ax.legend()
                fig.savefig(file_path)
                plt.close(fig)

            # display individuals ranked by mse
            print('----------------------------')
            print('Individuals by MSE')
            print('----------------------------')
            sorted_data = [{k: v for k, v in record.items()} for record in data]
            sorted_data.sort(key=lambda x: x['mse'])
            sorted_data = sorted_data[::-1]
            for record in sorted_data:
                print('Individual {:03d}, MSE: {:0.3f}'.format(record['index'], record['mse']))
            print('----------------------------')

            archived_data = _read_data(archive.generations[-1])
            archived_mses = [record['mse'] for record in archived_data]
            evaluated_data = _read_data(interface.evaluated_individuals)
            evaluated_mses = [record['mse'] for record in evaluated_data]

            mse_min = np.min(evaluated_mses)
            mse_max = np.max(evaluated_mses)
            q1 = np.percentile(evaluated_mses, 25)
            q3 = np.percentile(evaluated_mses, 75)

            archived_colors = ['green' if mse <= q1 else 'red' if mse >= q3 else 'blue' for mse in archived_mses]
            evaluated_colors = ['green' if mse <= q1 else 'red' if mse >= q3 else 'blue' for mse in evaluated_mses]

            file_path = os.path.join(output_path, '{}_mse_distribution.png'.format(archive_name))
            print('Plotting to {}...'.format(file_path))
            img = _plot_distribution(archived_data, evaluated_data, x_min=mse_min, x_max=mse_max)
            img.save(file_path)

            file_path = os.path.join(output_path, '{}_phenome_mds.png'.format(archive_name))
            print('Plotting to {}...'.format(file_path))
            behaviors = BehaviorComputer.compute_phenome_matrix(archive.generations[-1] + interface.evaluated_individuals)
            distance_matrix = DistanceComputer.compute_distance_matrix(behaviors, archive.settings.distance_metric)
            img = _plot_mds(
                distance_matrix,
                title='MDS Plot of Individuals based on Phenome',
                num_labeled=len(archive.generations[-1]),
                labeled_colors=archived_colors,
                unlabeled_colors=evaluated_colors,
                radius=0.3
            )
            img.save(file_path)

            file_path = os.path.join(output_path, '{}_genome_mds.png'.format(archive_name))
            print('Plotting to {}...'.format(file_path))
            behaviors = BehaviorComputer.compute_genome_matrix(archive.generations[-1] + interface.evaluated_individuals)
            distance_matrix = DistanceComputer.compute_distance_matrix(behaviors, archive.settings.distance_metric)
            img = _plot_mds(
                distance_matrix,
                title='MDS Plot of Individuals based on Genome',
                num_labeled=len(archive.generations[-1]),
                labeled_colors=archived_colors,
                unlabeled_colors=evaluated_colors,
                radius=1.0
            )
            img.save(file_path)


def _plot_distribution(archived_data, evaluated_data, x_min=None, x_max=None, image_format='png'):
    """

    :param archived_data:
    :param evaluated_data:
    :param x_min:
    :param x_max:
    :param image_format:
    :return:
    """
    archived_mses = [record['mse'] for record in archived_data]
    evaluated_mses = [record['mse'] for record in evaluated_data]

    y_min = 0.0
    y_max = 100.0
    if (x_min is not None) and (x_max is not None) and (x_min == x_max):
        x_min -= x_min * 0.5
        x_max += x_max * 0.5
    if (y_min is not None) and (y_max is not None) and (y_min == y_max):
        y_min -= y_min * 0.5
        y_max += y_max * 0.5

    # prepare the plot
    plt.rc('font', family='Courier New', weight='bold')
    fig = plt.figure(figsize=(6.4, 4.8), dpi=100)
    ax = fig.gca()
    ax.set_title('MSE Distribution', fontweight='bold')
    ax.set_xlabel('MSE', fontweight='bold')
    ax.set_ylabel('Density', fontweight='bold')
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.grid(True)

    # plot density curve
    try:
        density = gaussian_kde(evaluated_mses)
        density.covariance_factor = lambda: 0.25
        density._compute_covariance()

        x = np.linspace(min(evaluated_mses), max(evaluated_mses), 100)
        y = density.evaluate(x)
        ax.plot(x, y * 100.0, 'g-')
        ax.fill_between(x, y * 100.0, 0.0, color='#00e673')
    except np.linalg.LinAlgError:
        pass
    except ValueError:
        pass

    # plot median and interquartile ranges
    ax.vlines(np.percentile(evaluated_mses, 25), 0.0, 100.0, linestyles='--', colors='#444444')
    ax.vlines(np.percentile(evaluated_mses, 50), 0.0, 100.0, linestyles='-', colors='#444444')
    ax.vlines(np.percentile(evaluated_mses, 75), 0.0, 100.0, linestyles='--', colors='#444444')

    # plot rug plot
    ax.vlines(archived_mses, 0.0, 5.0, linestyles='-', colors='g')

    # create image and close plot
    img_stream = io.BytesIO()
    fig.savefig(img_stream, format=image_format)
    img = Image.open(img_stream)
    plt.close(fig)

    return img


def _plot_mds(distance_matrix, title=None, num_labeled=None, labeled_colors='blue', unlabeled_colors='gray', radius=1.0, image_format='png'):
    """

    :param distance_matrix:
    :param title:
    :param num_labeled:
    :param labeled_colors:
    :param unlabeled_colors:
    :param radius:
    :param image_format:
    :return:
    """
    # prepare the plot
    plt.rc('font', family='Courier New', weight='bold')
    fig = plt.figure(figsize=(6.4, 4.8), dpi=100)
    ax = fig.gca()
    if title is None:
        ax.set_title('Distance Projection (Multi-Dimensional Scaling)', fontweight='bold')
    else:
        ax.set_title(title)
    ax.set_xlim([-radius, radius])
    ax.set_ylim([-radius, radius])
    ax.grid(True)

    # perform multidimensional scaling
    mds = manifold.MDS(n_components=2, dissimilarity='precomputed', random_state=1)
    results = mds.fit(distance_matrix)
    coords = results.embedding_

    # plot points and label them
    fig.subplots_adjust(bottom=0.1)

    if num_labeled is None:
        num_labeled = distance_matrix.shape[0]

    ax.scatter(coords[num_labeled:, 0], coords[num_labeled:, 1], marker='.', color=unlabeled_colors, alpha=0.5)
    ax.scatter(coords[:num_labeled, 0], coords[:num_labeled, 1], marker='o', color=labeled_colors, edgecolor='black')
    labels = list(range(1, num_labeled + 1))
    for label, x, y in zip(labels, coords[:num_labeled, 0], coords[:num_labeled, 1]):
        ax.annotate(label, xy=(x, y), xytext=(10, -10), textcoords='offset points', ha='right', va='bottom')

    # create image and close plot
    img_stream = io.BytesIO()
    fig.savefig(img_stream, format=image_format)
    img = Image.open(img_stream)
    plt.close(fig)

    return img


def _read_data(individuals):
    """

    :param individuals:
    :return:
    """
    data = list()
    for i in range(len(individuals)):
        # get output from individual
        try:
            enki_output = individuals[i].phenome.values
            record = {k: v for k, v in enki_output.items()}
            record['index'] = i + 1
            if 'actual_speed' and 'goal_speed' in record:
                record['mse'] = mean_squared_error(record['actual_speed'], record['goal_speed'])
            data.append(record)
        except KeyError:
            print('ERROR: Incompatible archive file.')
            sys.exit(1)
    return data


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
        help='directory to save plots to',
        default=os.getcwd(),
        type=str
    )
    return parser.parse_args()


if __name__ == '__main__':
    import io
    from enki.core.io import ArchiveFileReader
    from enki.core.plotting import plot
    from enki.core.misc import BehaviorComputer
    from enki.core.misc import DistanceComputer
    from PIL import Image
    from scipy.stats import gaussian_kde
    from sklearn import manifold
    from sklearn.metrics import mean_squared_error
    import matplotlib.pyplot as plt
    main(_read_commandline())
