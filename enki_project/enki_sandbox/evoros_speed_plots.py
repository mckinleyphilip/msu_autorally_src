# -*- coding: utf-8 -*-

import argparse
import numpy as np
import os
from enki.core.errors import EnkiError
from enki.core.errors import EnkiFileError
from enki.core.io import ArchiveFileReader
from enki.core.plotting import plot
from enki.core.misc import BehaviorComputer
from enki.core.misc import DistanceComputer
from PIL import Image
from sklearn.metrics import mean_squared_error


def main(args):
    """

    :param args:
    :return:
    """
    try:
        _process_archive(args.archive_path, args.output_path)
    except EnkiError as e:
        print('ERROR: {}'.format(e))
    except KeyboardInterrupt:
        print('\nAborting.')


def _process_archive(archive_path, output_path):
    """

    :param archive_path:
    :param output_path:
    :return:
    """
    if not os.path.exists(archive_path):
        raise EnkiFileError('Invalid location for archive file.')
    else:
        archive_name = os.path.splitext(os.path.basename(archive_path))[0]

        # read archive object from file
        reader = ArchiveFileReader(archive_path)
        interface = reader.read_interface()
        archive = reader.read(interface)

        # check if archive is empty
        if len(archive.generations) == 0:
            raise EnkiFileError('Archive file is empty.')
        else:
            # create output path
            if not os.path.exists(output_path):
                print('Creating output directory {}...'.format(output_path))
                os.makedirs(output_path)

            archived_individuals = archive.archived_individuals
            archived_data = _read_data(archived_individuals)
            archived_mses = [record['mse'] for record in archived_data]

            evaluated_individuals = list(archive.evaluated_individuals.values())
            evaluated_data = _read_data(evaluated_individuals)
            evaluated_mses = [record['mse'] for record in evaluated_data]

            mse_min = np.min(evaluated_mses)
            mse_max = np.max(evaluated_mses)
            q1 = np.percentile(evaluated_mses, 25)
            q3 = np.percentile(evaluated_mses, 75)

            archived_labels = [i + 1 for i in range(len(archived_individuals))]
            archived_colors = ['#00FF00' if mse <= q1 else '#FF0000' if mse >= q3 else '#0000FF' for mse in archived_mses]
            evaluated_labels = [None for _ in range(len(evaluated_individuals))]
            evaluated_colors = ['#88DD88' if mse <= q1 else '#DD8888' if mse >= q3 else '#8888DD' for mse in evaluated_mses]

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
            img_stream = plot.plot_curve(novelties)
            img = Image.open(img_stream)
            img.save(file_path)

            # display individuals ranked by mse
            print('----------------------------')
            print('Individuals by MSE')
            print('----------------------------')
            sorted_data = [{k: v for k, v in record.items()} for record in archived_data]
            sorted_data.sort(key=lambda x: x['mse'], reverse=True)
            for record in sorted_data:
                print('Individual {:03d}, MSE: {:0.3f}'.format(record['index'], record['mse']))
            print('----------------------------')

            # plot MSE distribution
            file_path = os.path.join(output_path, '{}_mse_distribution.png'.format(archive_name))
            print('Plotting to {}...'.format(file_path))
            img_stream = plot.plot_distribution(
                archived_mses + evaluated_mses,
                rug_values=archived_mses,
                x_min=mse_min,
                x_max=mse_max,
                y_scale=100.0,
                title='MSE Distribution',
                x_label='MSE'
            )
            img = Image.open(img_stream)
            img.save(file_path)

            # plot phenome MDS
            file_path = os.path.join(output_path, '{}_phenome_mds.png'.format(archive_name))
            print('Plotting to {}...'.format(file_path))
            behaviors = BehaviorComputer.compute_phenome_matrix(archived_individuals + evaluated_individuals)
            distance_matrix = DistanceComputer.compute_distance_matrix(behaviors, archive.settings.distance_metric)
            distance_labels = archived_labels + evaluated_labels
            distance_colors = archived_colors + evaluated_colors
            img_stream = plot.plot_mds(
                distance_matrix,
                labels=distance_labels,
                colors=distance_colors,
                title='MDS Plot of Individuals based on Phenome',
                radius=0.3
            )
            img = Image.open(img_stream)
            img.save(file_path)

            # plot genome MDS
            file_path = os.path.join(output_path, '{}_genome_mds.png'.format(archive_name))
            print('Plotting to {}...'.format(file_path))
            behaviors = BehaviorComputer.compute_genome_matrix(archived_individuals + evaluated_individuals)
            distance_matrix = DistanceComputer.compute_distance_matrix(behaviors, archive.settings.distance_metric)
            distance_labels = archived_labels + evaluated_labels
            distance_colors = archived_colors + evaluated_colors
            img_stream = plot.plot_mds(
                distance_matrix,
                labels=distance_labels,
                colors=distance_colors,
                title='MDS Plot of Individuals based on Genome',
                radius=1.0
            )
            img = Image.open(img_stream)
            img.save(file_path)


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
            if 'actual_speed' in record and 'goal_speed' in record:
                record['mse'] = mean_squared_error(record['actual_speed'], record['goal_speed'])
            data.append(record)
        except KeyError:
            raise EnkiFileError('Incompatible archive file.')
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
    main(_read_commandline())
