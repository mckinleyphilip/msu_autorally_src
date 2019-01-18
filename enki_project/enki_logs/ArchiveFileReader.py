# -*- coding: utf-8 -*-

import json
import os
import pickle
from enki.core.config import Settings
from enki.core.evolution import Archive
from enki.core.evolution import Genome
from enki.core.evolution import Individual
from enki.core.evolution import Phenome
from enki.core.io import InterfaceFileReader


class ArchiveFileReader(object):
    """Handles reading an archive from a file.

    """
    def __init__(self, file_path):
        """Initializes the object.

        :param file_path: a file path to an archive file
        """
        self._file_path = file_path
        self._file_type = os.path.splitext(self._file_path)[1]

    def read(self, interface=None):
        """Reads an archive from an archive file.

        :param interface: an interface to the executable being processed by Enki
        :return: an archive
        """
        archive = None
        if os.path.exists(self._file_path):
            # read the file contents
            if self._file_type == '.json':
                data = self._read_json()
            elif self._file_type == '.pickle':
                data = self._read_pickle()
            else:
                data = dict()

            if 'settings' in data:
                # read the settings from the archive file and create an archive object
                settings = Settings(**data['settings'])
                archive = Archive(settings)

                # if no interface is given, try to load the interface referenced by the archive file
                if interface is None and os.path.exists(archive.settings.executable_path):
                    reader = InterfaceFileReader(archive.settings.executable_path)
                    interface = reader.read()

                if interface is None:
                    raise ValueError('Unable to read interface.')

                # reconstruct all evaluated individuals
                if 'evaluated' in data:
                    # reconstruct each individual
                    for individual_definition in data['evaluated']:
                        # encode a genome
                        genome = None
                        if interface is not None and 'inputs' in individual_definition:
                            encoding = interface.input.encode(individual_definition['inputs'])
                            genome = Genome(encoding)

                        # encode a phenome
                        phenome = None
                        if interface is not None and 'outputs' in individual_definition:
                            encoding = interface.output.encode(individual_definition['outputs'])
                            phenome = Phenome(encoding)

                        # create the individual and assign its attributes
                        individual = Individual(genome, phenome)
                        if 'age' in individual_definition:
                            individual.age = individual_definition['age']

                        if 'novelty' in individual_definition:
                            individual.novelty = individual_definition['novelty']

                        # add the individual to the generation
                        interface.evaluated_individuals.append(individual)

                # reconstruct all archive generations
                if 'generations' in data:
                    # reconstruct each generation in the archive file
                    for generation_definition in data['generations']:
                        generation = list()
                        # reconstruct each individual in the generation
                        for individual_definition in generation_definition:
                            # encode a genome
                            genome = None
                            if interface is not None and 'inputs' in individual_definition:
                                encoding = interface.input.encode(individual_definition['inputs'])
                                genome = Genome(encoding)

                            # encode a phenome
                            phenome = None
                            if interface is not None and 'outputs' in individual_definition:
                                encoding = interface.output.encode(individual_definition['outputs'])
                                phenome = Phenome(encoding)

                            # create the individual and assign its attributes
                            individual = Individual(genome, phenome)
                            if 'age' in individual_definition:
                                individual.age = individual_definition['age']
                            if 'novelty' in individual_definition:
                                individual.novelty = individual_definition['novelty']

                            # add the individual to the generation
                            generation.append(individual)

                        # add the generation to the archive
                        archive.generations.append(generation)

        return archive

    def read_interface(self):
        """Reads the interface from the executable referenced by an archive file.

        :return: an interface to the executable being processed by Enki
        """
        result = None
        if os.path.exists(self._file_path):
            # read the file contents
            if self._file_type == '.json':
                data = self._read_json()
            elif self._file_type == '.pickle':
                data = self._read_pickle()
            else:
                data = dict()

            if 'settings' in data:
                # read the settings from the archive file
                settings = Settings(**data['settings'])

                # read the interface data from the executable referenced by the archive file
                reader = InterfaceFileReader(settings.executable_path)
                result = reader.read()
        return result

    def _read_json(self):
        """Reads contents from a json file.

        :return: file contents
        """
        with open(self._file_path, 'r') as f:
            result = json.load(f)
        return result

    def _read_pickle(self):
        """Reads contents from a pickle file.

        :return: file contents
        """
        with open(self._file_path, 'rb') as f:
            result = pickle.load(f)
        return result
