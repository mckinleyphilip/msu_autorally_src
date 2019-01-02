import numpy as np
from enki.core.interface import EnkiExecutable


class TestExecutable(EnkiExecutable):
    """TestExecutable class

    """
    @property
    def input_parameters(self):
        return {
            'radius': [0.0, 1.0],
            'angle': [0.0, 2.0 * np.pi]
        }

    @property
    def output_parameters(self):
        return {
            'x': [0.0, 1.0],
            'y': [0.0, 1.0]
        }

    def execute(self, input_values):
        """

        :param input_values:
        :return:
        """
        result = {
            'x': input_values['radius'] * np.cos(input_values['angle']),
            'y': input_values['radius'] * np.sin(input_values['angle'])
        }

        return result


if __name__ == '__main__':
    import pprint
    import time

    pp = pprint.PrettyPrinter(indent=4)
    start_time = time.time()

    exe_instance = TestExecutable()
    print('Selecting random inputs...')
    exe_inputs = dict()
    for k, v in exe_instance.input_parameters.items():
        try:
            exe_inputs[k] = np.random.rand() * (v[1] - v[0]) + v[0]
            if type(v[0]) == int:
                exe_inputs[k] = int(exe_inputs[k])
        except TypeError:
            exe_inputs[k] = v[np.random.randint(len(v))]

    print('Inputs:')
    pp.pprint(exe_inputs)

    print('Executing...')
    exe_outputs = exe_instance.execute(exe_inputs)

    print('Outputs:')
    pp.pprint(exe_outputs)

    elapsed_time = time.time() - start_time
    print('Done. ({:02.0f}m {:02.0f}s)'.format(*divmod(elapsed_time, 60.0)))

