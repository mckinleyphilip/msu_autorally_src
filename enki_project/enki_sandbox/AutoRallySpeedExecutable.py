# -*- coding: utf-8 -*-

import numpy as np
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
	def input_definition(self):
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
		try:
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
		except:
			raise ValueError('Failed to create speed signal.')

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
		
		print('Recieved result..')
		for entry, value in evoros_result.iteritems:
			print(entry)
		# get actual speed from Evo-ROS
		if 'Actual Speed' in evoros_result:
			actual_speed = np.array(evoros_result['Actual Speed'])
		elif 'actual_speed' in evoros_result:
			actual_speed = np.array(evoros_result['actual_speed'])
		else:
			raise ValueError('Missing actual speed.')

		# get goal speed from Evo-ROS
		if 'Goal Speed' in evoros_result:
			goal_speed = np.array(evoros_result['Goal Speed'])
		elif 'goal_speed' in evoros_result:
			goal_speed = np.array(evoros_result['goal_speed'])
		else:
			raise ValueError('Missing goal speed.')

		# compute the error
		error = np.abs(actual_speed - goal_speed)

		# convert to Enki result
		enki_result = {
			'pid_settings': PID_SETTINGS,
			'min_speed': MIN_SPEED,
			'max_speed': MAX_SPEED,
			'time_start': TIME_START,
			'time_end': TIME_END,
			'time_step': TIME_STEP,
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
		if f_type == 'linear':
			# use a linear function
			x = f_max + (t - t_max) * (f_max - f_min) / (t_max - t_min)
		elif f_type == 'cosine':
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
