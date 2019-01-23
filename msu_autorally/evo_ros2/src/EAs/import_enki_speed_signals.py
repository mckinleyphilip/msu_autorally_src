import sys
import json
from pprint import pprint

class EnkiSpeedSignalImporter():
	def __init__(self):
		pass
		
	def import_signals(self, filename):
		with open(filename) as f:
			data = json.load(f)
			
		return data
		
if __name__ == "__main__":
	prog = EnkiSpeedSignalImporter()
	filename = "exp2_c1_enki_run_10ms_top5_novel_signals.json"
	prog.import_signals(filename)
	
		
