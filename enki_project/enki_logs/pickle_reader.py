import os
import pickle

class enki_log_reader():
	def __init__(self):
		PICKLE_FILE = "run1.pickle"
		
		data = self.read_from_pickle(PICKLE_FILE)
		
		
			
		
	
		
		
    def _read_pickle(self):
        """Reads contents from a pickle file.

        :return: file contents
        """
        with open(self._file_path, 'rb') as f:
            result = pickle.load(f)
        return result

			
		
if __name__ == "__main__":
	prog = enki_log_reader()
		
