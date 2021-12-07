#!/usr/bin/env python3

##############################
#         Exit codes         #
# 0 = brain has been stopped #
# 1 = brain is not running   #
# 2 = Error                  #
##############################

import os
import signal

BRAIN_STATUS_PATH = os.path.join(os.path.dirname(__file__), '.brain_status') # hacky way: https://stackoverflow.com/questions/7505988/importing-from-a-relative-path-in-python


class CorruptedFileError(Exception):
	pass


def stop(verbose=False):
	# #### Brain status handeling ####
	try:
		with open(BRAIN_STATUS_PATH, 'r') as f:
			# First read status file content
			content = f.readlines()
		if not content:
			raise CorruptedFileError
		# First character of the first line is the status: 0=stopped, 1=running
		status = content[0][0]
		if status == '0':
			# Brain is not started
			if verbose:
				print('Brain is not started according to the .brain_status file.')
				print('If you think otherwise, consider running cleaner.py')
			return 1
		elif status == '1':
			# Brain is running
			if len(content) < 2:
				raise CorruptedFileError
			# Second line is a list of PID separated by semi-colons
			pids = [int(pid) for pid in content[1].split(';') if pid.isdigit()]
			# To stop the brain, send a SIGTERM to every PID in this list
			for pid in pids:
				try:
					os.kill(pid, signal.SIGTERM)
				except ProcessLookupError:
					pass
			# Finally, overwrite the status file to indicate that the brain is stopped
			with open(BRAIN_STATUS_PATH, 'w') as f:
				f.write('0\n')
			if verbose: print('Stopped.')
			return 0
		else:
			# Unknown status
			if verbose: print('Fatal error: Unknown status number:', status)
			return 2
	except (FileNotFoundError, CorruptedFileError):
		if verbose:
			print('No .brain_status file, the brain may not be started.')
			print('If you think otherwise, consider running cleaner.py')
		return 1
	# #### End of Brain status handeling ####


if __name__ == '__main__':
	exit_code = stop(verbose=True)
	exit(exit_code)
