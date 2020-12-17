#!/usr/bin/env python3
import os
import signal


class CorruptedFileError(Exception):
	pass


# #### Brain status handeling ####
try:
	with open('.brain_status', 'r') as f:
		content = f.readlines()
		if not content:
			raise CorruptedFileError
		status = content[0][0]
		if status == '0':
			# Brain is not started
			raise
			print('Brain is not started according to the .brain_status file.')
			print('If you think otherwise, consider running cleaner.py')
			exit(1)
		elif status == '1':
			# Brain is running
			if len(content) < 2:
				raise CorruptedFileError
			# TODO: read pids from .brain_status file and terminate them (sigterm)
			pids = [int(pid) for pid in content[1].split(';') if pid.isdigit()]
			for pid in pids:
				try:
					os.kill(pid, signal.SIGTERM)
				except ProcessLookupError:
					pass
			with open('.brain_status', 'w') as f:
				f.write('0\n')
			exit(0)
		else:
			# Unknown status
			print('Fatal error: Unknwon status number:', status)
			exit(2)
except (FileNotFoundError, CorruptedFileError):
	print('No .brain_status file, the brain should not be started.')
	print('If you think otherwise, consider running cleaner.py')
# #### End of Brain status handeling ####
