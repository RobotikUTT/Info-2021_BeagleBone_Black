#!/usr/bin/env python3

import os
import time
import logging
from pathlib import Path
from datetime import datetime

# Path for the .brain_status file (created in "start.py")
BRAIN_STATUS_PATH = Path(__file__).parent.joinpath('.brain_status')

# Define the path for the logs directory, and create it if it doesn't exist in the "Info-2021_BeagleBone_Black" folder
LOG_DIR = Path(__file__).parent.parent.joinpath('logs')
if not LOG_DIR.is_dir():
	LOG_DIR.mkdir()

# Define the path for the current logs file, in the "logs" folder
CURRENT_LOG_PATH = LOG_DIR.joinpath('.current_log')


def configure_logger(level: int = logging.INFO):

	# Define a path and a name for the logfile, then create and open it.
	# The logfile name will have this pattern : YEAR-MONTH-DAY--HOUR-MINUTES-SECONDS.log     ex : 2021-12-09--18-38-21 for the 12th of December 2021, at 18:38:21
	logfile_path = LOG_DIR.joinpath(datetime.now().strftime('%Y-%m-%d--%H-%M-%S') + '.log')
	logfile_path.touch()
	with open(CURRENT_LOG_PATH, 'w') as f:
		f.write(logfile_path.name)

	config = {
		'level': level,
		'filename': logfile_path,
		'filemode': 'a',
		'encoding': 'utf-8',
		'format': '{asctime}.{msecs:03.0f} - {levelname} - {name}: {message}',
		'datefmt': '%H:%M:%S',
		'style': '{',
	}
	logging.basicConfig(**config)

	return logfile_path.name


def tailf(filename, n=10, stop_function=None, stop_args=()):
	with open(filename, 'r') as f:
		f.seek(0, os.SEEK_END)
		pos = f.tell()
		if n and pos > 0:
			pos -= 1
			f.seek(pos, 0)
			nb_lf = -1
			while True:
				if pos == 0:
					break
				c = f.read(1)
				if c == '\n':
					nb_lf += 1
					if nb_lf == n:
						break
				pos -= 1
				f.seek(pos, 0)

		while True:
			line = f.readline()
			if line:
				yield line
			else:
				if stop_function is not None:
					if stop_function(*stop_args):
						break
				time.sleep(0.1)


def file_has_changed(fd, previous_mtime):
	if os.fstat(fd).st_mtime != previous_mtime:
		return True
	return False


def log_viewer(level: int = logging.INFO, filter_: logging.Filter = None, n: int = 10):
	while not CURRENT_LOG_PATH.is_file():
		time.sleep(0.1)
	with open(CURRENT_LOG_PATH, 'r') as cur_file:
		cur_fd = cur_file.fileno()
		while True:
			mtime = os.fstat(cur_fd).st_mtime
			cur_file.seek(0, os.SEEK_SET)
			filename = cur_file.read()
			logfile = LOG_DIR.joinpath(filename)
			if not logfile.is_file():
				print(f'Error: {filename}: File not found.')
				break
			print(f'Current log file: {filename}')
			for line in tailf(logfile, n, file_has_changed, (cur_fd, mtime)):
				record = parse_record(line)
				if record is None:
					print('### Invalid record ###:', line.strip())
				else:
					if record.level >= level and (filter_ is None or filter_(record)):
						print(line.strip())


def parse_record(line: str):
	attrdict = {}
	parts = line.split(' - ', 2)
	if len(parts) == 3:
		time_, levelname, name_msg = parts
		sub_parts = name_msg.split(': ', 1)
		if len(sub_parts) == 2:
			name, message = sub_parts
			level = logging._nameToLevel.get(levelname)
			if level is not None:
				attrdict = {
					'level': level,
					'name': name,
					'message': message.strip()
				}
				return logging.makeLogRecord(attrdict)
	return None


if __name__ == '__main__':
	from argparse import ArgumentParser
	parser = ArgumentParser()
	parser.add_argument('-D', '--debug', action='store_true',
						help='Enable DEBUG level for logging and log viewer.')
	parser.add_argument('-n', '--lines', type=int, default=10,
						help='Output the last n lines of the current log file.')
	args = parser.parse_args()
	try:
		log_viewer(level=(logging.DEBUG if args.debug else logging.INFO), n=args.lines)
	except KeyboardInterrupt:
		print()
