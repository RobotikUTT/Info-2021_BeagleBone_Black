#!/usr/bin/env python3
import os
import signal
from subprocess import Popen

# #### Arguments parsing definition ####
from argparse import ArgumentParser
parser = ArgumentParser()
parser.add_argument('-C', '--clean', action='store_true',
					help='Run cleaner.py (Warning: this will kill every user python3 process)')
parser.add_argument('-R', '--restart', action='store_true',
					help='Run cleaner.py and then restart. (Warning: this will kill every user python3 process)')
args = parser.parse_args()
# #### End of Arguments parsing definition ####


# TODO: parse arguments (like "simulation" and "debug") and handle them
if args.clean or args.restart:
	from cleaner import clean
	clean()
	if not args.restart:		# If we are not restarting, just exit now. Else continue.
		exit(0)


# #### Brain status handeling ####
try:
	with open('.brain_status', 'r') as f:
		status = f.read(1)
		if status == '0':
			# Brain is not started
			# Continue normally
			pass
		elif status == '1':
			# Brain is already started or didn't get stopped properly
			print('Fatal error: Brain is already started or didn\'t get properly stopped')
			print('Consider using --restart (-R) or --clean (-C).')
			exit(1)
		else:
			# Unknown status
			print('Fatal error: Unknwon status number:', status)
			exit(2)
except FileNotFoundError:
	pass
finally:
	with open('.brain_status', 'w') as f:
		f.write('1\n')
# #### End of Brain status handeling ####

# Fork the main thread into a child process in order to be able to close the parent while preserving the
# child. Thus permiting to close the printer loop and exit the SSH session without stoping the brain.
child_pid = os.fork()
if child_pid:
	# Parent process (run in foreground and can be stopped with CTRL+C)

	# Write both processes PID to the .brain_status file in order to be able to stop them later
	with open('.brain_status', 'a') as f:
		f.write(';' + str(os.getegid()) + ';' + str(child_pid))

	# TODO: `tail -f` like printer for log module
	proc = Popen(['tail', '-f', 'test.log'])
	with open('.brain_status', 'a') as f:
		f.write(';' + str(proc.pid))
	proc.wait()
else:
	# Child process (run in background and should not print anything)

	# #### Catching Keyboard Intterupt signal (SIGINT) ####
	def signal_pass(signum, frame):
		# Ignore signal.
		pass
	signal.signal(signal.SIGINT, signal_pass)
	# #### End of SIGINT ####

	# #### Node management ####
	from _nodes_engine import start_nodes_engine
	start_nodes_engine()
	# #### End of Node management ####

	print('End of things')
	# If execution goes here, then all nodes have stopped.
	# This is a clean exit, so put that in log.
	# log.info('Clean exit.')
