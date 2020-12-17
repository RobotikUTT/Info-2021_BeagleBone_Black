#!/usr/bin/env python3
import os
import time
import signal
import subprocess

do_not_kill = []


def get_python_processes():
	# Uses `ps -ux` to get a list of running processes, then filter for 'python3' string in the launch command
	sub_ps = subprocess.Popen(['ps', '-ux'], stdout=subprocess.PIPE)
	output, error = sub_ps.communicate()
	return [line.split() for line in output.splitlines() if b'python3' in line]


def kill_python_processes(signum):
	# 	a) Get all pid for Python3 processes
	ps_out = get_python_processes()
	# 	b) Send signal {signum}
	for process in ps_out:
		pid = int(process[1])
		cmd = b' '.join(process[10:])
		if pid not in do_not_kill:
			ans = input(f'Send {signum.name} to process "{cmd.decode()}" (pid={pid}) ? (Y/n) ')
			if ans.lower() in ('', 'y', 'yes', 'o', 'oui'):
				os.kill(pid, signum)
				print(f'- {signum.name} sent to python3 process with pid {pid}: {cmd.decode()}')
			else:
				do_not_kill.append(pid)


def clean():
	do_not_kill.append(os.getpid())
	# 1) Try to terminate all Python3 processes except itself
	print('[Cleaner] Trying to terminate every Python3 process..')
	if len(get_python_processes()) == len(do_not_kill):
		print('No python3 process found. Skiping this part.')
	else:
		kill_python_processes(signal.SIGTERM)
		for i in range(3):
			print('.', end='')
			if len(get_python_processes()) > len(do_not_kill):
				time.sleep(1)
		print()
		if len(get_python_processes()) > len(do_not_kill):
			# 2) If some are still not terminated, try to kill them
			print('[Cleaner] Some processes are not responding.')
			ans = input('Kill them ? (with extreme violence) (Y/n) ')
			if ans.lower() in ['y', 'yes', 'o', 'oui', '']:
				print('[Cleaner] Killing every Python3 process..')
				kill_python_processes(signal.SIGKILL)
			else:
				print('[Warning] Brain may be still running. Please check manually with `ps -ux`')

	# 3) Overwrite the .brain_status file
	print('[Cleaner] Overwriting .brain_status file..')
	with open('.brain_status', 'w') as f:
		f.write('0')

	print('[Cleaner] Cleaning done.')


if __name__ == '__main__':
	clean()
