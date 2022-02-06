#!/usr/bin/env python3

##############################
#         Exit codes         #
# 0 = brain has been stopped #
# 1 = brain is not running   #
# 2 = Error                  #
##############################

import os
import signal

BRAIN_STATUS_PATH = os.path.join(os.path.dirname(__file__), '.brain_status')


class CorruptedFileError(Exception):
    pass


def __kill_processes(pids: list, verbose: bool) -> None:
    """
    Kill the processes which pids are given in the pids list
    :argument pids: a list of the pids of the processes to kill
    """
    # To stop the brain, send a SIGTERM to every PID in this list
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    # Overwrite the status file to indicate that the brain is stopped
    with open(BRAIN_STATUS_PATH, 'w') as f:
        f.write('0\n')
    if verbose:
        print('Stopped.')


def __get_brain_status_content() -> tuple[int, list]:
    """
    Search in the brain status file the current status of the brain and the list of pids of processes which are running
    :raise: CorruptedFileError
    :return: a list which first element is the status and the second the list of pids
    """
    with open(BRAIN_STATUS_PATH, 'r') as f:
        # First read status file content
        content = f.readlines()
    try:
        status = int(content[0][0])
    except ValueError:  # if the first character of the file is not a digit, this means the file is corrupted
        raise CorruptedFileError
    if len(content) > 1:
        # Second line is a list of PID separated by semi-colons
        pids = [int(pid) for pid in content[1].split(';') if pid.isdigit()]
    else:
        pids = []
    return status, pids


def __stop_brain(verbose: bool) -> int:
    """
    Check the brain status file and stops the brain according to its content
    :param verbose: whether complete message on stdout should be displayed or not
    :raise CorruptedFileError
    :return: an exit code : 0 = brain has been stopped ; 1 = brain was not running when the function was called ;
     2 = Error
    """
    status, pids = __get_brain_status_content()
    # status: 0=stopped, 1=running
    if status == 0:  # Brain is not started
        if verbose:
            print('Brain is not started according to the .brain_status file.\n'
                  'If you think otherwise, consider running cleaner.py')
        return 1
    elif status == 1:  # Brain is running
        __kill_processes(pids, verbose)
        return 0
    else:  # Unknown status
        if verbose:
            print('Fatal error: Unknown status number:', status)
        return 2


def stop(verbose: bool = False) -> int:
    """
    Stop the brain and return an exit code.
    - 0 = brain has been stopped
    - 1 = brain was not running when the function was called
    - 2 = Error
    :param verbose: a boolean which tells if the function should display complete messages or not
    """
    try:
        return __stop_brain(verbose)
    except (FileNotFoundError, CorruptedFileError):
        if verbose:
            print('No .brain_status file, the brain may not be started.\n'
                  'If you think otherwise, consider running cleaner.py')
        return 1


if __name__ == '__main__':
    exit_code = stop(verbose=True)
    exit(exit_code)
