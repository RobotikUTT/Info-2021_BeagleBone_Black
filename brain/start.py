#!/usr/bin/env python3
import os
import signal
import logging
from argparse import ArgumentParser, Namespace

BRAIN_STATUS_PATH = os.path.join(os.path.dirname(__file__), '.brain_status')


def __parse_args(parser: ArgumentParser) -> Namespace:
    # TODO: parse arguments (like "simulation" and "debug") and handle them
    # #### Arguments parsing definition ####
    parser.add_argument('-C', '--clean', action='store_true',
                        help='Run cleaner.py (Warning: this might kill every user Python3 process)')
    parser.add_argument('-R', '--restart', action='store_true',
                        help='Run cleaner.py and then restart. (Warning: this might kill every user Python3 process)')
    parser.add_argument('-I', '--interactive', action='store_true',
                        help='Open a Ipython interpreter while the brain is running (require Ipython)')
    parser.add_argument('-D', '--debug', action='store_true',
                        help='Enable DEBUG level for logging and log viewer.')
    parser.add_argument('-N', '--noprint', action='store_true',
                        help='Log viewer disable. Background process only.')
    parser.add_argument('-S', '--simulation', action='store_true',
                        help='Start the brain in simulation mode. (Virtual Can, etc)')
    parser.add_argument('--nocan', action='store_true',
                        help='Start the brain without Can support (Usefull for local tests)')
    args = parser.parse_args()
    return args


def __check_args(args) -> None:
    if args.clean or args.restart:
        from brain.stop import stop
        from brain.cleaner import clean
        print('Cleaning...')
        exit_code = stop()
        if exit_code == 0:
            print('Brain stopped.')
        else:
            clean()
        if not args.restart:  # If we are not restarting, just exit now. Else continue.
            exit(0)
        else:
            print('Restarting...')


def __handle_brain_status() -> None:
    try:
        with open(BRAIN_STATUS_PATH, 'r') as f:
            status = f.read(1)
            match status:
                case '0':  # Brain is not started. Continue normally
                    pass
                case '1':
                    print('Fatal error: Brain is already started or didn\'t get properly stopped\n'
                          'Consider using --restart (-R) or --clean (-C).')
                    exit(1)
                case _:
                    # Unknown status
                    print('Fatal error: Unknwon status number:', status)
                    exit(2)
    except FileNotFoundError:
        pass
    finally:
        with open(BRAIN_STATUS_PATH, 'w') as f:
            f.write('1\n')


def start():
    parser = ArgumentParser()
    args = __parse_args(parser)
    __check_args(args)
    __handle_brain_status()

    # Fork the main thread into a child process in order to be able to close the parent while preserving the
    # child. Thus permiting to close the printer loop and exit the SSH session without stoping the brain.
    child_pid = os.fork()
    if child_pid:
        # Parent process (run in foreground and can be stopped with CTRL+C)

        # Write both processes PID to the .brain_status file in order to be able to stop them later
        with open(BRAIN_STATUS_PATH, 'a') as f:
            f.write(f';{os.getegid()};{child_pid}')

        if args.interactive:
            try:
                os.waitpid(child_pid, 0)
            except KeyboardInterrupt:
                print()
        elif not args.noprint:
            import time
            from brain.logger import log_viewer
            time.sleep(0.1)
            try:
                log_viewer(logging.DEBUG if args.debug else logging.INFO)
            except KeyboardInterrupt:
                print()
    else:
        # Child process (run in background and should not print anything)

        # #### Catching Keyboard Intterupt signal (SIGINT) ####
        def signal_pass(signum, frame):
            # Ignore signal.
            pass

        signal.signal(signal.SIGINT, signal_pass)
        # #### End of SIGINT ####

        # Configure the logger
        from brain.logger import configure_logger
        configure_logger(logging.DEBUG if args.debug else logging.INFO)

        # #### Node management ####
        from brain._nodes_engine import start_nodes_engine
        start_nodes_engine(args)
        # #### End of Node management ####

        print('End of things')
    # If execution goes here, then all nodes have stopped.
    # This is a clean exit, so put that in log.
    # log.info('Clean exit.')


if __name__ == '__main__':
    start()
