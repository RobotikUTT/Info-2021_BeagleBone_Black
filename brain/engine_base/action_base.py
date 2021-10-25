from __future__ import annotations

import logging
from threading import Thread, Event
from queue import Queue, deque
from abc import ABC, abstractmethod
from collections.abc import Iterable, Callable

from .node_base import NodeBase


action_logger = logging.getLogger('action')


class InterruptBase:
	pass

	def __repr__(self):
		return f'{self.__class__.__name__}'


class ActionAborted(Exception):
	pass


class ActionBase(ABC):
	parent_node = None
	calling_action = None

	def __init__(self, **kwargs):
		self.pause_event = Event()
		self.pause_event.set()
		self.abort = False
		self.pending_interrupts = deque()
		self.interupt_handlers = {}
		self.current_interrupt = None

	def __repr__(self):
		return f'{self.__class__.__name__}'

	def start(self, parent_node: NodeBase, calling_action: ActionBase):
		self.parent_node = parent_node
		self.calling_action = calling_action
		action_logger.info(f'Start of action: {self}')
		try:
			self.run()
		except ActionAborted:
			self.on_abort()
			action_logger.info(f'Aborted action ! {self}')
		action_logger.info(f'End of action: {self}')

	""" Function to be defined by inherited actions but make sure to call
		self.check_for_interruption() as often as possible to permit pauses and interrupts.
	"""
	@abstractmethod
	def run(self):
		self.check_for_interruption()
		pass

	def run_action(self, action):
		self.calling_action.run_action(action)

	def is_paused(self):
		return not self.pause_event.is_set()

	def pause(self):
		self.pause_event.clear()

	def resume(self):
		self.pause_event.set()

	def abort(self):
		self.abort = True
		if self.is_paused():		# If action is actualy paused
			self.resume()			# Resume action in order to abort

	def on_abort(self):
		"""
		Code to run on action abort. Like stop the wheels.
		"""
		pass

	def add_interrupt_handler(self, interruption_cls, handler: Callable[[ActionBase], None]):
		self.interupt_handlers[interruption_cls] = handler

	def interrupt(self, interruption: InterruptBase):
		self.pending_interrupts.append(interruption)
		if self.is_paused():		# If action is actualy paused
			self.resume()			# Resume action in order to abort

	def check_for_interruption(self):
		self.pause_event.wait()		# Block if pause Event is unset, pass if set.
		if self.abort:
			raise ActionAborted()
		if self.pending_interrupts:
			self.current_interrupt = self.pending_interrupts.pop()
			if self.current_interrupt.__class__ in self.interupt_handlers:
				handler = self.interupt_handlers[self.current_interrupt.__class__]
				handler(self)
			else:
				action_logger.warning(
					f'Unhandled interruption: {self.current_interrupt} in action: {self}. Skipping.\n' +
					'Consider using `skip_interrupt` handler to remove this warning.'
				)
			self.current_interrupt = None
			self.check_for_interruption()


class ActionThread(ActionBase):
	all_instances = []

	def __init__(self, parent_node: NodeBase, **kwargs):
		super().__init__(**kwargs)
		self.all_instances.append(self)
		self.parent_node = parent_node
		self.current_action = None
		self.actions_stack = deque()
		self.interupt_event = Event().set()
		self.actions_schedule = Queue()

	def schedule_action(self, action: ActionBase):
		self.actions_schedule.put(action)

	def start(self):
		t_name = f'{self.__class__.__name__}_{len(self.all_instances)}'
		t = Thread(name=t_name, target=self.run, daemon=True)
		self.parent_node.threads.append(t)
		t.start()

	def run_action(self, action):
		self.current_action = action
		self.actions_stack.append(action)
		action.start(self.parent_node, self)
		self.actions_stack.pop()
		if self.actions_stack:
			self.current_action = self.actions_stack[-1]
		else:
			self.current_action = None

	def run(self):
		action_logger.info(f'Start of action thread: {self}')
		while not self.actions_schedule.empty():
			next_action = self.actions_schedule.get()
			self.run_action(next_action)
		action_logger.info(f'End of action thread: {self}')

	def is_paused(self):
		if self.current_action is not None:
			return self.current_action.is_paused()
		else:
			return False

	def pause(self):
		if self.current_action is not None:
			self.current_action.pause()

	def resume(self):
		if self.current_action is not None:
			self.current_action.resume()

	def abort(self):
		if self.current_action is not None:
			self.current_action.abort()


class ActionSet(ActionBase):
	name = ''
	actions = None
	action_idx = 0

	def __init__(self, name: str = '', actions: Iterable = []):
		super().__init__(self)
		self.name = name
		self.actions = list(actions)

	def __repr__(self):
		return self.name if self.name else f'ActionSet({self.actions})'

	def run(self):
		while self.action_idx < len(self.actions):
			next_action = self.actions[self.action_idx]
			self.run_action(next_action)
			self.check_for_interruption()
			if not next_action.abort:
				self.action_idx += 1


def skip_interrupt(interrupted_action: ActionBase):
	"""
	Example of interrupt handler that just skip the current interruption.
	"""
	action_logger.debug(f'Skipping {interrupted_action.current_interrupt} in action: {interrupted_action}.')
	return
