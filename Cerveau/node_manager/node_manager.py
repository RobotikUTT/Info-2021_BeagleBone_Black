from queue import Queue
from typing import Callable
from threading import Thread
import inspect
import logging

from event import EventBase, EventRequestBase, EventReplyBase


class NodeManager:
	# singleton
	_instance = None

	@classmethod
	def get_instance(cls_):
		if NodeManager._instance is None:
			NodeManager()
		return NodeManager._instance

	def __init__(self):
		if NodeManager._instance is None:
			NodeManager._instance = self
		else:
			return

		self.nodes = {}
		self.observers = {}

	def add_node(self, node_cls, args=(), kwargs={}):
		assert issubclass(node_cls, NodeBase)
		node = node_cls(*args, **kwargs)
		nodes = self.nodes.setdefault(node_cls, [])
		nodes.append(node)

	def start_nodes(self):
		for node in sum(self.nodes.values(), []):
			node.start()

	def register_observer(self, observer, ev_cls, states=None):
		assert issubclass(ev_cls, EventBase)
		ev_cls_observers = self.observers.setdefault(ev_cls, {})
		ev_cls_observers_states = ev_cls_observers.setdefault(observer, set())
		if states is not None:
			ev_cls_observers_states.update(states)

	def unregister_observer(self, observer, ev_cls):
		assert issubclass(ev_cls, EventBase)
		observers = self.observers.get(ev_cls, {})
		observers.pop(observer)

	def get_observers(self, ev: EventBase, state=None):
		observers = []
		for obs, states in self.observers.get(ev.__class__, {}).items():
			if state is None or not states or state in states:
				observers.append(obs)

		return observers

	def send_event_to_observers(self, ev: EventBase, state=None):
		obs = self.get_observers(ev, state)
		if not obs:
			logging.warning('Event lost: {} from {}: No observers for this event.'.format(
				ev.__class__.__name__, ev.src.__class__.__name__))

		for observer in obs:
			observer._sent_event(ev, state)


class NodeBase:
	_manager = NodeManager.get_instance()

	def __init__(self):
		self.threads = []
		self.events = Queue()
		self.event_handlers = {}
		self.stop_event_loop = False

		# Register handlers for this instance
		# and register itself as an observer to the AppManager
		for m_name, method in inspect.getmembers(self, inspect.ismethod):
			if hasattr(method, 'callers'):
				for ev_cls, states in m.callers.items():
					self.register_handler(ev_cls, method)
					self.observe_event(ev_cls, states)

	def start(self):
		"""
		Hook that is called after startup initialization is done.
		"""
		t_name = self.__class__.__name__ + '_event_loop'
		t = Thread(name=t_name, target=self._event_loop, daemon=True)
		self.threads.append(t)
		t.start()

	def observe_event(self, ev_cls, states=None):
		assert issubclass(ev_cls, EventBase)
		self._manager.register_observer(self, ev_cls, states)

	def unobserve_event(self, ev_cls):
		assert issubclass(ev_cls, EventBase)
		self._manager.unregister_observer(self, ev_cls)

	def register_handler(self, ev_cls, handler: Callable):
		assert issubclass(ev_cls, EventBase)
		handlers = self.event_handlers.setdefault(ev_cls, [])
		handlers.append(handler)

	def unregister_handler(self, ev_cls, handler: Callable):
		assert issubclass(ev_cls, EventBase)
		self.event_handlers[ev_cls].remove(handler)
		if not self.event_handlers[ev_cls]:
			del self.event_handlers[ev_cls]

	def get_handlers(self, ev: EventBase, state=None):
		"""Returns a list of handlers for the specific event.

		:param ev: The event to handle.
		:param state: The current state. ("dispatcher")
					If None is given, returns all handlers for the event.
					Otherwise, returns only handlers that are interested
					in the specified state.
					The default is None.
		"""
		ev_cls = ev.__class__
		handlers = self.event_handlers.get(ev_cls, [])
		if state is None:
			return handlers

		def test(h):
			if not hasattr(h, 'callers') or ev_cls not in h.callers:
				# dynamically registered handlers does not have
				# h.callers element for the event.
				return True
			states = h.callers[ev_cls].dispatchers
			if not states:
				# empty states means all states
				return True
			return state in states

		return filter(test, handlers)

	def _event_loop(self):
		while not self.stop_event_loop or not self.events.empty():
			ev, state = self.events.get()
			handlers = self.get_handlers(ev, state)
			for handler in handlers:
				handler(ev)

	def _send_event(self, ev: EventBase, state):
		self.events.put((ev, state))

	def send_event(self, ev: EventBase, state=None):
		"""
		Send the specified event to the node manager for dispatching.
		"""

		if ev.src is None:
			ev.src = self
		self._manager.send_event_to_observers(ev, state)

	def send_request(self, req: EventRequestBase):
		"""
		Make a synchronous request.
		Set req.sync to True, send it and block until receiving a reply.
		Returns the received reply.
		"""

		req.sync = True
		req.reply_q = Queue()
		self.send_event(, req)
		# Going to sleep for the reply because Queue().get() block until not empty.
		return req.reply_q.get()

	def reply_to_request(self, req: EventRequestBase, rep: EventReplyBase):
		"""
		Send a reply for a synchronous request sent by send_request.
		"""

		rep.dst = req.src
		if req.sync:
			if rep.src is None:
				rep.src = self
			req.reply_q.put(rep)
		else:
			self.send_event(rep)

	# If event are to slow: implement a synchrone fast_request function that call the manager
	# which call a synchrone fast_reply on node. Thus, the fasy_reply function is executed by
	# the fast_request thread.
	# That should be used carefuly to avoid conflict with the reply node threads
	# (I suggest to do that mostly only for attibute getters)
