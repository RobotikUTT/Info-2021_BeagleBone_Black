from queue import Queue
from typing import Callable
from threading import Thread
import inspect
import logging
from collections.abc import Iterable

from .event_base import EventBase, EventRequestBase, EventReplyBase


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
		self.subscriptions = {}

	def add_node(self, node_cls, args=(), kwargs={}):
		assert issubclass(node_cls, NodeBase)
		node = node_cls(*args, **kwargs)
		nodes = self.nodes.setdefault(node_cls, [])
		nodes.append(node)

	def get_all_nodes(self):
		for node in sum(self.nodes.values(), []):
			yield node

	def start_nodes(self):
		for node in self.get_all_nodes():
			node.start()

	def subscribe_to_event(self, subscriber, ev_cls, states=None):
		assert issubclass(ev_cls, EventBase)
		ev_cls_subscribers = self.subscriptions.setdefault(ev_cls, {})
		subscriber_states = ev_cls_subscribers.setdefault(subscriber, set())
		if states is not None:
			subscriber_states.update(states)

	def unsubscribe_to_event(self, subscriber, ev_cls):
		assert issubclass(ev_cls, EventBase)
		ev_cls_subscribers = self.subscriptions.get(ev_cls, {})
		if subscriber in ev_cls_subscribers:
			ev_cls_subscribers.pop(subscriber)
		else:
			logging.warning('Unsuscribe to event failed: {} is not a subscriber of {}.'.format(
				subscriber.__class__.__name__, ev_cls.__name__))

	def get_subscribers_for_event(self, ev_cls, state=None):
		subscribers = []
		for subscriber, states in self.subscriptions.get(ev_cls, {}).items():
			if state is None or not states or state in states:
				subscribers.append(subscriber)

		return subscribers

	def send_event_to_subscribers(self, ev: EventBase, state=None):
		subscribers = self.get_subscribers_for_event(ev.__class__, state)
		if not subscribers:
			logging.warning('Event lost: {} from {}: No subscribers for this event.'.format(
				ev.__class__.__name__, ev.src.__class__.__name__))

		for subscriber in subscribers:
			subscriber.send_event(ev, state)


class NodeBase:
	_manager = NodeManager.get_instance()

	def __init__(self):
		self.threads = []
		self.events_queue = Queue()
		self.event_handlers = {}
		self.stop_event_loop = False

		# Register event handlers for this instance
		# and register itself as a subscriber to the NodeManager
		for m_name, method in inspect.getmembers(self, inspect.ismethod):
			if hasattr(method, 'callers'):
				for ev_cls, states in method.callers.items():
					self.register_event_handler(ev_cls, method)
					self._manager.subscribe_to_event(self, ev_cls, states)
					# self.subscribe_to_event(ev_cls, states)		# Looks pretty useless to me

	def _event_loop(self):
		while not self.stop_event_loop or not self.events_queue.empty():
			ev, state = self.events_queue.get()
			handlers = self.get_handlers_for_event(ev, state)
			for handler in handlers:
				handler(ev)

	def start(self):
		"""
		Hook that is called after startup initialization is done.
		Can be overrided.
		"""
		t_name = self.__class__.__name__ + '_event_loop'
		t = Thread(name=t_name, target=self._event_loop, daemon=True)
		self.threads.append(t)
		t.start()

	# Looks pretty useless to me
	# def subscribe_to_event(self, ev_cls, states=None):
	# 	assert issubclass(ev_cls, EventBase)
	# 	self._manager.subscribe_to_event(self, ev_cls, states)

	# def unsubscribe_to_event(self, ev_cls):
	# 	assert issubclass(ev_cls, EventBase)
	# 	self._manager.unsubscribe_to_event(self, ev_cls)

	def register_event_handler(self, ev_cls, handler: Callable):
		assert issubclass(ev_cls, EventBase)
		handlers = self.event_handlers.setdefault(ev_cls, [])
		handlers.append(handler)

	def unregister_event_handler(self, ev_cls, handler: Callable):
		assert issubclass(ev_cls, EventBase)
		handlers = self.event_handlers.get(handler)
		if handlers is None:
			logging.warning('Unable to unregister event handler "{}" from "{}": No handlers for this event.'.format(
				handler.__name__, self.__class__.__name__))
		elif handler not in handlers:
			logging.warning('Unable to unregister event handler "{}" from "{}": Not in handlers list for this event.'.format(
				handler.__name__, self.__class__.__name__))
		else:
			handlers.remove(handler)
			if not self.event_handlers[ev_cls]:
				del self.event_handlers[ev_cls]

	def get_handlers_for_event(self, ev: EventBase, state=None):
		"""Returns a list of event handlers for the specific event.

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

	def send_event(self, ev: EventBase, state):
		self.events_queue.put((ev, state))

	def publish_event(self, ev: EventBase, state=None):
		"""
		Publish the specified event to the node manager for dispatching.
		"""

		if ev.src is None:
			ev.src = self
		self._manager.send_event_to_subscribers(ev, state)

	def publish_request(self, req: EventRequestBase):
		"""
		Make a synchronous request.
		Set req.sync to True, publish it and block until receiving a reply.
		Returns the received reply.
		"""

		req.reply_q = Queue()
		self.publish_event(req)
		if req.sync:
			# Going to sleep for the reply because Queue().get() block until not empty.
			return req.reply_q.get()

	def check_for_reply(self, req: EventRequestBase):
		if isinstance(req.reply_q, Queue):
			return not req.reply_q.empty()
		return False

	def get_reply(self, req: EventRequestBase):
		if self.check_for_reply(req):
			return req.reply_q.get()
		return None

	def reply_to_request(self, req: EventRequestBase, rep: EventReplyBase):
		"""
		Send a reply for a synchronous request sent by publish_request.
		"""

		rep.dst = req.src
		if req.sync:
			if rep.src is None:
				rep.src = self
			req.reply_q.put(rep)
		else:
			self.publish_event(rep)

	# If events are to slow: implement a synchrone fast_request function that call the manager
	# which call a synchrone fast_reply on node. Thus, the fasy_reply function is executed by
	# the fast_request thread.
	# That should be used carefuly to avoid conflict with the reply node threads
	# (I suggest to do that mostly only for attibute getters)


def subscribe_to_event(ev_cls, states=None):
	"""
	A decorator for node to declare an event handler inside a node class definition.

	Decorated method will become an event handler.
	ev_cls is an event class (or a list of event class) whose instances of this
	node want to receive.
	states is an optional argument specifies one or a list of states permiting a smaller
	granularity to filter events that shoud be handled by this handler.
	Note that, in case an event changes the state, the state before the change
	is used to check the interest.
	"""
	def _subscribe_to_event_dec(handler_func):
		if not hasattr(handler_func, 'callers'):
			handler_func.callers = {}
		for e in _listify(ev_cls):
			handler_func.callers[e] = _listify(states)
		return handler_func
	return _subscribe_to_event_dec


def _listify(may_list):
	if may_list is None:
		may_list = []
	elif not isinstance(may_list, list):
		if isinstance(may_list, Iterable):
			may_list = list(may_list)
		else:
			may_list = [may_list]
	return may_list
