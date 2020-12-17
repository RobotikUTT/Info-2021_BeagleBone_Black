from collections.abc import Iterable


def observe_event(ev_cls, states=None):
	"""
	A decorator for node to declare an event handler.

	Decorated method will become an event handler.
	ev_cls is an event class (or a list of event class) whose instances of this
	node want to receive.
	states is an optional argument specifies one or a list of states permiting a smaler
	granularity to filter events that shoud be handle to this handler.
	Note that, in case an event changes the state, the state before the change
	is used to check the interest.
	"""
	def _observe_event_dec(handler_func):
		if not hasattr(handler_func, 'callers'):
			handler_func.callers = {}
		for e in _listify(ev_cls):
			handler_func.callers[e] = _listify(states)
		return handler_func
	return _observe_event_dec


def _listify(may_list):
	if may_list is None:
		may_list = []
	elif not isinstance(may_list, list):
		if isinstance(may_list, Iterable):
			may_list = list(may_list)
		else:
			may_list = [may_list]
	return may_list
