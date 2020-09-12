class EventBase():
	"""
	The base of all event classes.

	A node can define its own event type by creating a subclass.
	"""

	def __init__(self, src=None):
		self.src = src


class EventRequestBase(EventBase):
	"""
	The base class for synchronous/asynchronus request.
	"""

	def __init__(self, src=None, dst=None):
		super().__init__(src)
		self.dst = dst
		self.sync = False
		self.reply_q = None


class EventReplyBase(EventBase):
	"""
	The base class for synchronous/asynchronus request reply.
	"""

	def __init__(self, src=None, dst=None):
		super().__init__(src)
		self.dst = dst
