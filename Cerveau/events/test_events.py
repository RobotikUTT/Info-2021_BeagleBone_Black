from Cerveau.node_manager.event import EventBase


class EventTimer(EventBase):

	def __init__(self, count, src=None):
		super().__init__(src)
		self.count = count
