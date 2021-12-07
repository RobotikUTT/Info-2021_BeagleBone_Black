from ..engine_base.event_base import EventBase
# TODO: Relative imports for intra-package imports are highly discouraged.

class EventTimer(EventBase):

	def __init__(self, count, src=None):
		super().__init__(src)
		self.count = count
