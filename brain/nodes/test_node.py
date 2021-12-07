from ..engine_base.node_base import NodeBase, subscribe_to_event
from ..events.test_events import EventTimer
# TODO: Relative imports for intra-package imports are highly discouraged.

from threading import Thread
from time import sleep


class TestNode_1(NodeBase):

	def __init__(self, intervale: float = 2.0):
		super().__init__()
		self.intervale = intervale
		self.count = 0

	def start(self):
		super().start()
		t_name = self.__class__.__name__ + '_timer'
		t = Thread(name=t_name, target=self.timer, daemon=True)
		self.threads.append(t)
		t.start()

	def timer(self):
		while True:
			sleep(self.intervale)
			ev = EventTimer(self.count)
			self.publish_event(ev)
			self.count += 1


class TestNode_2(NodeBase):

	def __init__(self):
		super().__init__()

	@subscribe_to_event(EventTimer)
	def listen_for_timer(self, ev):
		with open('test.log', 'a') as f:
			f.write(f'Got timer event nb: {ev.count}\n')
