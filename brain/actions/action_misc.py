import time

from ..engine_base.action_base import ActionBase
from ..engine_base.node_base import NodeManager
from .action_goto import Toto


class ActionSleep(ActionBase):

	atomic_sleep_duration = 0.01

	def __init__(self, sleep_duration: int = 1, **kwargs):
		super().__init__(**kwargs)
		self.sleep_duration = sleep_duration

	def run(self):
		for i in range(int(self.sleep_duration / self.atomic_sleep_duration)):
			self.check_for_interruption()
			time.sleep(self.atomic_sleep_duration)
		self.check_for_interruption()
		time.sleep(self.sleep_duration % self.atomic_sleep_duration)


class ActionShutdown(ActionBase):

	def run(self):
		NodeManager.get_instance().shutdown.set()


class ActionDisableSonar(ActionBase):

	def run(self):
		Toto.get_instance().DISABLE_SONAR = True
