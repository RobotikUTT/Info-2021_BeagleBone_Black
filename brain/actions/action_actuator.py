from ..engine_base.action_base import ActionBase
from ..events.actuator_events import EventActuatorServo


class ActionHandsUp(ActionBase):

	def run(self):
		ev = EventActuatorServo(9, 60)
		self.parent_node.publish_event(ev)


class ActionHandsDown(ActionBase):

	def run(self):
		ev = EventActuatorServo(9, 125)
		self.parent_node.publish_event(ev)


class ActionCloseClaws(ActionBase):

	def __init__(self, claws: list, **kwargs):
		super().__init__(**kwargs)
