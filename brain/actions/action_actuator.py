import time

from ..engine_base.action_base import ActionBase
from ..events.actuator_events import EventActuatorServo, EventActuatorStepperParams, RequestActuatorStepperSteps


# For id from 0 to 4: (angle in closed position, angle in sligtly opened position)
claws_angles = [(35, 65), (120, 90), (89, 50), (100, 70), (90, 50)]
# claws_support_pos = None


class ActionHandsUp(ActionBase):

	def run(self):
		ev = EventActuatorServo(9, 60)
		self.parent_node.publish_event(ev)


class ActionHandsDown(ActionBase):

	def run(self):
		ev = EventActuatorServo(9, 125)
		self.parent_node.publish_event(ev)


class ActionCloseClaws(ActionBase):

	def __init__(self, claws: set, **kwargs):
		super().__init__(**kwargs)
		self.claws = claws

	def run(self):
		for claw_id in self.claws:
			if 0 <= claw_id < 5:
				ev = EventActuatorServo(claw_id, claws_angles[claw_id][0])
				self.parent_node.publish_event(ev)
				time.sleep(0.1)


class ActionOpenClaws(ActionBase):

	def __init__(self, claws: set, **kwargs):
		super().__init__(**kwargs)
		self.claws = claws

	def run(self):
		for claw_id in self.claws:
			if 0 <= claw_id < 5:
				ev = EventActuatorServo(claw_id, claws_angles[claw_id][1])
				self.parent_node.publish_event(ev)
				time.sleep(0.1)


class ActionMoveClawsSupportUp(ActionBase):

	def __init__(self, speed: int = 125, **kwargs):
		super().__init__(**kwargs)
		self.speed = speed

	def run(self):
		# global claws_support_pos
		# if claws_support_pos is not None:
		# 	travel = claws_support_pos - 100
		# else:
		# 	travel = 5000
		ev = EventActuatorStepperParams(1, 1, self.speed)
		self.parent_node.publish_event(ev)
		time.sleep(0.1)
		ev = RequestActuatorStepperSteps(5000)
		self.parent_node.publish_request(ev)
		ev = EventActuatorStepperParams(1, 1, 60)
		self.parent_node.publish_event(ev)
		time.sleep(0.1)
		ev = RequestActuatorStepperSteps(500)
		self.parent_node.publish_request(ev)
		# claws_support_pos = 0


class ActionMoveClawsSupportDown(ActionBase):
	def __init__(self, nb_steps: int, speed: int = 125, **kwargs):
		super().__init__(**kwargs)
		self.nb_steps = nb_steps
		self.speed = speed

	def run(self):
		# global claws_support_pos
		# if claws_support_pos is None:
		# 	self.run_action(ActionMoveClawsSupportUp(self.speed))
		ev = EventActuatorStepperParams(1, 0, self.speed)
		self.parent_node.publish_event(ev)
		time.sleep(0.1)
		ev = RequestActuatorStepperSteps(self.nb_steps - 200)
		rep = self.parent_node.publish_request(ev)
		ev = EventActuatorStepperParams(1, 0, 60)
		self.parent_node.publish_event(ev)
		time.sleep(0.1)
		ev = RequestActuatorStepperSteps(200)
		rep = self.parent_node.publish_request(ev)
		# claws_support_pos = self.nb_steps


class ActionDisableStepper(ActionBase):

	def run(self):
		ev = EventActuatorStepperParams(0, 1, 100)
		self.parent_node.publish_event(ev)
