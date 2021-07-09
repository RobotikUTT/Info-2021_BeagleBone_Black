from typing import Dict

from ..engine_base.event_base import EventBase, EventRequestBase, EventReplyBase


class EventActuatorServo(EventBase):

	def __init__(self, id_servo: int = 0, angle: int = 0, args_dict: Dict = None, **kwargs):
		super().__init__(**kwargs)
		if args_dict is not None:
			self.args_dict = args_dict
		else:
			self.args_dict = {
				'id_servo': id_servo,
				'angle': angle
			}


class EventActuatorStepperParams(EventBase):

	def __init__(self, enable_state: int = 0, direction: int = 0, speed_rpm: int = 120, args_dict: Dict = None, **kwargs):
		super().__init__(**kwargs)
		if args_dict is not None:
			self.args_dict = args_dict
		else:
			self.args_dict = {
				'enable_state': enable_state,
				'direction': direction,
				'speed_rpm': speed_rpm
			}


class RequestActuatorStepperSteps(EventRequestBase):

	def __init__(self, nb_steps: int = 0, args_dict: Dict = None, **kwargs):
		super().__init__(**kwargs)
		if args_dict is not None:
			self.args_dict = args_dict
		else:
			self.args_dict = {
				'nb_steps': nb_steps
			}


class ReplyActuatorStepperSteps(EventReplyBase):

	def __init__(self, nb_steps_done: int, **kwargs):
		super().__init__(**kwargs)
		self.nb_steps_done = nb_steps_done
