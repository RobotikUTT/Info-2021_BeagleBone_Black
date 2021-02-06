from ..engine_base.event_base import EventBase, EventRequestBase, EventReplyBase

from typing import Dict


class EventStmNewCoderData(EventBase):

	def __init__(self, coder_data: Dict, src=None):
		super().__init__(src)
		self.coder_data = coder_data


class EventStmNewCurrentPos(EventBase):

	def __init__(self, current_pos: Dict, src=None):
		super().__init__(src)
		self.current_pos = current_pos


class EventStmNewCurrentPwm(EventBase):

	def __init__(self, current_pwm: Dict, src=None):
		super().__init__(src)
		self.current_pwm = current_pwm


class EventStmNewCurrentSpeed(EventBase):

	def __init__(self, current_speed: Dict, src=None):
		super().__init__(src)
		self.current_speed = current_speed


class EventStmOrderComplete(EventBase):

	pass


class EventStmRobotBlocked(EventBase):

	pass


class RequestStmSetSpeed(EventRequestBase):

	def __init__(self, linear_speed: int = 0, angular_speed: int = 0, duration: int = 2000,
		wanted_speed: Dict = None, src=None):
		super().__init__(src)
		if wanted_speed is not None:
			self.wanted_speed = wanted_speed
		else:
			self.wanted_speed = {
				'linear_speed': linear_speed,
				'angular_speed': angular_speed,
				'duration': duration
			}


class ReplyStm(EventReplyBase):

	pass
