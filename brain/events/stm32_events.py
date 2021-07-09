from typing import Dict

from ..engine_base.event_base import EventBase, EventRequestBase, EventReplyBase


STM32_MODES = [
	'stop',
	'start',
	'pause',
	'resume',
	'reset_id',
	'set_emergency_stop',
	'next_order',
	'reset_order',
	'reset_emergency_stop'
]

FLOAT_PRECISION = 1000.0

DIRECTION_ANY = 0
DIRECTION_FORWARD = 1
DIRECTION_BACKWARD = -1


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


class EventStmSetMode(EventBase):

	def __init__(self, mode: str = '', args_dict: Dict = None, **kwargs):
		super().__init__(**kwargs)
		if args_dict is not None:
			self.args_dict = args_dict
		else:
			self.args_dict = {
				'mode': mode
			}


class EventStmSetParam(EventBase):

	def __init__(self, max_linear_speed: int = 1000, max_angular_speed: float = 0.6, max_acc: int = 1500, args_dict: Dict = None, **kwargs):
		super().__init__(**kwargs)
		if args_dict is not None:
			self.args_dict = args_dict
		else:
			self.args_dict = {
				'max_linear_speed': max_linear_speed,
				'max_angular_speed': max_angular_speed,
				'max_acc': max_acc
			}


class EventStmSetPos(EventBase):

	def __init__(self, x: int = 0, y: int = 0, angle: float = 0.0, args_dict: Dict = None, **kwargs):
		super().__init__(**kwargs)
		if args_dict is not None:
			self.args_dict = args_dict
		else:
			self.args_dict = {
				'x': x,
				'y': y,
				'angle': angle
			}


class EventStmSetPID(EventBase):

	def __init__(self, p: float = 0.0, i: float = 0.0, d: float = 0.0, mode='BOTH', args_dict: Dict = None, **kwargs):
		super().__init__(**kwargs)
		if mode not in ('LEFT', 'RIGHT', 'BOTH'):
			mode = 'BOTH'
		self.mode = mode
		if args_dict is not None:
			self.args_dict = args_dict
		else:
			self.args_dict = {
				'p': p,
				'i': i,
				'd': d
			}


class RequestStmSetSpeed(EventRequestBase):

	def __init__(self, linear_speed: int = 0, angular_speed: int = 0, duration: int = 2000,
		args_dict: Dict = None, **kwargs):
		super().__init__(**kwargs)
		if args_dict is not None:
			self.args_dict = args_dict
		else:
			self.args_dict = {
				'linear_speed': linear_speed,
				'angular_speed': angular_speed,
				'duration': duration
			}


class RequestStmSetPWM(EventRequestBase):

	def __init__(self, left_pwm: int = 0, right_pwm: int = 0, duration: int = 2000,
		args_dict: Dict = None, **kwargs):
		super().__init__(**kwargs)
		if args_dict is not None:
			self.args_dict = args_dict
		else:
			self.args_dict = {
				'left_pwm': left_pwm,
				'right_pwm': right_pwm,
				'duration': duration
			}


class RequestStmGoToAngle(EventRequestBase):

	def __init__(self, x: int = 0, y: int = 0, angle: float = 0.0,
		direction: int = 0, args_dict: Dict = None, **kwargs):
		super().__init__(**kwargs)
		if args_dict is not None:
			self.args_dict = args_dict
		else:
			self.args_dict = {
				'x': x,
				'y': y,
				'angle': angle,
				'direction': direction
			}


class RequestStmGoTo(EventRequestBase):

	def __init__(self, x: int = 0, y: int = 0, direction: int = 0, args_dict: Dict = None, **kwargs):
		super().__init__(**kwargs)
		if args_dict is not None:
			self.args_dict = args_dict
		else:
			self.args_dict = {
				'x': x,
				'y': y,
				'direction': direction
			}


class RequestStmRotate(EventRequestBase):

	def __init__(self, angle: float = 0.0, args_dict: Dict = None, **kwargs):
		super().__init__(**kwargs)
		if args_dict is not None:
			self.args_dict = args_dict
		else:
			self.args_dict = {
				'angle': angle
			}


class ReplyStm(EventReplyBase):

	pass
