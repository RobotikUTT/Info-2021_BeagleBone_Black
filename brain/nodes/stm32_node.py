import time
import logging
from queue import Queue

from ..interfaces.interface_description.python_parser import framesList, framesByName
from ..engine_base.node_base import NodeBase, subscribe_to_event
from ..events.can_events import EventCanReceive, EventCanSend
from ..events.stm32_events import (STM32_MODES, FLOAT_PRECISION, EventStmNewCoderData, EventStmNewCurrentPos, EventStmNewCurrentPwm,
	EventStmNewCurrentSpeed, EventStmOrderComplete, EventStmRobotBlocked, EventStmSetParam,
	EventStmSetMode, EventStmSetPos, EventStmSetPID, RequestStmSetSpeed, RequestStmSetPWM, RequestStmGoToAngle, RequestStmGoTo, RequestStmRotate, ReplyStm)


stm_logger = logging.getLogger('stm')


class Stm32Node(NodeBase):

	coder_data = {
		'left_wheel_dist': 0,
		'right_wheel_dist': 0,
		'freshness': time.time()
	}
	current_pos = {
		'x': 0,
		'y': 0,
		'angle': 0,
		'freshness': time.time()
	}
	current_pwm = {
		'left_pwm': 0,
		'right_pwm': 0,
		'freshness': time.time()
	}
	current_speed = {
		'linear_speed': 0,
		'left_speed': 0,
		'right_speed': 0,
		'freshness': time.time()
	}
	pending_requests = None

	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.pending_requests = Queue()
		self.command_dispatcher = {
			5: self.recv_coder_data,
			18: self.recv_current_pos,
			19: self.recv_current_pwm,
			20: self.recv_current_speed,
			26: self.recv_order_complete,
			28: self.recv_robot_blocked
		}

	def recv_coder_data(self, frame, data_fields):
		Stm32Node.coder_data = {
			'left_wheel_dist': data_fields['left_wheel_dist'],
			'right_wheel_dist': data_fields['right_wheel_dist'],
			'freshness': time.time()
		}
		ev = EventStmNewCoderData(Stm32Node.coder_data)
		self.publish_event(ev)

	def recv_current_pos(self, frame, data_fields):
		Stm32Node.current_pos = {
			'x': data_fields['x'],
			'y': data_fields['y'],
			'angle': data_fields['angle'] / FLOAT_PRECISION,
			'freshness': time.time()
		}
		ev = EventStmNewCurrentPos(Stm32Node.current_pos)
		self.publish_event(ev)

	def recv_current_pwm(self, frame, data_fields):
		Stm32Node.current_pwm = {
			'left_pwm': data_fields['left_pwm'],
			'right_pwm': data_fields['right_pwm'],
			'freshness': time.time()
		}
		ev = EventStmNewCurrentPwm(Stm32Node.current_pwm)
		self.publish_event(ev)

	def recv_current_speed(self, frame, data_fields):
		Stm32Node.current_speed = {
			'linear_speed': data_fields['linear_speed'],
			'left_speed': data_fields['left_speed'],
			'right_speed': data_fields['right_speed'],
			'freshness': time.time()
		}
		ev = EventStmNewCurrentSpeed(Stm32Node.current_speed)
		self.publish_event(ev)

	def recv_order_complete(self, frame, data_fields):

		if not self.pending_requests.empty():
			req = self.pending_requests.get()
			req.reply_q.put(ReplyStm(src=self, dst=req.src))
			stm_logger.debug(f'Order compelted. {req}.')
		else:
			stm_logger.debug('Order compelted. Unknown request.')
		ev = EventStmOrderComplete()
		self.publish_event(ev)
		pass

	def recv_robot_blocked(self, frame, data_fields):
		ev = EventStmRobotBlocked()
		self.publish_event(ev)
		pass

	@subscribe_to_event(EventCanReceive, states=[f.cmd_id for f in framesList if f.source == 'stm'])
	def message_from_can(self, ev):
		func = self.command_dispatcher[ev.frame_type.cmd_id]
		func(ev.frame_type, ev.data_fields)

	@subscribe_to_event(EventStmSetMode)
	def set_mode(self, req):
		if req.args_dict['mode'] not in STM32_MODES:
			stm_logger.error(f'Invalid mode for STM32: {req.args_dict["mode"]}')
		else:
			if req.args_dict['mode'] == 'reset_order':
				while not self.pending_requests.empty():
					request = self.pending_requests.get()
					request.reply_q.put(ReplyStm(src=self, dst=request.src))
					stm_logger.debug(f'Order reset. {request}.')

			req.args_dict['mode'] = STM32_MODES.index(req.args_dict['mode'])
			frame = framesByName['set_stm_mode']
			ev = EventCanSend(frame, req.args_dict)
			self.publish_event(ev)

	@subscribe_to_event(EventStmSetParam)
	def set_stm_param(self, req):
		req.args_dict['max_angular_speed'] = int(req.args_dict['max_angular_speed'] * FLOAT_PRECISION)
		frame = framesByName['set_stm_param']
		ev = EventCanSend(frame, req.args_dict)
		self.publish_event(ev)

	@subscribe_to_event(EventStmSetPos)
	def set_pos(self, req):
		req.args_dict['angle'] = int(req.args_dict['angle'] * FLOAT_PRECISION)
		frame = framesByName['set_position']
		ev = EventCanSend(frame, req.args_dict)
		self.publish_event(ev)

	@subscribe_to_event(EventStmSetPID)
	def set_pid(self, req):
		req.args_dict['p'] = int(req.args_dict['p'] * FLOAT_PRECISION)
		req.args_dict['i'] = int(req.args_dict['i'] * FLOAT_PRECISION)
		req.args_dict['d'] = int(req.args_dict['d'] * FLOAT_PRECISION)
		if req.mode == 'LEFT':
			frame = framesByName['set_left_pid']
		elif req.mode == 'RIGHT':
			frame = framesByName['set_right_pid']
		else:
			frame = framesByName['set_both_pid']
		ev = EventCanSend(frame, req.args_dict)
		self.publish_event(ev)

	@subscribe_to_event(RequestStmGoToAngle)
	def go_to_angle(self, req):
		req.args_dict['angle'] = int(req.args_dict['angle'] * FLOAT_PRECISION)
		self.pending_requests.put(req)
		frame = framesByName['go_to_angle']
		ev = EventCanSend(frame, req.args_dict)
		self.publish_event(ev)

	@subscribe_to_event(RequestStmGoTo)
	def go_to(self, req):
		self.pending_requests.put(req)
		frame = framesByName['go_to']
		ev = EventCanSend(frame, req.args_dict)
		self.publish_event(ev)

	@subscribe_to_event(RequestStmRotate)
	def rotate(self, req):
		req.args_dict['angle'] = int(req.args_dict['angle'] * FLOAT_PRECISION)
		self.pending_requests.put(req)
		frame = framesByName['rotate']
		ev = EventCanSend(frame, req.args_dict)
		self.publish_event(ev)

	@subscribe_to_event(RequestStmSetSpeed)
	def set_speed(self, req):
		self.pending_requests.put(req)
		frame = framesByName['set_speed']
		ev = EventCanSend(frame, req.args_dict)
		self.publish_event(ev)

	@subscribe_to_event(RequestStmSetPWM)
	def set_pwm(self, req):
		self.pending_requests.put(req)
		frame = framesByName['set_pwm']
		ev = EventCanSend(frame, req.args_dict)
		self.publish_event(ev)
