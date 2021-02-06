import time
from queue import Queue

from ..interfaces.interface_description.python_parser import framesList, framesByName
from ..engine_base.node_base import NodeBase, subscribe_to_event
from ..events.can_events import EventCanReceive, EventCanSend
from ..events.stm32_events import (EventStmNewCoderData, EventStmNewCurrentPos, EventStmNewCurrentPwm,
	EventStmNewCurrentSpeed, EventStmOrderComplete, EventStmRobotBlocked, RequestStmSetSpeed, ReplyStm)


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
	pending_requests = Queue()

	def recv_coder_data(self, frame, data_fields):
		self.coder_data = {
			'left_wheel_dist': data_fields['left_wheel_dist'],
			'right_wheel_dist': data_fields['right_wheel_dist'],
			'freshness': time.time()
		}
		ev = EventStmNewCoderData(self.coder_data)
		self.publish_event(ev)

	def recv_current_pos(self, frame, data_fields):
		self.current_pos = {
			'x': data_fields['x'],
			'y': data_fields['y'],
			'angle': data_fields['angle'],
			'freshness': time.time()
		}
		ev = EventStmNewCurrentPos(self.current_pos)
		self.publish_event(ev)

	def recv_current_pwm(self, frame, data_fields):
		self.current_pwm = {
			'left_pwm': data_fields['left_pwm'],
			'right_pwm': data_fields['right_pwm'],
			'freshness': time.time()
		}
		ev = EventStmNewCurrentPwm(self.current_pwm)
		self.publish_event(ev)

	def recv_current_speed(self, frame, data_fields):
		self.current_speed = {
			'linear_speed': data_fields['linear_speed'],
			'left_speed': data_fields['left_speed'],
			'right_speed': data_fields['right_speed'],
			'freshness': time.time()
		}
		ev = EventStmNewCurrentSpeed(self.current_speed)
		self.publish_event(ev)

	def recv_order_complete(self, frame, data_fields):
		if not self.pending_requests.empty():
			req = self.pending_requests.get()
			req.reply_q.put(ReplyStm(src=self, dst=req.src))
		ev = EventStmOrderComplete()
		self.publish_event(ev)
		pass

	def recv_robot_blocked(self, frame, data_fields):
		ev = EventStmRobotBlocked()
		self.publish_event(ev)
		pass

	@subscribe_to_event(EventCanReceive, states=[f.cmd_id for f in framesList if f.source == 'stm'])
	def message_from_can(self, ev):
		command_dispatcher = {
			5: self.recv_coder_data,
			18: self.recv_current_pos,
			19: self.recv_current_pwm,
			20: self.recv_current_speed,
			26: self.recv_order_complete,
			28: self.recv_robot_blocked
		}
		func = command_dispatcher[ev.frame_type.cmd_id]
		func(ev.frame_type, ev.data_fields)

	@subscribe_to_event(RequestStmSetSpeed)
	def set_speed(self, req):
		self.pending_requests.put(req)
		frame = framesByName['set_speed']
		ev = EventCanSend(frame, req.wanted_speed)
		self.publish_event(ev)
