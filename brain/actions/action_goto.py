import logging
import time
from math import pi

from ..engine_base.action_base import ActionBase, InterruptBase
from ..engine_base.node_base import subscribe_to_event
from ..nodes.stm32_node import Stm32Node
from ..events.sonar_events import EventSonarDist, RequestSonarDist
from ..events.stm32_events import (DIRECTION_FORWARD, DIRECTION_BACKWARD, DIRECTION_ANY, EventStmRobotBlocked, EventStmSetParam,
	EventStmSetMode, EventStmSetPos, EventStmSetPID, RequestStmGoToAngle, RequestStmGoTo, RequestStmSetSpeed, RequestStmSetPWM, RequestStmRotate)


goto_logger = logging.getLogger('goto')


SONAR_MIN_STOP_DELAY = 1


class Toto():

	# singleton
	_instance = None
	DISABLE_SONAR = False

	@classmethod
	def get_instance(cls_):
		if Toto._instance is None:
			Toto()
		return Toto._instance

	def __init__(self):
		if Toto._instance is None:
			Toto._instance = self
		else:
			return


def bypass_sonar():
	if Toto.get_instance().DISABLE_SONAR:
		return True

	LOWER_BORDER_X = 0
	UPPER_BORDER_X = 2000
	LOWER_BORDER_Y = 0
	UPPER_BORDER_Y = 3000

	BYPASS_WIDTH = 250

	# On X
	x, y, angle = Stm32Node.current_pos['x'], Stm32Node.current_pos['y'], Stm32Node.current_pos['angle']
	if (x < LOWER_BORDER_X + BYPASS_WIDTH and not (-pi * 2 / 3 < angle < pi * 2 / 3)) or (x > UPPER_BORDER_X - BYPASS_WIDTH and (-pi / 3 < angle < pi / 3)):
		return True
	if (y < LOWER_BORDER_Y + BYPASS_WIDTH and (-pi * 2 / 3 < angle < -pi / 3)) or (y > UPPER_BORDER_Y - BYPASS_WIDTH and (pi / 3 < angle < pi * 2 / 3)):
		return True
	return False


class ActionAsservSetup(ActionBase):

	def __init__(self, start_pos=None, PID_left=None, PID_right=None, speed_params=None, **kwargs):
		super().__init__(**kwargs)
		self.start_pos = start_pos
		self.PID_left = PID_left
		self.PID_right = PID_right
		self.speed_params = speed_params

	def run(self):
		ev = EventStmSetMode(mode='start')
		self.parent_node.publish_event(ev)
		ev = EventStmSetMode(mode='resume')
		self.parent_node.publish_event(ev)
		ev = EventStmSetMode(mode='reset_emergency_stop')
		self.parent_node.publish_event(ev)
		ev = EventStmSetMode(mode='reset_order')
		self.parent_node.publish_event(ev)
		ev = EventStmSetMode(mode='reset_id')
		self.parent_node.publish_event(ev)

		if self.start_pos is not None:
			ev = EventStmSetPos(*self.start_pos)
			self.parent_node.publish_event(ev)
		if self.PID_left is not None:
			ev = EventStmSetPID(*self.PID_left, mode='LEFT')
			self.parent_node.publish_event(ev)
		if self.PID_right is not None:
			ev = EventStmSetPID(*self.PID_right, mode='RIGHT')
			self.parent_node.publish_event(ev)

		if self.speed_params is not None:
			ev = EventStmSetParam(*self.speed_params)
			self.parent_node.publish_event(ev)


class ActionAsservStop(ActionBase):

	def run(self):
		ev = EventStmSetMode(mode='pause')
		self.parent_node.publish_event(ev)
		ev = EventStmSetMode(mode='reset_order')
		self.parent_node.publish_event(ev)
		ev = EventStmSetMode(mode='reset_id')
		self.parent_node.publish_event(ev)
		ev = EventStmSetMode(mode='stop')
		self.parent_node.publish_event(ev)


class ActionPWM(ActionBase):

	def __init__(self, left_pwm: int = 0, right_pwm: int = 0, duration: int = 2000, **kwargs):
		super().__init__(**kwargs)
		self.left_pwm = left_pwm
		self.right_pwm = right_pwm
		self.duration = duration

		self.emergency = False
		self.emergency_time = time.time()
		self.triggered_sonars = 0
		self.add_interrupt_handler(SonarEmergencySetInterrupt, self.sonar_emergency_handler_set)
		self.add_interrupt_handler(SonarEmergencyClearInterrupt, self.sonar_emergency_handler_clear)

	def run(self):
		self.parent_node.register_event_handler(EventSonarDist, self.handle_sonar_alert)
		self.parent_node._manager.subscribe_to_event(self.parent_node, EventSonarDist)

		self.triggered_sonars = 0
		rep = self.parent_node.publish_request(RequestSonarDist())
		if self.left_pwm > 0 or self.right_pwm > 0:
			if not rep.sonars_states_ok['dist_front_left']:
				self.triggered_sonars += 1
			if not rep.sonars_states_ok['dist_front_right']:
				self.triggered_sonars += 1
		if self.left_pwm < 0 or self.right_pwm < 0:
			if not rep.sonars_states_ok['dist_back_left']:
				self.triggered_sonars += 1
			if not rep.sonars_states_ok['dist_back_right']:
				self.triggered_sonars += 1

		self.start_time = time.time()
		self.sub_run()
		ev = EventStmSetMode(mode='set_emergency_stop')
		self.parent_node.publish_event(ev)
		ev = EventStmSetMode(mode='reset_order')
		self.parent_node.publish_event(ev)
		req = RequestStmSetPWM(0, 0, 2000, sync=False)
		self.parent_node.publish_request(req)
		time.sleep(0.2)
		ev = EventStmSetMode(mode='reset_emergency_stop')
		self.parent_node.publish_event(ev)

		self.parent_node.unregister_event_handler(EventSonarDist, self.handle_sonar_alert)
		self.parent_node._manager.unsubscribe_to_event(self.parent_node, EventSonarDist)

	def sub_run(self):
		# dur = max(50, int(self.duration - (time.time() - self.start_time) * 1000))
		# dur = self.duration
		req = RequestStmSetPWM(self.left_pwm, self.right_pwm, 2000, sync=False)
		self.parent_node.publish_request(req)

		# while not self.parent_node.check_for_reply(req):
		# 	self.check_for_interruption()
		while time.time() - self.start_time < self.duration:
			self.check_for_interruption()
			time.sleep(0.02)

		if self.emergency:
			self.emergency = False
			self.run()

	def sonar_emergency_handler_set(self, action):
		ev = EventStmSetMode(mode='set_emergency_stop')
		action.parent_node.publish_event(ev)
		ev = EventStmSetMode(mode='reset_order')
		action.parent_node.publish_event(ev)
		self.emergency_time = time.time()
		goto_logger.info('Emergency stop set !')
		action.pause()

	def sonar_emergency_handler_clear(self, action):
		delta = time.time() - self.emergency_time
		if delta < SONAR_MIN_STOP_DELAY:
			time.sleep(SONAR_MIN_STOP_DELAY - delta)
		ev = EventStmSetMode(mode='reset_emergency_stop')
		action.parent_node.publish_event(ev)
		goto_logger.info('Emergency stop clear.')
		action.resume()

	# @subscribe_to_event(EventSonarDist)
	def handle_sonar_alert(self, ev):
		if self.left_pwm > 0 or self.right_pwm > 0:
			if ev.sonar == 'dist_front_left' or ev.sonar == 'dist_front_right':
				if not ev.state_ok:
					if not bypass_sonar():
						self.triggered_sonars += 1
						goto_logger.info(f'trig sonar: {self.triggered_sonars}')
						if not self.emergency:
							self.emergency = True
							self.interrupt(SonarEmergencySetInterrupt())
				else:
					self.triggered_sonars -= 1
					goto_logger.info(f'trig sonar: {self.triggered_sonars}')
					if not self.triggered_sonars:
						self.interrupt(SonarEmergencyClearInterrupt())
		if self.left_pwm < 0 or self.right_pwm < 0:
			if ev.sonar == 'dist_back_left' or ev.sonar == 'dist_back_right':
				if not ev.state_ok:
					if not bypass_sonar():
						self.triggered_sonars += 1
						goto_logger.info(f'trig sonar: {self.triggered_sonars}')
						if not self.emergency:
							self.emergency = True
							self.interrupt(SonarEmergencySetInterrupt())
				else:
					self.triggered_sonars -= 1
					goto_logger.info(f'trig sonar: {self.triggered_sonars}')
					if not self.triggered_sonars:
						self.interrupt(SonarEmergencyClearInterrupt())


class ActionGotoAngle(ActionBase):

	def __init__(self, x: int = 0, y: int = 0, angle: float = 0.0, direction: int = 0, **kwargs):
		super().__init__(**kwargs)
		self.x = x
		self.y = y
		self.angle = angle
		self.direction = direction

		self.emergency = False
		self.emergency_time = time.time()
		self.triggered_sonars = 0
		self.add_interrupt_handler(SonarEmergencySetInterrupt, self.sonar_emergency_handler_set)
		self.add_interrupt_handler(SonarEmergencyClearInterrupt, self.sonar_emergency_handler_clear)

	def run(self):
		self.parent_node.register_event_handler(EventSonarDist, self.handle_sonar_alert)
		self.parent_node._manager.subscribe_to_event(self.parent_node, EventSonarDist)

		self.triggered_sonars = 0
		rep = self.parent_node.publish_request(RequestSonarDist())
		if self.direction == DIRECTION_FORWARD or self.direction == DIRECTION_ANY:
			if not rep.sonars_states_ok['dist_front_left']:
				self.triggered_sonars += 1
			if not rep.sonars_states_ok['dist_front_right']:
				self.triggered_sonars += 1
		if self.direction == DIRECTION_BACKWARD or self.direction == DIRECTION_ANY:
			if not rep.sonars_states_ok['dist_back_left']:
				self.triggered_sonars += 1
			if not rep.sonars_states_ok['dist_back_right']:
				self.triggered_sonars += 1

		self.start_time = time.time()
		self.sub_run()

		self.parent_node.unregister_event_handler(EventSonarDist, self.handle_sonar_alert)
		self.parent_node._manager.unsubscribe_to_event(self.parent_node, EventSonarDist)

	def sub_run(self):
		req = RequestStmGoToAngle(self.x, self.y, self.angle, self.direction, sync=False)
		self.parent_node.publish_request(req)

		while not self.parent_node.check_for_reply(req):
			self.check_for_interruption()

		if self.emergency:
			self.emergency = False
			self.run()

	def sonar_emergency_handler_set(self, action):
		ev = EventStmSetMode(mode='set_emergency_stop')
		action.parent_node.publish_event(ev)
		ev = EventStmSetMode(mode='reset_order')
		action.parent_node.publish_event(ev)
		self.emergency_time = time.time()
		goto_logger.info('Emergency stop set !')
		action.pause()

	def sonar_emergency_handler_clear(self, action):
		delta = time.time() - self.emergency_time
		if delta < SONAR_MIN_STOP_DELAY:
			time.sleep(SONAR_MIN_STOP_DELAY - delta)
		ev = EventStmSetMode(mode='reset_emergency_stop')
		action.parent_node.publish_event(ev)
		goto_logger.info('Emergency stop clear.')
		action.resume()

	# @subscribe_to_event(EventSonarDist)
	def handle_sonar_alert(self, ev):
		if self.direction == DIRECTION_FORWARD or self.direction == DIRECTION_ANY:
			if ev.sonar == 'dist_front_left' or ev.sonar == 'dist_front_right':
				if not ev.state_ok:
					if not bypass_sonar():
						self.triggered_sonars += 1
						if not self.emergency:
							self.emergency = True
							self.interrupt(SonarEmergencySetInterrupt())
				else:
					self.triggered_sonars -= 1
					if not self.triggered_sonars:
						self.interrupt(SonarEmergencyClearInterrupt())
		if self.direction == DIRECTION_BACKWARD or self.direction == DIRECTION_ANY:
			if ev.sonar == 'dist_back_left' or ev.sonar == 'dist_back_right':
				if not ev.state_ok:
					if not bypass_sonar():
						self.triggered_sonars += 1
						if not self.emergency:
							self.emergency = True
							self.interrupt(SonarEmergencySetInterrupt())
				else:
					self.triggered_sonars -= 1
					if not self.triggered_sonars:
						self.interrupt(SonarEmergencyClearInterrupt())


class ActionGoto(ActionBase):

	def __init__(self, x: int = 0, y: int = 0, direction: int = 0, **kwargs):
		super().__init__(**kwargs)
		self.x = x
		self.y = y
		self.direction = direction

		self.emergency = False
		self.emergency_time = time.time()
		self.triggered_sonars = 0
		self.add_interrupt_handler(SonarEmergencySetInterrupt, self.sonar_emergency_handler_set)
		self.add_interrupt_handler(SonarEmergencyClearInterrupt, self.sonar_emergency_handler_clear)

	def run(self):
		self.parent_node.register_event_handler(EventSonarDist, self.handle_sonar_alert)
		self.parent_node._manager.subscribe_to_event(self.parent_node, EventSonarDist)

		self.triggered_sonars = 0
		rep = self.parent_node.publish_request(RequestSonarDist())
		if self.direction == DIRECTION_FORWARD or self.direction == DIRECTION_ANY:
			if not rep.sonars_states_ok['dist_front_left']:
				self.triggered_sonars += 1
			if not rep.sonars_states_ok['dist_front_right']:
				self.triggered_sonars += 1
		if self.direction == DIRECTION_BACKWARD or self.direction == DIRECTION_ANY:
			if not rep.sonars_states_ok['dist_back_left']:
				self.triggered_sonars += 1
			if not rep.sonars_states_ok['dist_back_right']:
				self.triggered_sonars += 1

		self.start_time = time.time()
		self.sub_run()

		self.parent_node.unregister_event_handler(EventSonarDist, self.handle_sonar_alert)
		self.parent_node._manager.unsubscribe_to_event(self.parent_node, EventSonarDist)

	def sub_run(self):
		req = RequestStmGoTo(self.x, self.y, self.direction, sync=False)
		self.parent_node.publish_request(req)

		while not self.parent_node.check_for_reply(req):
			self.check_for_interruption()

		if self.emergency:
			self.emergency = False
			self.run()

	def sonar_emergency_handler_set(self, action):
		ev = EventStmSetMode(mode='set_emergency_stop')
		action.parent_node.publish_event(ev)
		ev = EventStmSetMode(mode='reset_order')
		action.parent_node.publish_event(ev)
		self.emergency_time = time.time()
		goto_logger.info('Emergency stop set !')
		action.pause()

	def sonar_emergency_handler_clear(self, action):
		delta = time.time() - self.emergency_time
		if delta < SONAR_MIN_STOP_DELAY:
			time.sleep(SONAR_MIN_STOP_DELAY - delta)
		ev = EventStmSetMode(mode='reset_emergency_stop')
		action.parent_node.publish_event(ev)
		goto_logger.info('Emergency stop clear.')
		action.resume()

	# @subscribe_to_event(EventSonarDist)
	def handle_sonar_alert(self, ev):
		if self.direction == DIRECTION_FORWARD or self.direction == DIRECTION_ANY:
			if ev.sonar == 'dist_front_left' or ev.sonar == 'dist_front_right':
				if not ev.state_ok:
					if not bypass_sonar():
						self.triggered_sonars += 1
						if not self.emergency:
							self.emergency = True
							self.interrupt(SonarEmergencySetInterrupt())
				else:
					self.triggered_sonars -= 1
					if not self.triggered_sonars:
						self.interrupt(SonarEmergencyClearInterrupt())
		if self.direction == DIRECTION_BACKWARD or self.direction == DIRECTION_ANY:
			if ev.sonar == 'dist_back_left' or ev.sonar == 'dist_back_right':
				if not ev.state_ok:
					if not bypass_sonar():
						self.triggered_sonars += 1
						if not self.emergency:
							self.emergency = True
							self.interrupt(SonarEmergencySetInterrupt())
				else:
					self.triggered_sonars -= 1
					if not self.triggered_sonars:
						self.interrupt(SonarEmergencyClearInterrupt())


class ActionRotate(ActionBase):

	def __init__(self, angle: float, **kwargs):
		super().__init__(**kwargs)
		self.angle = angle

	def run(self):
		req = RequestStmRotate(self.angle, sync=False)
		self.parent_node.publish_request(req)

		while not self.parent_node.check_for_reply(req):
			self.check_for_interruption()

		if self.emergency:
			self.emergency = False
			self.run()


class SonarEmergencySetInterrupt(InterruptBase):
	pass


class SonarEmergencyClearInterrupt(InterruptBase):
	pass
