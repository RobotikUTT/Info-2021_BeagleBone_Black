import logging
from queue import Queue

from ..interfaces.interface_description.python_parser import framesByName
from ..engine_base.node_base import NodeBase, subscribe_to_event
from ..events.can_events import EventCanReceive, EventCanSend
from ..events.actuator_events import (EventActuatorServo, EventActuatorStepperParams,
	RequestActuatorStepperSteps, ReplyActuatorStepperSteps)

actuator_logger = logging.getLogger('actuator')


class ActuatorNode(NodeBase):

	pending_requests = None

	def __init__(self, **kwargs):
		super().__init__(**kwargs)
		self.pending_requests = Queue()

	@subscribe_to_event(EventCanReceive, states=[framesByName['end_step'].cmd_id])
	def steps_done(self, ev):
		nb_steps_done = ev.data_fields['nb_steps_done']
		if not self.pending_requests.empty():
			req = self.pending_requests.get()
			rep = ReplyActuatorStepperSteps(nb_steps_done=nb_steps_done)
			self.reply_to_request(req, rep)
			actuator_logger.debug(f'Order compelted. {nb_steps_done} out of {req.args_dict["nb_steps"]}.')
		else:
			actuator_logger.debug(f'Order compelted. {nb_steps_done} out of Unknown.')

	@subscribe_to_event(RequestActuatorStepperSteps)
	def stepper_make_steps(self, req):
		self.pending_requests.put(req)
		frame = framesByName['stepper_steps']
		ev = EventCanSend(frame, req.args_dict)
		self.publish_event(ev)

	@subscribe_to_event(EventActuatorStepperParams)
	def stepper_params(self, req):
		frame = framesByName['stepper_param']
		ev = EventCanSend(frame, req.args_dict)
		self.publish_event(ev)

	@subscribe_to_event(EventActuatorServo)
	def move_servo(self, req):
		frame = framesByName['servo_angle']
		ev = EventCanSend(frame, req.args_dict)
		self.publish_event(ev)
