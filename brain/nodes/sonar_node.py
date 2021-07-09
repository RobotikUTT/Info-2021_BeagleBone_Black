import logging

from ..interfaces.interface_description.python_parser import framesByName
from ..engine_base.node_base import NodeBase, subscribe_to_event
from ..events.can_events import EventCanReceive
from ..events.sonar_events import EventSonarDist, RequestSonarDist, ReplySonarDist

sonar_logger = logging.getLogger('sonar')


SONAR_DIST_THRESHOLD_LOW = 20
SONAR_DIST_THRESHOLD_HIGH = 25


class SonarNode(NodeBase):
	sonars_data = {
		'dist_front_left': 100,
		'dist_front_right': 100,
		'dist_back_left': 100,
		'dist_back_right': 100,
		'dist_side_left': 100,
		'dist_side_right': 100
	}

	sonars_states_ok = {
		'dist_front_left': True,
		'dist_front_right': True,
		'dist_back_left': True,
		'dist_back_right': True,
		'dist_side_left': True,
		'dist_side_right': True
	}

	def __init__(self, **kwargs):
		super().__init__(**kwargs)
		pass

	@subscribe_to_event(EventCanReceive, states=[framesByName['sonar_distance'].cmd_id])
	def new_sonars_values(self, ev):
		self.sonars_data = ev.data_fields
		for sonar, dist in self.sonars_data.items():
			# Hysteresis
			if self.sonars_states_ok[sonar]:
				if dist <= SONAR_DIST_THRESHOLD_LOW:
					self.sonars_states_ok[sonar] = False
					self.publish_event(EventSonarDist(sonar, False))
					sonar_logger.info(f'{sonar} - ALERT SET.')
			else:
				if dist >= SONAR_DIST_THRESHOLD_HIGH:
					self.sonars_states_ok[sonar] = True
					self.publish_event(EventSonarDist(sonar, True))
					sonar_logger.info(f'{sonar} - ALERT CLEAR.')

	@subscribe_to_event(RequestSonarDist)
	def send_sonars_data(self, req):
		rep = ReplySonarDist(self.sonars_data, {k: v for k, v in self.sonars_states_ok.items()})
		self.reply_to_request(req, rep)
