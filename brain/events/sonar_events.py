from typing import Dict

from ..engine_base.event_base import EventBase, EventRequestBase, EventReplyBase


class EventSonarDist(EventBase):

	def __init__(self, sonar: str, state_ok: bool, src=None):
		super().__init__(src)
		self.sonar = sonar
		self.state_ok = state_ok


class RequestSonarDist(EventRequestBase):

	pass


class ReplySonarDist(EventReplyBase):

	def __init__(self, sonars_data: Dict, sonars_states_ok: Dict, src=None):
		super().__init__(src)
		self.sonars_data = sonars_data
		self.sonars_states_ok = sonars_states_ok
