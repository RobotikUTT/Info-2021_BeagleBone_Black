from Cerveau.interfaces.interface_description.python_parser import Frame
from Cerveau.node_manager.event import EventBase

from typing import Dict


class EventCanReceive(EventBase):

	def __init__(self, frame_type: Frame, data_fields: Dict, src=None):
		super().__init__(src)
		self.frame_type = frame_type
		self.data_fields = data_fields


class EventCanSend(EventBase):

	def __init__(self, frame_type: Frame, data_fields: Dict, src=None):
		super().__init__(src)
		self.frame_type = frame_type
		self.data_fields = data_fields
