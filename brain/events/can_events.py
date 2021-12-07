from ..interfaces.interface_description.python_parser import Frame
from ..engine_base.event_base import EventBase
# Relative imports for intra-package imports are highly discouraged TODO

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
