from Cerveau.interfaces.interface_description.python_parser import Frame
from Cerveau.interfaces.can_interface.can_handler import CanHandler
from Cerveau.node_manager.node_manager import NodeBase
from Cerveau.node_manager.event_handler import observe_event
from Cerveau.events.can_events import EventCanReceive, EventCanSend

from typing import Dict


class CanNode(NodeBase):

	def __init__(self, dev='can0'):
		super().__init__()
		self.can_handler = CanHandler(dev, callback=self.on_message, autostart=False)

	def start(self):
		super().start()
		self.threads.append(self.can_handler.can_input_thread)
		self.can_handler.start_input_thread()

	def on_message(self, frame_type: Frame, data_fields: Dict):
		ev = EventCanReceive(frame_type, data_fields)
		self.send_event(ev, state=frame_type.cmd_id)

	@observe_event(EventCanSend)
	def send_message(self, ev):
		self.can_handler.send(ev.frame_type, ev.data_fields)
