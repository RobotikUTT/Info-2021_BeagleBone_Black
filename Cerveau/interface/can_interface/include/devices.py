from lib.xml_class_parser.include.xml_class_parser import Parsable, Bind, BindDict, Context, Slice, ParsingException
from lib.xml_class_parser.include.xml_class_parser.helper import AttrAttrTuple

from typing import Dict, List

# A device is just a name/value tuple with value being device id
Device = AttrAttrTuple("device", "name", "id", int)

@Parsable(
	name="devices",
	children=[
		BindDict(
			to="by_name",
			key="name",
			type=Device,
			post_cast=Slice("value")
		),
		Bind(to="broadcast", xml_name="broadcast", type=Device),
	]
)
class DeviceList:
	def __init__(self):
		super().__init__()

		# Devices and broadcast
		self.by_id: Dict[int, str] = {}
		self.by_name: Dict[str, int] = {}

		self.broadcast: Device = None

	def __parsed__(self, context: Context):
		if self.broadcast is None:
			raise ParsingException("no broadcast provided in device list")

		# Add broadcast to list
		self.by_name[self.broadcast.name] = self.broadcast.value

		# Save devices by id
		for dev_name in self.by_name:
			self.by_id[self.by_name[dev_name]] = dev_name
