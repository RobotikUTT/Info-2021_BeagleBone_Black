from lib.xml_class_parser.include.xml_class_parser import Parsable, ParsingException, Context
from lib.xml_class_parser.include.xml_class_parser.bind import Bind, BindDict, BindList

from lib.args_lib.include.argumentable import Argumentable

from .param import Param, MissingParameterException

from typing import Dict

import can
import logging

@Parsable(
	name="frame",
	attributes={
		"name": Bind(mandatory=True),
		"id": Bind(mandatory=True, type=int),
		"source": str,
		"dest": str
	},
	children=[
		BindList(to="params", type=Param)
	],
	ignored_children = ["description"]
)
class Frame:
	"""
		A frame represent a possible exchange value in CAN bus
	"""
	def __init__(self):
		self.name = ""
		self.id = -1

		self.source = None
		self.dest = None

		self.params = []
	
	def __parsed__(self, context: Context):
		devices = context.get("devices")
		
		self.source = self.source if self.source is not None else devices.broadcast.name
		self.dest = self.dest if self.dest is not None else devices.broadcast.name

		# Check that devices exists
		if self.source not in devices.by_name:
			raise ParsingException("{} is not a valid device".format(self.source))
		
		elif self.dest not in devices.by_name:
			raise ParsingException("{} is not a valid device".format(self.dest))
		
		# Compute params byte_start property
		current_offset: int = 1
		for param in self.params:
			param.byte_start = current_offset
			current_offset += param.size
	
	def extract_frame_data(self, frame: 'can.Message') -> Argumentable:
		"""
			Extract data from frame data, and return an argumentable
			having all the properties
		"""
		data = Argumentable()

		for param in self.params:
			param.can_to(frame, data)
		
		return data
	
	def size(self):
		"""Returns frame data size"""
		if len(self.params) == 0:
			return 1
		
		return self.params[-1].byte_start + self.params[-1].size

	def get_frame_data(self, values: Argumentable) -> bytearray:
		"""
			Insert parameters into frame, returns a byte array if
			all parameters are here, otherwise returns None
		"""
		data_array = [0] * 8
		data_array[0] = self.id
		
		try:
			for param in self.params:
				param.to_can(data_array, values)
		except MissingParameterException as e:
			logging.error("unable to find parameter {} for frame {}, not sending"
				.format(e, self.name))

			return bytearray(0)
		
		return bytearray(data_array)

@Parsable(
	name="frames",
	children=[
		BindDict( to="by_name", key="name", type=Frame )
	]
)
class FrameList:
	def __init__(self):
		# Devices and broadcast
		self.by_id: Dict[int, str] = {}
		self.by_name: Dict[str, int] = {}

	def __parsed__(self):
		# Save devices by id
		for dev_name in self.by_name:
			self.by_id[self.by_name[dev_name]] = str(self.by_name[dev_name])
