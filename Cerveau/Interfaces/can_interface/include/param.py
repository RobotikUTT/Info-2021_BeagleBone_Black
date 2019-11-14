from lib.xml_class_parser.include.xml_class_parser import Parsable, Enum, Slice
from lib.xml_class_parser.include.xml_class_parser.bind import Bind, BindDict
from lib.xml_class_parser.include.xml_class_parser.helper import TagAttrTuple

from lib.args_lib.include.args_lib.argumentable import Argumentable

from typing import List

from can import Message

import logging


class MissingParameterException(Exception):
	pass

@Parsable(
	name = Bind(to="size", type=Enum(binding={ "word": 2, "byte": 1 })),
	attributes = {
		"name": Bind(mandatory=True)
	},
	children = [
		BindDict(to="values", key="name", type=TagAttrTuple("v", int), post_cast=Slice("value"))
	]
)
class Param:
	"""
		A parameter in a frame
	"""
	def __init__(self):
		self.name: str = ""
		self.size: int = -1
		self.byte_start: int = 0
		self.values = {}

	def __str__(self):
		return "{}[{}-{}]".format(self.name, self.byte_start, self.byte_start + self.size - 1)
	
	def ros_to_can(self, data_array: List[int], values: Argumentable):
		"""
			Apply current parameter into data array from given values
		"""

		if not values.has(self.name):
			raise MissingParameterException(self.name)

		value = int(values.get(self.name, int))

		if value < 0 or value > 255 ** self.size:
			logging.error("unable to encode {} value {}".format(self.name, value))
			return

		if self.size == 1:
			data_array[self.byte_start] = value
		elif self.size == 2:
			data_array[self.byte_start] = value >> 8
			data_array[self.byte_start + 1] = value & 0x00FF
		else:
			raise Exception("size not handled yet, go back to coding")
	
	def can_to_ros(self, frame: Message, values: Argumentable):
		"""
			Retrieve param data from frame data with binary operations
			and put in in the given argumentable
		"""
		value = frame.data[self.byte_start + self.size - 1]

		for index in range(self.size - 1):
			value = value | \
				frame.data[self.byte_start + index] << (self.size - index - 1) * 8

		values.set(self.name, int(value))