#!/usr/bin/python3

from dataclasses import dataclass, field
from typing import List, Dict

import json
# import can


@dataclass
class Device:
	name: str
	_id: int


@dataclass
class Bind:
	frame: str
	message: str


@dataclass
class Message:
	direction: str
	binds: List[Bind]


@dataclass
class Field:
	name: str
	_type: str
	enum: Dict[str, int] = field(default_factory=dict)
	r_enum: Dict[int, str] = field(default_factory=dict)


@dataclass
class Frame:
	name: str
	description: str
	cmd_id: int
	source: str
	destination: str
	fields: list


def main():
	with open('../data/devices.json', 'r') as f:
		devices = json.load(f)["devices"]
	# A list of Device objects (please see the Device @dataclass for the structure)
	devicesList = [Device(**device) for device in devices]

	with open('../data/messages.json', 'r') as f:
		messages = json.load(f)["messages"]
	# A list of Message objects (please see the Message @dataclass for the structure)
	messagesList = [Message(message["direction"], [Bind(**bind) for bind in message["binds"]]) for message in messages]

	with open('../data/framesshort.json', 'r') as f:
		frames = json.load(f)["frames"]
	# A list of Frame objects (please see the Frame @dataclass for the structure)
	framesList = []
	for frame in frames:
		frame['fields'] = [Field(**field, r_enum=dict(((v, k) for k, v in field.get('enum', {}).items()))) for field in frame['fields']]
		framesList.append(Frame(**frame))


	# interface = input("interface [can0] : ")
	# if interface == "":
	# 	interface = "can0"

	# can.rc['interface'] = 'socketcan_ctypes'
	# bus = can.Bus(interface)



"""
	devices = DeviceList.parse_file("can/devices.xml", "interface_description")
	frames = FrameList.parse_file("can/frames.xml", "interface_description", context={"devices": devices})

	while True:
		frame_name = input("can frame to send : ")

		while frame_name not in frames.by_name:
			print("available frames :\n\t{}".format("\n\t".join(frames.by_name.keys())))
			frame_name = input("can frame to send : ")

		frame = frames.by_name[frame_name]

		values = Argumentable()
		for param in frame.params:
			val = None

			while val is None:
				try:
					val = int(input("value for {} : ".format(param.name)))
				except ValueError:
					print("value must be int")
			values.set(param.name, val)

		print("preparing frame")
		message = can.Message(
			data=frame.get_frame_data(values),
			arbitration_id=devices.by_name[frame.dest],
			is_extended_id=False,
			dlc=frame.size()
		)

		print("sending frame")
		bus.send(message)
"""

if __name__ == '__main__':
	main()
