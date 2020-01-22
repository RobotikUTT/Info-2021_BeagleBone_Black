# Bridge interface_msgs

from lib.xml_class_parser.include.xml_class_parser import Context

from lib.args_lib.include.args_lib.argumentable import Argumentable

from .frames import Frame, FrameList
from .devices import DeviceList

from typing import Dict

import time
import threading
import traceback
import logging

# python-can library
import can
can.rc['interface'] = 'socketcan_ctypes'

class CanHandler:

	def __init__(self, dev="vcan0", callback=None):
		self.callback = callback

		# Parse devices
		self.devices: DeviceList = DeviceList.parse_file(
			"can/devices.xml",
			package="interface_description"
		)

		# Then frames
		self.frames: Dict[str, Frame] = FrameList.parse_file(
			"can/frames.xml",
			package="interface_description",
			context=Context(devices=self.devices)
		)

		# Create can bus with given interface
		self.bus = can.interface.Bus(dev)
		self.can_input_thread = threading.Thread(name="can_input", target=self.wait_for_can_message)

		# Start thread
		self.can_input_thread.start()

	def wait_for_can_message(self):
		'''
			Loop in CAN bus to read data
		'''
		try:
			for message in self.bus:
				try:
					self.on_can_message(message)
				except:
					traceback.print_exc()
					logging.error("received invalid can frame")
					logging.error(message)
		except:
			self.bus.shutdown()
			logging.warning("Can reception interrupted")
			
	
	def on_can_message(self, frame: can.Message):
		"""
			Callback from messages from can
		"""
		if len(frame.data) == 0:
			logging.error("received empty frame, ignoring")
			return

		elif frame.data[0] not in self.frames:
			logging.error("received unhandled frame of id {}".format(frame.data[0]))
			return
		
		# Get frame type
		frame_type = self.frames[frame.data[0]].by_id
		
		logging.debug("received frame from can of type {}".format(frame_type.name))

		# Create message
		try:
			params = frame_type.extract_frame_data(frame).to_list()
		except IndexError:
			logging.error("received incomplete frame of type {}, ignoring".format(frame_type.name))
			return

		if self.callback is not None:
			self.callback(frame_type.name, params)

	def send(self, type, params: Argumentable):
		"""
			Handle message and build a frame
		"""
		
		logging.debug("received frame to send of type {}".format(self.callback.type))

		# Get message params as argumentable
		values = params
		frame_type = self.frames[type].by_name

		# Prepare data array and set frame type
		data = frame_type.get_frame_data(values)
		
		if data is not None:
			# Setup output frame
			frame: can.Message = can.Message(
				timestamp=time.time(),
				is_remote_frame=False,
				is_error_frame=False,
				is_extended_id=False,
				dlc=frame_type.size(),
				arbitration_id=self.devices.by_name[frame_type.dest],
				data=data
			)

			# Send frame
			self.bus.send(frame)
