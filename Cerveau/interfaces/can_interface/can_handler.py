#!/usr/bin/python3

from Cerveau.interface.interface_description.python_parser import Frame, devicesList, messagesList, framesList

from threading import Thread, Lock
from typing import Dict
import logging
import time
# import traceback

# python-can library
import can
can.rc['interface'] = 'socketcan_ctypes'


class CanHandler:

	def __init__(self, dev='can0', callback=None, autostart=True):
		self.lock = Lock()

		# Callback format: def func_name(frame_type: Frame, fields: Dict)
		self.callback = callback

		# Create can bus with given interface
		self.bus = can.interface.Bus(dev)
		self.can_input_thread = Thread(name='can_input', target=self.wait_for_can_message, daemon=True)		# daemon=True => Thread closed on parent exit

		if autostart:
			self.start_input_thread()

	def start_input_thread(self):
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
				except Exception as e:
					# traceback.print_exc()
					logging.warning('Received invalid can frame:')
					logging.warning(message)
					logging.warning('with following exception:')
					logging.warning(e)

		except Exception as e:
			self.bus.shutdown()
			logging.error('Can reception interrupted with following exception:')
			logging.error(e)

	def on_can_message(self, message: can.Message):
		'''
			Parse messages from can and call Callback
		'''
		if len(message.data) == 0:
			logging.debug('Received empty frame, ignoring.')
			return

		# Get frame type
		frame_type = None
		for frame in framesList:
			if message.data[0] == frame.cmd_id:
				frame_type = frame
				break
		else:		# If no-break
			logging.debug(f'Received unhandled frame command id: {message.data[0]}')
			return

		# Get destination Device
		device = None
		for dev in devicesList:
			if message.arbitration_id == dev.arbitration_id:
				device = dev
				break
		else:		# If no-break
			logging.debug(f'Received frame destinated to an unknown device id: {message.arbitration_id}')
			return

		logging.info(f'Received can frame "{frame_type.name}" to device: {device.name}')

		# Create message
		try:
			raw_data = frame.data[1:]
			data_fields = {}
			for field in frame_type.fields:
				if field._type == 'byte':
					data_fields[field.name] = raw_data[0]
					raw_data = raw_data[1:]
				else:
					# Big-Endian, so MSByte first
					data_fields[field.name] = (raw_data[0] << 8) | raw_data[1]
					raw_data = raw_data[2:]
		except IndexError:
			logging.debug(f'Received incomplete frame of type {frame_type.name}, ignoring')
			return

		if self.callback is not None:
			self.callback(frame_type, data_fields)

	def send(self, frame_type: Frame, data_fields: Dict):
		'''
			Handle message from ROS and build a frame from it
		'''
		with self.lock:
			logging.info(f'New frame to send of type {frame_type.name}')

			# Prepare data array
			try:
				data = []
				for field in frame_type.fields:
					if field._type != 'byte':
						# Big-Endian, so MSByte first
						data.append((data_fields[field.name] >> 8) & 0xff)
					data.append(data_fields[field.name] & 0xff)
			except KeyError:
				logging.error('Message NOT sent: Malformed data_fields')

			# Setup output frame
			frame = can.Message(
				data=data,
				arbitration_id=self.devices.by_name[frame_type.dest],
				# is_remote_frame=False,
				# is_error_frame=False,
				is_extended_id=False,
				timestamp=time.time()
			)

			# Send frame
			try:
				self.bus.send(frame)
				logging.info(f'Message sent on {self.channel_info}:')
				logging.info(frame)
			except can.CanError as e:
				logging.error(f'Message NOT sent: CanError: {e}')
