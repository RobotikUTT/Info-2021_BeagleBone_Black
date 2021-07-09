#!/usr/bin/python3

from brain.interfaces.interface_description.python_parser import Frame, devicesById, devicesByName, framesByCmdId

from threading import Thread, Lock
from typing import Dict
import logging
import struct
import time

# python-can library
import can
can.rc['interface'] = 'socketcan_ctypes'

can_logger = logging.getLogger('can_handler')

msg_count = 0


class CanHandler:

	def __init__(self, dev='can1', callback=None, autostart=True):
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
					can_logger.warning('Received invalid can frame:')
					can_logger.warning(message)
					can_logger.warning('with following exception:')
					can_logger.warning(e)

		except Exception as e:
			self.bus.shutdown()
			can_logger.error('Can reception interrupted with following exception:')
			can_logger.error(e)

	def on_can_message(self, message: can.Message):
		'''
			Parse messages from can and call Callback
		'''
		global msg_count

		# Hopefully this avoid concurrency access races (spoiler: no xO)
		msg_num = (msg_count := msg_count + 1)

		# Get destination Device
		dst_device = devicesById.get(message.arbitration_id)
		if dst_device is None:
			can_logger.warning(f'{msg_num}: Received frame destinated to an unknown device id: {message.arbitration_id}')
			can_logger.debug(f'{msg_num}: ' + self.frame_to_str(message))
			return

		if len(message.data) == 0:
			can_logger.warning(f'{msg_num}: Received empty frame (dst={dst_device.name}), ignoring.')
			can_logger.debug(f'{msg_num}: ' + self.frame_to_str(message))
			return

		# Get frame type
		frame_type = framesByCmdId.get(message.data[0])
		if frame_type is None:
			can_logger.warning(f'{msg_num}: Received unhandled frame command id: {message.data[0]}')
			can_logger.debug(f'{msg_num}: ' + self.frame_to_str(message))
			return

		if dst_device.name == 'all' or dst_device.name == 'bbb':
			can_logger.debug(f'{msg_num}: Received can frame "{frame_type.name}" for device: {dst_device.name}')
		else:
			can_logger.debug(f'{msg_num}: Received can frame "{frame_type.name}" for device: {dst_device.name}')
		can_logger.debug(f'{msg_num}: ' + self.frame_to_str(message))

		# Create message
		try:
			fmt = '>' + ''.join(('B' if field._type == 'byte' else 'h') for field in frame_type.fields)
			data_fields = {
				field.name: data for data, field in zip(struct.unpack_from(fmt, bytes(message.data), offset=1), frame_type.fields)
			}
		except struct.error:
			can_logger.warning(f'{msg_num}: Received incomplete frame of type "{frame_type.name}" (dst={dst_device.name}), ignoring')
			return

		# try:
		# 	raw_data = message.data[1:]
		# 	data_fields = {}
		# 	for field in frame_type.fields:
		# 		if field._type == 'byte':
		# 			data_fields[field.name] = raw_data[0]
		# 			raw_data = raw_data[1:]
		# 		else:
		# 			# Big-Endian, so MSByte first
		# 			data_fields[field.name] = (raw_data[0] << 8) | raw_data[1]
		# 			raw_data = raw_data[2:]
		# except IndexError:
		# 	can_logger.warning(f'{msg_num}: Received incomplete frame of type "{frame_type.name}" (dst={dst_device.name}), ignoring')
		# 	return

		if self.callback is not None:
			self.callback(frame_type, data_fields)

	def send(self, frame_type: Frame, data_fields: Dict):
		'''
			Handle message from ROS and build a frame from it
		'''
		with self.lock:
			global msg_count

			# Hopefully this avoid concurrency access races (spoiler: no xO)
			msg_num = (msg_count := msg_count + 1)

			can_logger.info(f'{msg_num}: New frame to send of type {frame_type.name}')

			# Prepare data array
			try:
				fmt = '>' + ''.join(('B' if field._type == 'byte' else 'h') for field in frame_type.fields)
				data = [frame_type.cmd_id] + list(struct.pack(fmt, *(data_fields[field.name] for field in frame_type.fields)))
				if frame_type.cmd_id == 46:
					can_logger.info(fmt)
					can_logger.info(data)
			except (KeyError, struct.error):
				can_logger.error(f'{msg_num}: Message NOT sent: {frame_type.name}: Malformed data_fields')
				return

			# try:
			# 	data = [frame_type.cmd_id]
			# 	for field in frame_type.fields:
			# 		if field._type != 'byte':
			# 			# Big-Endian, so MSByte first
			# 			data.append((data_fields[field.name] >> 8) & 0xff)
			# 		data.append(data_fields[field.name] & 0xff)
			# except KeyError:
			# 	can_logger.error(f'{msg_num}: Message NOT sent: {frame_type.name}: Malformed data_fields')
			# 	return

			dst_device = devicesByName.get(frame_type.destination)
			if dst_device is None:
				can_logger.error(f'{msg_num}: Message NOT sent: {frame_type.name}: Unknown destination')
				return

			# Setup output frame
			frame = can.Message(
				data=data,
				arbitration_id=dst_device.arbitration_id,
				# is_remote_frame=False,
				# is_error_frame=False,
				is_extended_id=False,
				timestamp=time.time()
			)

			# Send frame
			try:
				self.bus.send(frame)
				# can_logger.debug(f'{msg_num}: Message sent on {self.bus.channel_info}:')
				# can_logger.debug(f'{msg_num}: ' + self.frame_to_str(message))
				if frame_type.cmd_id == 46:
					can_logger.info(f'{msg_num}: ' + self.frame_to_str(frame))
			except can.CanError as e:
				can_logger.error(f'{msg_num}: Message NOT sent: {frame_type.name}: CanError: {e}')
				return

	def frame_to_str(self, message: can.Message):
		return f'{message.channel}    {message.arbitration_id:03}   [{message.dlc}]   ' + ' '.join(f'{b:02x}' for b in message.data)
