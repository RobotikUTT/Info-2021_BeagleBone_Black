#!/usr/bin/python3

from Cerveau.interface.interface_description.python_parser import devicesList, messagesList, framesList
#      VVV -> python-can library
import can
can.rc['interface'] = 'socketcan_ctypes'


def retrieve_by_name(_list, _name):
	for _item in _list:
		# La puissance du Python BOOM! Est-ce que _item a bien un attribut 'name' ? On sait pas et on s'en fout !
		if _item.name == _name:
			return _item


def main():
	interface = input('interface [default=can0] : ')
	if interface == '':
		interface = 'can0'

	can.rc['interface'] = 'socketcan_ctypes'
	try:
		bus = can.Bus(interface)
	except OSError as e:
		print('Interface "{}": OSError:'.format(interface), e)
		exit(1)

	while True:
		# C do-while like
		while True:
			try:
				frame_name = input('CAN frame to send : ')
			except KeyboardInterrupt:
				print()		# Clean LineFeed
				exit(0)
			if frame_name in (frame.name for frame in framesList):
				break
			print('Available frames :\n\t{}'.format('\n\t'.join((frame.name for frame in framesList))))

		frame = retrieve_by_name(framesList, frame_name)
		dest_device = retrieve_by_name(devicesList, frame.destination)

		values = bytearray()
		for field in frame.fields:
			val = None

			while val is None:
				try:
					val = int(input('Value for {} : '.format(field.name)))
				except ValueError:
					print('Value must be an int')
			if field._type != 'byte':
				# Big-Endian, so MSByte first
				values.append((val >> 8) & 0xff)
			values.append(val & 0xff)

		print('Preparing frame')
		msg = can.Message(
			data=values,
			arbitration_id=dest_device.arbitration_id,
			is_extended_id=False
		)

		print('Sending frame')
		try:
			bus.send(msg)
			print('Message sent on {}:'.format(bus.channel_info))
			print(msg)
		except can.CanError:
			print('Message NOT sent')


if __name__ == '__main__':
	main()
