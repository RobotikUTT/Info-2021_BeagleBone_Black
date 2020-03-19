#!/usr/bin/python3

from Cerveau.interface.interface_description.python_parser import devicesList, messagesList, framesList
import can


def retrieve_by_name(_list, _name):
	for _item in _list:
		# La puissance du Python BOOM! Est-ce que _item a bien un membre 'name' ? On sait pas et on s'en fout !
		if _item.name == _name:
			return _item


def main():
	interface = input('interface [can0] : ')
	if interface == '':
		interface = 'can0'

	can.rc['interface'] = 'socketcan_ctypes'
	bus = can.Bus(interface)

	while True:
		frame_name = input('CAN frame to send : ')
		while frame_name not in (frame.name for frame in framesList):
			print('Available frames :\n\t{}'.format('\n\t'.join((frame.name for frame in framesList))))
			frame_name = input('CAN frame to send : ')

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
