#!/usr/bin/python3

from brain.interfaces.can_interface.can_handler import CanHandler
from brain.interfaces.interface_description.python_parser import Frame, framesList


def handle_frame(frame: Frame, fields: dict):
	print('New Frame received:')
	print('\t.name:', frame.name)
	print('\t.description:', frame.description)
	print('\t.cmd_id:', frame.cmd_id)
	print('\t.destination:', frame.destination)
	print('\t.fields:')
	for field in frame.fields:
		print('\t\t.name:', field.name)
		print('\t\t._type:', field._type)
		print('\t\t.r_enum:')
		for k, v in field.r_enum:
			print(f'\t\t\t{k}: {v}')


canhandler = CanHandler(dev='vcan0', callback=handle_frame)

while True:
	cmd = input()
	print(cmd.encode())
	if cmd.lower in ('q', 'quit', 'exit'):
		break

# TODO: add device destination abr ID
