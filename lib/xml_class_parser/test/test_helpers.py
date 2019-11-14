#!/usr/bin/python3
import sys
import unittest

import rospy
import rostest

from xml_class_parser import Parsable, Bind, BindList, BindDict, Enum, ParsingException, Slice, Context

from typing import Dict, List

from classes import SomeClass, SomeOtherClass, YetAnotherClass

class TestHelpers(unittest.TestCase):
	def setUp(self):
		rospy.init_node('test_helpers', anonymous=True)
		
	def test_enum_binding_parsing(self):
		global SomeClass
		enum = Enum(binding={"one": 1, "two": 2, "three": 3, "four": 4, "five": 5})
		SomeClass = Parsable(name="two", content=Bind(type=enum, to="int_attr"))(SomeClass)

		# Parsing success
		parsed = SomeClass.parse_string("<two>one</two>")

		self.assertIsInstance(parsed, SomeClass)
		self.assertEqual(parsed.int_attr, 1)

		# Parsing error
		with self.assertRaises(ParsingException):
			SomeClass.parse_string("<two>zero</two>")

	def test_enum_values_parsing(self):
		global SomeClass
		enum = Enum(values=[5, 6, 7, 8, 9], cast=int)
		SomeClass = Parsable(name="three", attributes={"value": Bind(type=enum, to="int_attr") })(SomeClass)

		# Parsing success
		parsed = SomeClass.parse_string("<three value='7' />")

		self.assertIsInstance(parsed, SomeClass)
		self.assertEqual(parsed.int_attr, 7)

		# Parsing error
		with self.assertRaises(ParsingException):
			SomeClass.parse_string("<three value='3' />")


	def test_slicing(self):
		global SomeClass
		global SomeOtherClass

		SomeOtherClass = Parsable(name=Bind(to="name"), content=Bind(to="value", type=int))(SomeOtherClass)
		SomeClass = Parsable(name="box", children=[ BindDict(to="dict_attr", key="name", type=SomeOtherClass, post_cast=Slice("value")) ])(SomeClass)
		
		parsed: SomeClass = SomeClass.parse_string("""
			<box>
				<oui>5</oui>
				<non>21</non>
				<maybe>37</maybe>
			</box>
		""")

		self.assertIsInstance(parsed, SomeClass, "right class parsed")
		self.assertEqual(len(parsed.dict_attr.keys()), 3, "right number of keys")

		for key in parsed.dict_attr:
			if key == "oui":
				self.assertEqual(parsed.dict_attr[key], 5)
			elif key == "non":
				self.assertEqual(parsed.dict_attr[key], 21)
			elif key == "maybe":
				self.assertEqual(parsed.dict_attr[key], 37)


if __name__ == '__main__':
	rostest.rosrun('xml_class_parser', 'test_helpers', TestHelpers, sys.argv)
