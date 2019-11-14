#!/usr/bin/python3
import sys
import unittest

import rospy
import rostest

from xml_class_parser import Parsable, Bind, BindList, BindDict, Enum, ParsingException, Slice, Context

from typing import Dict, List

from classes import SomeClass, SomeOtherClass, YetAnotherClass

class TestParsingBehavior(unittest.TestCase):
	def setUp(self):
		rospy.init_node('test_behavior', anonymous=True)
		

	def test_ignoring_children(self):
		global YetAnotherClass
		YetAnotherClass = Parsable(name="o", ignored_children=["a"])(YetAnotherClass)

		YetAnotherClass.parse_string("<o><a></a><a></a></o>")

	def test_takes_external_object(self):
		global YetAnotherClass
		YetAnotherClass = Parsable(name="minus_one")(YetAnotherClass)

		# Try to set wow to 4 and oui (first arg) to True
		parsed = YetAnotherClass.parse_string("<minus_one />", obj=YetAnotherClass(True, wow=4))

		self.assertEqual(parsed.oui, True)
		self.assertEqual(parsed.wow, 4)

	def test_before_children_callback(self):
		global YetAnotherClass
		YetAnotherClass = Parsable(name="minus_two")(YetAnotherClass)

		# Try to set wow to 4 and oui (first arg) to True
		parsed = YetAnotherClass.parse_string("<minus_two />", YetAnotherClass(oui=False))

		self.assertEqual(parsed.oui, True, "call __before_children__ after parsing properties and before children")


	def test_recursive_parsing(self):
		global SomeClass
		SomeClass = Parsable(name="four", children=[ BindList(type=Parsable.SELF, to="array_attr") ])(SomeClass)

		# Parsing success
		parsed = SomeClass.parse_string("<four><four></four></four>")

		self.assertIsInstance(parsed, SomeClass)
		self.assertEqual(len(parsed.array_attr), 1)
		self.assertIsInstance(parsed.array_attr[0], SomeClass)

		# Parsing error
		with self.assertRaises(ParsingException):
			SomeClass.parse_string("<three value='3' />")
	


	def test_loop_parsing(self):
		global SomeClass
		global SomeOtherClass

		SomeOtherClass = Parsable(name="babana", children=[ BindList(to="children", type=SomeClass) ])(SomeOtherClass)
		SomeClass = Parsable(name="appel", children=[ BindList(to="array_attr", type=SomeOtherClass) ])(SomeClass)
		
		parsed: SomeClass = SomeClass.parse_string("""
			<appel>
				<babana>
					<appel>
						<babana />
					</appel>
				</babana>
				<babana/>
			</appel>
		""")

		self.assertIsInstance(parsed, SomeClass)
		self.assertEqual(len(parsed.array_attr), 2)

		# First depth
		self.assertEqual(len(parsed.array_attr[0].children), 1)
		self.assertEqual(len(parsed.array_attr[1].children), 0)

		# Second depth
		self.assertEqual(len(parsed.array_attr[0].children[0].array_attr), 1)

		self.assertEqual(len(parsed.array_attr[0].children[0].array_attr[0].children), 0)

	def test_alias_parsing(self):
		raise NotImplementedError()

	def test_call_parsed_callback(self):
		global YetAnotherClass

		YetAnotherClass = Parsable(name="a")(YetAnotherClass)

		self.assertTrue(YetAnotherClass.parse_string("<a />").oui)

if __name__ == '__main__':
	rostest.rosrun('xml_class_parser', 'test_behavior', TestParsingBehavior, sys.argv)
