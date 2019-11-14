#!/usr/bin/python3
import sys
import unittest

import rospy
import rostest

from xml_class_parser import Parsable, Bind, BindList, BindDict, Enum, ParsingException, Slice, Context

from typing import Dict, List

from classes import SomeClass, SomeOtherClass, YetAnotherClass

class TestParsingErrors(unittest.TestCase):
	def setUp(self):
		rospy.init_node('test_errors', anonymous=True)
		
	def test_invalid_name_exception(self):
		global SomeClass
		SomeClass = Parsable(
			name="yes"
		)(SomeClass)

		with self.assertRaises(ParsingException):
			SomeClass.parse_string("<no />")

	
	def test_mandatory(self):
		global SomeClass

		SomeClass = Parsable(name="lowl", attributes={"name": Bind(mandatory=True, to="str_attr")})(SomeClass)
		
		parsed = SomeClass.parse_string("<lowl name='bob' />")

		self.assertIsInstance(parsed, SomeClass)
		self.assertEqual(parsed.str_attr, "bob")

		with self.assertRaises(ParsingException):
			SomeClass.parse_string("<lowl />")


	def test_invalid_attribute(self):
		raise NotImplemented()

if __name__ == '__main__':
	rostest.rosrun('xml_class_parser', 'test_errors', TestParsingErrors, sys.argv)
