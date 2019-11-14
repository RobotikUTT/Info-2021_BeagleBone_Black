#!/usr/bin/python3
import sys
import unittest

import rospy
import rostest

from xml_class_parser import Parsable, Bind, BindList, BindDict, Enum, ParsingException, Slice, Context

from typing import Dict, List

from classes import SomeClass, SomeOtherClass, YetAnotherClass

class TestBindings(unittest.TestCase):
	def setUp(self):
		rospy.init_node('test_bindings', anonymous=True)

	def test_simple_binding(self):
		global SomeClass
		SomeClass = Parsable(name="i", attributes={"int_attr": int})(SomeClass)

		self.assertEqual(SomeClass.parse_string("<i int_attr='4' />").int_attr, 4)

	def test_context_parent(self):
		raise NotImplementedError()

	def test_property_call(self):
		"""Test that a class property is not override by parsed values"""
		global YetAnotherClass
		YetAnotherClass = Parsable(name=Bind(to="x", type=Enum(binding = {"zero": 0})))(YetAnotherClass)

		parsed = YetAnotherClass.parse_string('<zero />')

		self.assertIsInstance(parsed, YetAnotherClass)
		self.assertEqual(parsed.set_called, True, "setter called on parsing")
		self.assertEqual(parsed.x, 5, "property response unchanged")
		self.assertEqual(parsed.get_called, True, "getter called")


	def test_context_passing(self):
		global SomeClass
		global YetAnotherClass

		YetAnotherClass = Parsable(name="array_attr")(YetAnotherClass)
		SomeClass = Parsable(name="o", children = [ BindList(type=YetAnotherClass, to="array_attr") ])(SomeClass)

		parsed = SomeClass.parse_string("<o><array_attr /></o>", context=Context(name="hello"))

		self.assertEqual(parsed.str_attr, "hello", "original context is passed")
		self.assertEqual(len(parsed.array_attr), 1)
		self.assertEqual(parsed.array_attr[0].wow, 76, "context is passed with additional args")

	def test_content_parsing(self):
		global SomeClass
		SomeClass = Parsable(name="two", content=Bind(type=int, to="int_attr"))(SomeClass)

		parsed = SomeClass.parse_string("<two>84</two>")

		self.assertIsInstance(parsed, SomeClass)
		self.assertEqual(parsed.int_attr, 84)

	def test_list_parsing(self):
		global SomeClass
		global SomeOtherClass

		SomeClass = Parsable(name="in", attributes={ "v": Bind(to="int_value", type=int) })(SomeClass)
		SomeOtherClass = Parsable(name="box", children = [ BindList(to="children", type=SomeClass) ])(SomeOtherClass)
		
		parsed: SomeOtherClass = SomeOtherClass.parse_string("""
			<box>
				<in v="1" />
				<in v="2" />
				<in v="3" />
			</box>
		""")

		self.assertIsInstance(parsed, SomeOtherClass, "class parsed")
		self.assertEqual(len(parsed.children), 3, "right number of children")

		for i in range(3):
			child = parsed.children[i]

			self.assertIsInstance(child, SomeClass, "children has right class")
			self.assertEqual(child.int_value, i + 1, "values are parsed in order")

	def test_dict_parsing(self):
		global SomeClass
		global SomeOtherClass

		SomeOtherClass = Parsable(name=Bind(to="name"), content=Bind(to="value", type=int))(SomeOtherClass)
		SomeClass = Parsable(name="box", children=[ BindDict(to="dict_attr", key="name", type=SomeOtherClass) ])(SomeClass)
		
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
				self.assertEqual(parsed.dict_attr[key].value, 5)
			elif key == "non":
				self.assertEqual(parsed.dict_attr[key].value, 21)
			elif key == "maybe":
				self.assertEqual(parsed.dict_attr[key].value, 37)




if __name__ == '__main__':
	rostest.rosrun('xml_class_parser', 'test_bindings', TestBindings, sys.argv)
