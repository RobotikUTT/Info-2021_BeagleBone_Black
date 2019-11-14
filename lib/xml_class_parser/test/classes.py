from xml_class_parser import Context
from typing import Union

# Some classes for testing

class SomeClass:
	def __init__(self):
		self.str_attr = "default"
		self.int_attr = 36
		self.array_attr = []
		self.dict_attr = {}

	def __parsed__(self, context: Context):
		if "name" in context.params:
			self.str_attr = context.get("name")

	def __before_children__(self, context: Context):
		context.set("bob", 76)

class YetAnotherClass(SomeClass):
	def __init__(self, oui=False, wow=0):
		super().__init__()
		self.oui = oui
		self.wow = wow
		
		self.get_called = False
		self.set_called = False

	def __parsed__(self, context: Context):
		self.oui = True
		
		if "bob" in context.params:
			self.wow = context.get("bob")

	def __before_children__(self, ctx):
		pass

	@property
	def x(self):
		self.get_called = True
		return 5
	
	@x.setter
	def x(self, x):
		self.set_called = True

class SomeOtherClass:
	def __init__(self):
		self.name = ""
		self.value = 0
		self.contained: SomeClass = None
		self.children: List[SomeClass] = []