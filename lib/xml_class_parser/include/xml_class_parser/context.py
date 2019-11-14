from . import ParsingException

from typing import Any

class Context:
	def __init__(self, **params):
		self.parent = None
		self.params = params
	
	def get(self, name: str):
		try:
			return self.params[name]
		except KeyError:
			raise ParsingException("some element require context element '{}' but it wasn't provided".format(name))
		
	def set(self, name: str, value: Any):
		if name in self.params:
			print("warning : overriding value of {}".format(name))
		
		self.params[name] = value