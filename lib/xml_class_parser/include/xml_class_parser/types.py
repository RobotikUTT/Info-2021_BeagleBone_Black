from . import ParsingException

from typing import List, Any, Dict, Union, Optional

def Enum(values: List[Any] = None, cast=str, binding: Union[Dict, None] = None):
	'''
		Return a type cast that check that a value is in the range.
		
		It first cast the value to provided cast element (default: str)
		Then apply binding if provided, raising exception is no bound value found.
		
		Then check the value is in values array provided (if provided), raising
		exception if the bound value is not in the array.

		In the end, if the value have a binding and that binding (or initial value)
		is in authorized values, it returns it.
	'''

	if not isinstance(binding, Dict) and binding is not None:
		binding = vars(binding)

	def cast_value(value: str):
		value = cast(value)

		if binding is not None:
			

			if value in binding:
				value = binding[value]
			else:
				raise ParsingException(
					"unexpected value {}, expected value in [{}]"
					.format(value, ", ".join(map(str, binding.keys()))))
		
		if values is not None:
			if not (value in values):
				raise ParsingException(
					"unexpected bound value {}, expected value in [{}]"
					.format(value, ",".join(map(str, values))))
		
		return value
	
	return cast_value

def Slice(key: str):
	def slicer(value: object):
		return getattr(value, key)
	
	return slicer

def BlackList(*values, cast: type = str):
	def filter(value):
		if value in values:
			raise ParsingException("forbidden value {}".format(value))

		return cast(value)
	
	return filter

def Bool(value: str) -> bool:
	if value.lower() == "true":
		return True
	elif value.lower() == "false":
		return False
	
	raise ParsingException(
		"{} is not a valid boolean value (true|false)".format(value))