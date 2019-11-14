from .parsing_exception import ParsingException
from .context import Context

from typing import Union, Any, Type, Optional
from xml.etree import ElementTree

class Bind:
	'''
		Define a binding between XML element and properties
	'''
	def __init__(self, to: Optional[str] = None, type: Type = str, mandatory=False, xml_name: str = None):
		self.to = to
		self.type: Type = type
		self.mandatory = mandatory
		self.xml_name = xml_name

	def apply(self, obj: object, value: Union[str, ElementTree.Element], context: Union[Context, None] = None):
		'''Apply value to object as defined in the binding'''
		if self.to is None:
			raise ParsingException("destination is not defined inside the binding")

		# If given type provide a parsing option
		if hasattr(self.type, "parse"):
			if context == None:
				raise ParsingException("missing context for {}", self.type)

			self.apply_casted(obj, getattr(self.type, "parse")(value, None, context, xml_name=self.xml_name))

		# Otherwise call it
		else:
			self.apply_casted(obj, self.type(value))
		

	@property
	def xml_name(self):
		if self.__xml_name != None:
			return self.__xml_name
		elif hasattr(self.type, "xml_name"):
			return self.type.xml_name
		else:
			return None

	@xml_name.setter
	def xml_name(self, xml_name):
		self.__xml_name = xml_name

	def apply_casted(self, obj: object, value: Any):
		setattr(obj, self.to, value)
		

class BindList(Bind):
	def apply_casted(self, obj: object, value: Any):
		'''Apply value to object as defined in the binding'''

		# Check that the attribute is a list
		if not hasattr(obj, self.to) or not isinstance(getattr(obj, self.to), list):
			raise ParsingException("{} does not exists or is not a list".format(self.to))
		
		getattr(obj, self.to).append(value)

class BindDict(Bind):
	'''
		Define a binding between XML element and dict property

		Contains a key attribute that should be obtained in values provided,
		and become the key of the object

		Add a *post-cast* attribute that cast the object after it's key has been extracted
	'''
	def __init__(self, key: str, to: Optional[str] = None, type: Type = str, mandatory = False, xml_name = None, post_cast = None):
		super().__init__(to=to, type=type, mandatory=mandatory, xml_name=xml_name)

		self.key: str = key
		self.post_cast: Type = post_cast

	def apply_casted(self, obj: object, value: Any):
		'''Apply value to object as defined in the binding'''

		# Check that attribute is a dict
		if not hasattr(obj, self.to) or not isinstance(getattr(obj, self.to), dict):
			raise ParsingException("{} does not exists or is not a list".format(self.to))

		# Check for key values existence
		if not hasattr(value, self.key):
			raise ParsingException("{} does not have a parameter named {}".format(value, self.key))

		# Register to dict
		if self.post_cast != None:
			getattr(obj, self.to)[getattr(value, self.key)] = self.post_cast(value)
		else:
			getattr(obj, self.to)[getattr(value, self.key)] = value