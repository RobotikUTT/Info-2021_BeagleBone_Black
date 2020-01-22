from typing import List, Dict, Union


class Argumentable:
	"""
		This object make it easier to manipulate variant type argument.
	"""

	def __init__(self, values=None):
		self.argument = ["", 0]
		# Convert to string
		if values is not None:
			for name in values:
				values[name] = str(values[name])
		else:
			values = {}

		self.values: Dict[str, str] = values
		
	def get(self, name: str, default = None) -> str:
		if name in self.values:
			return self.values[name]
		else:
			return default
		
	def set(self, name: str, value: Union[str, int, float]) -> None:
		"""
			Set value to attr with given value
		"""
		self.values[name] = value.__str__()

	def has(self, name: str):
		if name in self.values:
			try:
				type(self.values[name])
			except ValueError:
				return False
			return True
		return False

	def keys(self):
		return self.values.keys()

	def from_list(self, args: list, reset: bool = False) -> 'Argumentable':
		if reset:
			self.values = {}

		for arg in args:
			self.values[arg.name] = arg.value
		
		return self
	
	def to_list(self) -> list:
		result = []

		for key in self.values:
			arg=self.argument
			arg[0] = key
			arg[1] = self.values[key]

			result.append(arg)

		return result
	
	def __str__(self):
		return ", ".join(["{} = {}".format(name, self.values[name]) for name in self.values])