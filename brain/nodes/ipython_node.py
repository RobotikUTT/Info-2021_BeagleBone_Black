from ..engine_base.node_base import NodeBase

import os
import signal
import IPython
import logging
import importlib
import pkgutil
from pathlib import Path
from threading import Thread

ipython_logger = logging.getLogger('ipython')


class IpythonNode(NodeBase):

	def __init__(self, locals_ns={}):
		super().__init__()
		self.locals_ns = locals_ns

	def start(self):
		# super().start()
		t_name = self.__class__.__name__
		t = Thread(name=t_name, target=self.start_interpreter, daemon=True)
		self.threads.append(t)
		t.start()

	def start_interpreter(self):
		"""From very little to no idea of what I have done here.\n
		Just so you know: it is an equivalent to 'from module import *' for all submoduels of `brain`.\n
		This way, every class, function and variable definition is accessible trough the Ipython interpreter."""

		def iter_packages(path, prefix, onerror=None):
			""" Find packages recursively, including PEP420 packages """
			yield from pkgutil.walk_packages(path, prefix, onerror)
			namespace_packages = {}
			for path_root in path:
				for sub_path in Path(path_root).iterdir():
					if sub_path.is_dir() and not (sub_path / '__init__.py').exists():
						ns_paths = namespace_packages.setdefault(prefix + sub_path.name, [])
						ns_paths.append(str(sub_path))
			for name, paths in namespace_packages.items():
				yield pkgutil.ModuleInfo(None, name, True)
				yield from iter_packages(paths, name + '.', onerror)

		exclude_paths = ['.git', '.doc', '__pycache__', 'cpp_header_gen']

		path = Path(__file__).parent.parent.parent
		__all__ = []
		_globals = globals()
		for loader, module_name, is_pkg in iter_packages([str(path)], ''):
			if module_name[:5] == 'brain':
				for exclude in exclude_paths:
					if exclude in module_name:
						break
				else:
					__all__.append(module_name)
					_module = importlib.import_module(module_name)
					_globals[module_name] = _module
					for attr in dir(_module):
						if attr[0] != '_':
							_globals[attr] = eval(module_name + '.' + attr)

		_locals = locals()
		_locals.update(self.locals_ns)

		ipython_logger.info('Ipython interpreter started.')
		IPython.embed(locals_ns=_locals, global_ns=_globals, colors='Linux')
		ipython_logger.info('Ipython interpreter closed.')
		try:
			os.kill(os.getppid(), signal.SIGINT)
		except ProcessLookupError:
			pass
