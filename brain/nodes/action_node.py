from ..engine_base.node_base import NodeBase
from ..engine_base.action_base import ActionBase, ActionThread


class ActionNode(NodeBase):
	main_action_thread = None

	def __init__(self):
		super().__init__()
		self.main_action_thread = ActionThread(self)

	def start(self):
		super().start()
		self.threads.append(self.main_action_thread)
		self.main_action_thread.start()

	def add_action(self, action: ActionBase):
		self.main_action_thread.schedule_action(action)

	def add_action_in_new_thread(self, action: ActionBase):
		new_action_thread = ActionThread(self)
		self.threads.append(new_action_thread)
		new_action_thread.schedule_action(action)
		new_action_thread.start()
