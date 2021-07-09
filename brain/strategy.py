from math import pi

from .actions.action_misc import ActionSleep, ActionShutdown
from .actions.action_bbb_gpio import ActionWaitForTirette
from .actions.action_goto import ActionAsservSetup, ActionAsservStop, ActionPWM, ActionGoto
from .events.stm32_events import DIRECTION_FORWARD, DIRECTION_BACKWARD
from .actions.action_actuator import ActionHandsUp, ActionHandsDown


# def deploy_strategy(action_node):
# 	start_pos = (0, 0, 0.0)
# 	PID_left = (0.24, 0.0, 20.0)
# 	PID_right = (0.13, 0.0, 20.0)
# 	action_node.add_action(ActionAsservSetup(start_pos, PID_left, PID_right))
# 	action_node.add_action(ActionSleep(0.02))

# 	action_node.add_action(ActionWaitForTirette())
# 	action_node.add_action(ActionGoto(500, 0, 0.0, DIRECTION_FORWARD))
# 	action_node.add_action(ActionPWM(-160, -160, 1.5))

# 	# action_node.add_action(ActionGoto(0x900, 0x700, 0.0, DIRECTION_FORWARD))

# 	# action_node.add_action(ActionSleep(1))

# 	# action_node.add_action(ActionGoto(0x700, 0x700, 0.0, DIRECTION_BACKWARD))

# 	action_node.add_action(ActionAsservStop())

# 	action_node.add_action(ActionSleep(1))
# 	action_node.add_action(ActionShutdown())


# def deploy_strategy_(action_node):
# 	start_pos = (0, 0, 0.0)
# 	PID_left = (0.24, 0.0, 20.0)
# 	PID_right = (0.13, 0.0, 20.0)
# 	speed_params = (800, 0.5, 1200)
# 	action_node.add_action(ActionAsservSetup(start_pos, PID_left, PID_right, speed_params))
# 	action_node.add_action(ActionSleep(0.02))

# 	action_node.add_action(ActionWaitForTirette())

# 	action_node.add_action(ActionGoto(600, 0, 0.0, DIRECTION_FORWARD))
# 	action_node.add_action(ActionGoto(600, 500, pi / 2, DIRECTION_FORWARD))
# 	action_node.add_action(ActionGoto(520, 500, pi * 5 / 4, DIRECTION_FORWARD))
# 	action_node.add_action(ActionGoto(270, 250, pi * 5 / 4, DIRECTION_FORWARD))

# 	# action_node.add_action(ActionPWM(-160, -160, 2.0))

# 	action_node.add_action(ActionAsservStop())

# 	action_node.add_action(ActionSleep(1))
# 	action_node.add_action(ActionShutdown())


# Bleu
def deploy_strategy(action_node):
	start_pos = (0, 0, 0.0)
	PID_left = (0.24, 0.0, 20.0)
	PID_right = (0.13, 0.0, 20.0)
	action_node.add_action(ActionHandsUp())
	action_node.add_action(ActionAsservSetup(start_pos, PID_left, PID_right))
	action_node.add_action(ActionSleep(0.02))

	action_node.add_action(ActionWaitForTirette())

	action_node.add_action(ActionGoto(600, 0, DIRECTION_FORWARD))
	action_node.add_action(ActionGoto(600, 500, DIRECTION_FORWARD))
	action_node.add_action(ActionGoto(520, 500, DIRECTION_FORWARD))
	action_node.add_action(ActionGoto(270, 250, DIRECTION_FORWARD))

	# action_node.add_action(ActionBaisserBras())

	action_node.add_action(ActionPWM(-160, -160, 2.0))
	action_node.add_action(ActionGoto(150, 500, DIRECTION_FORWARD))

	action_node.add_action(ActionAsservStop())

	action_node.add_action(ActionSleep(1))
	action_node.add_action(ActionShutdown())


# Jaune
def deploy_strategy_(action_node):
	start_pos = (0, 0, 0.0)
	PID_left = (0.24, 0.0, 20.0)
	PID_right = (0.13, 0.0, 20.0)
	action_node.add_action(ActionHandsUp())
	action_node.add_action(ActionAsservSetup(start_pos, PID_left, PID_right))
	action_node.add_action(ActionSleep(0.02))

	action_node.add_action(ActionWaitForTirette())

	action_node.add_action(ActionGoto(600, 0, DIRECTION_FORWARD))
	action_node.add_action(ActionGoto(600, -500, DIRECTION_FORWARD))
	action_node.add_action(ActionGoto(520, -500, DIRECTION_FORWARD))
	action_node.add_action(ActionGoto(270, -250, DIRECTION_FORWARD))

	# action_node.add_action(ActionBaisserBras())

	action_node.add_action(ActionPWM(-160, -160, 2))
	action_node.add_action(ActionGoto(150, -500, DIRECTION_FORWARD))

	action_node.add_action(ActionAsservStop())

	action_node.add_action(ActionSleep(1))
	action_node.add_action(ActionShutdown())


# def deploy_strategy(action_node):
# 	start_pos = (925, 134, pi / 2)
# 	PID_left = (0.24, 0.0, 20.0)
# 	PID_right = (0.13, 0.0, 20.0)
# 	speed_params = (800, 0.5, 1200)

# 	action_node.add_action(ActionHandsUp())

# 	action_node.add_action(ActionWaitForTirette())
# 	action_node.add_action(ActionAsservSetup(start_pos, PID_left, PID_right, speed_params))
# 	action_node.add_action(ActionSleep(0.02))

# 	action_node.add_action(ActionGoto(925, 200, DIRECTION_FORWARD))
# 	action_node.add_action(ActionGoto(1830, 200, DIRECTION_FORWARD))

# 	action_node.add_action(ActionGoto(1820, 500, DIRECTION_FORWARD))


# 	action_node.add_action(ActionGoto(1820, 780, DIRECTION_FORWARD))

# 	action_node.add_action(ActionPWM(-170, -170, 8.0))
# 	action_node.add_action(ActionHandsDown())


# 	action_node.add_action(ActionGoto(1830, 790, DIRECTION_FORWARD))
	
# 	action_node.add_action(ActionHandsUp())
# 	action_node.add_action(ActionPWM(-160, -160, 3.0))

# 	# action_node.add_action(ActionGoto(1700, 635, pi, DIRECTION_FORWARD))
# 	action_node.add_action(ActionGoto(150, 850, DIRECTION_FORWARD))
# 	action_node.add_action(ActionPWM(160, 160, 1.0))
# 	action_node.add_action(ActionPWM(-160, -160, 0.3))

# 	# Actionneurs
# 	action_node.add_action(ActionSleep(1))		# Fake action

# 	action_node.add_action(ActionPWM(-160, -160, 2.0))
# 	action_node.add_action(ActionGoto(750, 200, DIRECTION_FORWARD))

# 	# Poser les bouées

# 	action_node.add_action(ActionAsservStop())

# 	action_node.add_action(ActionSleep(1))
# 	action_node.add_action(ActionShutdown())
