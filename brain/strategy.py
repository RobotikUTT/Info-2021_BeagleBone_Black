from math import pi

from .actions.action_misc import ActionSleep, ActionShutdown, ActionDisableSonar
from .actions.action_bbb_gpio import ActionWaitForTirette
from .actions.action_goto import ActionAsservSetup, ActionAsservStop, ActionPWM, ActionGoto
from .events.stm32_events import DIRECTION_FORWARD, DIRECTION_BACKWARD
from .actions.action_actuator import ActionHandsUp, ActionHandsDown, ActionOpenClaws, ActionCloseClaws, ActionMoveClawsSupportUp, ActionMoveClawsSupportDown, ActionDisableStepper
from .nodes.stm32_node import Stm32Node

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


def test_recul_blue(action_node):
	start_pos = (0, 0, 0.0)
	PID_left = (0.24, 0.0, 20.0)
	PID_right = (0.13, 0.0, 20.0)
	action_node.add_action(ActionHandsUp())
	action_node.add_action(ActionAsservSetup(start_pos, PID_left, PID_right))
	action_node.add_action(ActionSleep(0.02))

	action_node.add_action(ActionWaitForTirette())


	action_node.add_action(ActionSleep(0.2))
	action_node.add_action(ActionPWM(180, 180, 1.0))
	action_node.add_action(ActionSleep(2))
	action_node.add_action(ActionPWM(-162, -162, 0.1))


	action_node.add_action(ActionSleep(4))
	action_node.add_action(ActionPWM(-160, -160, 3.0))



	action_node.add_action(ActionDisableStepper())

	action_node.add_action(ActionSleep(1))
	action_node.add_action(ActionShutdown())


# Rack V1 Bleu
def deploy_strategy_rack_v1_blue(action_node):
	start_pos = (925, 134, pi / 2)
	PID_left = (0.24, 0.0, 20.0)
	PID_right = (0.13, 0.0, 20.0)
	action_node.add_action(ActionHandsUp())
	action_node.add_action(ActionAsservSetup(start_pos, PID_left, PID_right))
	action_node.add_action(ActionSleep(0.02))

	action_node.add_action(ActionWaitForTirette())

	action_node.add_action(ActionGoto(925, 670, DIRECTION_FORWARD))
	action_node.add_action(ActionSleep(0.2))
	action_node.add_action(ActionGoto(1600, 670, DIRECTION_FORWARD))
	action_node.add_action(ActionSleep(0.2))
	action_node.add_action(ActionDisableSonar())
	action_node.add_action(ActionSleep(0.2))
	action_node.add_action(ActionGoto(1600, 80, DIRECTION_FORWARD))

	action_node.add_action(ActionSleep(0.2))
	action_node.add_action(ActionPWM(180, 180, 1.0))
	action_node.add_action(ActionSleep(1))
	action_node.add_action(ActionPWM(-162, -162, 0.2))
	action_node.add_action(ActionSleep(1))

	# Chope gobby
	action_node.add_action(ActionOpenClaws(set(range(5))))

	action_node.add_action(ActionMoveClawsSupportUp())
	action_node.add_action(ActionMoveClawsSupportDown(3000))
	action_node.add_action(ActionSleep(0.2))

	action_node.add_action(ActionCloseClaws(set(range(5))))
	action_node.add_action(ActionSleep(0.2))
	action_node.add_action(ActionMoveClawsSupportUp())
	action_node.add_action(ActionSleep(0.2))

	action_node.add_action(ActionSleep(0.2))
	action_node.add_action(ActionPWM(-160, -160, 3.0))
	action_node.add_action(ActionSleep(4))

	x, y, angle = Stm32Node.current_pos['x'], Stm32Node.current_pos['y'], Stm32Node.current_pos['angle']

	action_node.add_action(ActionAsservStop())
	action_node.add_action(ActionSleep(1))

	start_pos = (x, y, angle)

	PID_left = (0.24, 0.0, 20.0)
	PID_right = (0.23, 0.0, 20.0)

	action_node.add_action(ActionAsservSetup(start_pos, PID_left, PID_right))
	action_node.add_action(ActionSleep(1))

	action_node.add_action(ActionGoto(1600, 150, DIRECTION_FORWARD))
	action_node.add_action(ActionGoto(450, 150, DIRECTION_FORWARD))


	action_node.add_action(ActionMoveClawsSupportDown(3200))
	action_node.add_action(ActionSleep(0.2))
	action_node.add_action(ActionOpenClaws(set(range(5))))
	action_node.add_action(ActionSleep(0.2))
	action_node.add_action(ActionMoveClawsSupportUp())
	# End chop gobby

	action_node.add_action(ActionDisableStepper())

	action_node.add_action(ActionSleep(1))
	action_node.add_action(ActionShutdown())


# Rack V1 Bleu
def deploy_strategy_rack_v1_yellow(action_node):
	start_pos = (925, 2866, -pi / 2)
	PID_left = (0.24, 0.0, 20.0)
	PID_right = (0.23, 0.0, 20.0)
	action_node.add_action(ActionHandsUp())
	action_node.add_action(ActionAsservSetup(start_pos, PID_left, PID_right))
	action_node.add_action(ActionSleep(0.02))

	action_node.add_action(ActionWaitForTirette())

	action_node.add_action(ActionGoto(925, 2555, DIRECTION_FORWARD))
	action_node.add_action(ActionSleep(0.2))
	action_node.add_action(ActionGoto(1600, 255, DIRECTION_FORWARD))
	action_node.add_action(ActionSleep(0.2))
	action_node.add_action(ActionDisableSonar())
	action_node.add_action(ActionSleep(0.2))
	action_node.add_action(ActionGoto(1600, 2920, DIRECTION_FORWARD))

	action_node.add_action(ActionSleep(0.2))
	action_node.add_action(ActionPWM(180, 180, 1.0))
	action_node.add_action(ActionSleep(1))
	action_node.add_action(ActionPWM(-162, -162, 0.2))
	action_node.add_action(ActionSleep(1))

	# Chope gobby
	action_node.add_action(ActionOpenClaws(set(range(5))))

	action_node.add_action(ActionMoveClawsSupportUp())
	action_node.add_action(ActionMoveClawsSupportDown(3000))
	action_node.add_action(ActionSleep(0.2))

	action_node.add_action(ActionCloseClaws(set(range(5))))
	action_node.add_action(ActionSleep(0.2))
	action_node.add_action(ActionMoveClawsSupportUp())
	action_node.add_action(ActionSleep(0.2))

	action_node.add_action(ActionSleep(0.2))
	action_node.add_action(ActionPWM(-160, -160, 3.0))
	action_node.add_action(ActionSleep(4))

	x, y, angle = Stm32Node.current_pos['x'], Stm32Node.current_pos['y'], Stm32Node.current_pos['angle']

	action_node.add_action(ActionAsservStop())
	action_node.add_action(ActionSleep(1))

	start_pos = (x, y, angle)

	PID_left = (0.24, 0.0, 20.0)
	PID_right = (0.23, 0.0, 20.0)

	action_node.add_action(ActionAsservSetup(start_pos, PID_left, PID_right))
	action_node.add_action(ActionSleep(1))




	action_node.add_action(ActionGoto(1600, 2800, DIRECTION_FORWARD))
	action_node.add_action(ActionGoto(450, 2800, DIRECTION_FORWARD))


	action_node.add_action(ActionMoveClawsSupportDown(3200))
	action_node.add_action(ActionSleep(0.2))
	action_node.add_action(ActionOpenClaws(set(range(5))))
	action_node.add_action(ActionSleep(0.2))
	action_node.add_action(ActionMoveClawsSupportUp())
	# End chop gobby

	action_node.add_action(ActionDisableStepper())

	action_node.add_action(ActionSleep(1))
	action_node.add_action(ActionShutdown())



# Test stepper
def deploy_strategy_tester_stepper(action_node):
	start_pos = (0, 0, 0.0)
	PID_left = (0.24, 0.0, 20.0)
	PID_right = (0.13, 0.0, 20.0)
	action_node.add_action(ActionHandsUp())
	action_node.add_action(ActionAsservSetup(start_pos, PID_left, PID_right))
	action_node.add_action(ActionSleep(0.02))

	# Chope gobby
	action_node.add_action(ActionOpenClaws(set(range(5))))

	action_node.add_action(ActionMoveClawsSupportUp())
	action_node.add_action(ActionMoveClawsSupportDown(3000))
	action_node.add_action(ActionSleep(0.1))

	action_node.add_action(ActionCloseClaws(set(range(5))))
	action_node.add_action(ActionSleep(0.1))
	action_node.add_action(ActionMoveClawsSupportUp())
	action_node.add_action(ActionSleep(0.1))

	action_node.add_action(ActionPWM(-140, -140, 0.2))

	action_node.add_action(ActionMoveClawsSupportDown(3200))
	action_node.add_action(ActionSleep(1))
	action_node.add_action(ActionOpenClaws(set(range(5))))
	action_node.add_action(ActionSleep(1))
	action_node.add_action(ActionMoveClawsSupportUp())
	# End chop gobby

	action_node.add_action(ActionDisableStepper())

	action_node.add_action(ActionSleep(1))
	action_node.add_action(ActionShutdown())


# Bleu homologation
def deploy_strategy_homolog_blue(action_node):
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
def deploy_strategy_homolog_yellow(action_node):
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


deploy_strategy = deploy_strategy_rack_v1_blue
# deploy_strategy = deploy_strategy_rack_v1_yellow



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
