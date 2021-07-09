#!/usr/bin/env python3
import threading
import logging
from argparse import Namespace

from brain.engine_base.node_base import NodeManager
from brain.nodes.bbb_gpio_node import BBBgpioNode
from brain.nodes.stm32_node import Stm32Node
from brain.nodes.sonar_node import SonarNode
from brain.nodes.actuator_node import ActuatorNode
from brain.nodes.action_node import ActionNode
from brain.nodes.test_node import TestNode_1, TestNode_2
from brain.strategy import deploy_strategy


def start_nodes_engine(args: Namespace):
	# Arguments handeling
	if not args.nocan:
		from brain.nodes.can_node import CanNode
	if args.interactive:
		from brain.nodes.ipython_node import IpythonNode

	# NodeManager singleton
	man = NodeManager.get_instance()

	# Adding all nodes
	if not args.nocan:
		man.add_node(CanNode, kwargs={'dev': 'can1'})
	if args.interactive:
		man.add_node(IpythonNode, kwargs={'locals_ns': locals()})
	man.add_node(BBBgpioNode)
	man.add_node(Stm32Node)
	man.add_node(SonarNode)
	man.add_node(ActuatorNode)
	action_node = man.add_node(ActionNode)
	# man.add_node(TestNode_1, kwargs={'intervale': 1.5})
	# man.add_node(TestNode_2)

	# Starting strategy
	deploy_strategy(action_node)

	# Starting all nodes
	man.start_nodes()

	# Wait for shutdown
	man.shutdown.wait()
	for node in man.get_all_nodes():
		node.stop_event_loop = True

	# Wait for all nodes to stop (which means that the brain has shutdown)
	# for node in man.get_all_nodes():
	# 	for t in node.threads:
	# 		t.join()
	while True:
		T = threading.enumerate()
		for t in reversed(T):
			if isinstance(t, threading._MainThread) or isinstance(t, threading._DummyThread):
				continue
			if t.is_alive():
				t.join()
			break
		else:
			break


if __name__ == '__main__':
	start_nodes_engine()
