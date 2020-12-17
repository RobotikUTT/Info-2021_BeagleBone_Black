#!/usr/bin/env python3
from Cerveau.node_manager.node_manager import NodeManager
# from Cerveau.nodes.can_node import CanNode
from Cerveau.nodes.test_node import TestNode_1, TestNode_2


def start_nodes_engine():
	# NodeManager singleton
	man = NodeManager.get_instance()

	# Adding all nodes
	# man.add_node(CanNode, kwargs={'dev': 'can0'})
	man.add_node(TestNode_1, kwargs={'intervale': 1.5})
	man.add_node(TestNode_2)

	# Starting all nodes
	man.start_nodes()

	# TODO: wait for readyness

	# Wait for all nodes to stop (which means that the brain has shutdown)
	for node in man.get_all_nodes():
		for t in node.threads:
			t.join()


if __name__ == '__main__':
	start_nodes_engine()
