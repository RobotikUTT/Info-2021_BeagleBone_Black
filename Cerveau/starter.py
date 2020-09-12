from Cerveau.node_manager.node_manager import NodeManager
from Cerveau.nodes.can_node import CanNode

# TODO: parse arguments (like "simulation" and "debug") and handle them


# NodeManager singleton
man = NodeManager.get_instance()

# Adding all nodes
man.add_node(CanNode, kwargs={'dev': 'can0'})

# Starting all nodes
man.start_nodes()

# TODO: block here. Maybe something like thread.join ? but which thread ? All ?
# What about a killswitch
# Maybe put a logger printer loop ??
