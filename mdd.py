################################################################################
# Adam Spilchen
# Berke Boz


################################################################################
from enum import Enum
from typing import Dict, List, Tuple
from single_agent_planner import move
from collections import deque


# Node to hold data. Only children IDs are stored here. Use them to access
# the actual node in the MDD.
class MDD_Node:
    def __init__(self, id: int, location: Tuple, depth: int) -> None:
        self.id = id
        self.location = location  # Tuple
        self.depth = depth  # Integer
        self.children = []  # List containing children IDs
        pass


# Holds all the nodes for an MDD. Nodes are stored in a list, use node ID as lookup.
# Access nodes through <object>.nodes[<id>]
# Or through children_of(<node>)
# Node IDs should start from 0 as root. Child IDs are found in the node itself.
class MDD:
    def __init__(self, root) -> None:
        self.root = root
        self.nodes = [self.root]
        pass

    # Used during the construction of the MDD.
    def add_node(self, node: object, parent=None):
        if parent:
            parent.children.append(node.id)
        self.nodes.append(node)

    # Returns a list of all the concrete children of the given node.
    def children_of(self, node: object) -> List:
        return [self.nodes[i] for i in node.children]


# Helper that returns a list of all reachable positions from a given location.
def __next_locations(location):
    return [move(location, i) for i in range(5)]


# Pre:
# Heuristics must be computed (use the one provided in the project)
#
# Args:
# start: Tuple containing the root location for the mdd.
# maxCost: All paths will cost this much.
# heuristics: Heuristics provided as a dictionary lookup using locations.
#
# Return:
# A dictionary where locations are keys. Each entry contains the nodes
# depth and children.
#
# Description:
# BFS search over the heuristic lookup, using the heuristics to guide search
# directions. Uses a closed list to prevent duplicate node creation. When duplicates
# are found, they are just added as children to the current sub root.
#
# TODO: Add constraint handling
def generate_mdd(start: Tuple, maxCost: int, heuristics: Dict) -> MDD:
    # Optimal path costs more than the requested cost
    if maxCost < heuristics[start]:
        return None

    number = 0
    depth = 0
    node = MDD_Node(0, start, depth)
    mdd = MDD(node)
    queue = deque([number])
    closedList = {}  # Used to prevent duplicates
    while queue:
        number = queue.popleft()
        node = mdd.nodes[number]
        depth = node.depth
        currLoc = node.location
        # Iterate over all locations reachable from current
        nextLocs = __next_locations(currLoc)
        for loc in nextLocs:
            if loc in heuristics:
                nextDepth = depth + 1
                nextVal = heuristics[loc] + nextDepth
                # If the depth + heuristic is greater than the requested cost, skip node.
                # This is because heuristic decreases and depth increases. So this should
                # always remain the same value, or increase. If max cost is much larger than
                # the optimal path, it will lead to alot of extra wandering before proceeding
                # to the goal.
                if nextVal <= maxCost:
                    if (loc, nextDepth) in closedList:  # duplicate detected
                        # Just add duplicate as a child to the current node, and do not add to queue.
                        # Because this is a duplicate, the path is already calculated from this point on.
                        child = closedList[(loc, nextDepth)]
                        node.children.append(child)

                    else:
                        # Add new node with next depth
                        child = MDD_Node(len(mdd.nodes), loc, nextDepth)
                        closedList[(loc, nextDepth)] = child.id
                        mdd.add_node(child, node)
                        queue.append(child.id)
    return mdd


class collision_type(Enum):
    cardinal = 1
    semi_cardinal = 2
    non_cardinal = 3


def cross_nodes(agent_node: MDD_Node, other_node: MDD_Node, id: int) -> MDD_Node:
    if agent_node.depth != other_node.depth:
        print("Can't cross nodes from different levels")
        return None
    if agent_node.location == other_node.location:
        return None
    return MDD_Node(id, (agent_node, other_node), agent_node.depth)


#
# {'a1': path_index, 'a2': other_path_index, 'loc': collision_information['loc'],
# 'timestep': collision_information['timestep']}
#

def classify_collision(agent_mdd: MDD, other_agent_mdd: MDD, collision):
    if agent_mdd is None or other_agent_mdd is None:
        return None
    loc = collision['loc'][0]

    agent_node_filter = filter(lambda n: n.location == loc,agent_mdd.nodes)
    agent_path = agent_node_filter.__sizeof__() > 1

    other_agent_node_filter = filter(lambda n: n.location == loc,other_agent_mdd.nodes)
    other_path = other_agent_node_filter.__sizeof__() > 1

    if agent_path and other_path:
        return collision_type.non_cardinal
    elif agent_path is False and other_path is False:
        return collision_type.cardinal
    return collision_type.semi_cardinal

def cross_mdds(agent_mdd: MDD, other_agent_mdd: MDD):
    cross_list = []
    number = 0
    agent_open = [agent_mdd.root]
    other_agent_open = [other_agent_mdd.root]

    while len(agent_open) > 0 and len(other_agent_open) > 0:
        agent_set = {}
        other_set = {}

        while len(agent_open) > 0:
            agent = agent_open.pop()

            for other in other_agent_open:
                crossed_node = cross_nodes(agent, other, number)
                if crossed_node:
                    agent_set[agent.id] = agent
                    other_set[other.id] = other
                    number += 1
                    cross_list.append(crossed_node)
        next_level_agent_list = {}
        next_level_other_list = {}
        for agent_list in agent_set.values():
            for agent in agent_mdd.children_of(agent_list):
                next_level_agent_list[agent.id] = agent
        for other_list in other_set.values():
            for other in other_agent_mdd.children_of(other_list):
                next_level_other_list[other.id] = other

        agent_open = list(next_level_agent_list.values())
        other_agent_open = list(next_level_other_list.values())

    return cross_list
