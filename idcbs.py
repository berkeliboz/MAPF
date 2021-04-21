from cbs import detect_collisions as dtc_colisns
from collections import deque
from enum import Enum

# Used to hold all initial data for a MAPF problem.
class MAPF_Problem:
    def __init__(self, starts, goals, worldMap, heuristics=None) -> None:
        self.nAgents = len(starts)
        self.starts = starts
        self.goals = goals
        self.worldMap = worldMap
        if heuristics:
            self.hVals = [heuristics(worldMap, i) for i in goals]
        else:
            self.hVals = None


# Used to pass single agent data between functions.
class Agent:
    def __init__(self) -> None:
        self.start = None
        self.goal = None
        self.hVals = None
        self.constraints = None
        pass

    #def __init__(self, start, goal, hVals, constraints=None) -> None:
    #    self.start = start
    #    self.goal = goal
    #    self.hVals = hVals
    #    self.constraints = constraints

    class Actions(Enum):
        UP = (-1,0)
        DOWN = (1,0)
        LEFT = (0,-1)
        RIGHT =(0,1)
        STAY = (0,0)

    def act(self, location, action):
        return location[0] + action[0], location[1] + action[1]

# Not finished. Stateless IDA_star implementation used as a single agent solver in IDCBS.
# TODO Add constraint handling
class IDA_Star:

    # Interface
    # agent holds all required information for a single agent to be solved.
    # Step size determines how much to increase the bounds each iteration (default 1)
    # Initial bounds is optional for testing.
    # Limit is the maximum search bounds. Search will terminate once bounds reaches limit.
    # It is recommended this is calculated using information about the map, and used to ensure
    # search times are not endless.
    def find_path(self, agent, step=1, bounds=None, limit=None):
        return self.__high_level_search(agent, step, bounds, limit)

    # Search node data.
    class Node:
        def __init__(self, location, parent, gVal) -> None:
            self.location = location
            self.parent = parent
            self.gVal = gVal
            pass

    # Starts with optimal search bounds and increments by step size
    # until a solution is found, or until the bounds has reached the
    # limit.
    def __high_level_search(self, agent, step, bounds, limit):
        root = agent.start
        if not bounds:
            bounds = agent.hVals[root]
        path = None
        while not path:
            if limit and bounds >= limit:
                break
            path = self.__low_level_search(agent, bounds)
            bounds += step
        return path

    # DFS, runs until bounds are reached, or until goal.
    # Backtraces path from goal to build final result.
    def __low_level_search(self, agent, bounds):
        node = self.Node(agent.start, None, 0)
        stack = [node]
        while stack:
            node = stack.pop()
            if node.location == agent.goal:
                break
            if node.location in agent.hVals:
                fVal = node.gVal + agent.hVals[node.location]
                # Only add children that do not exceed the bounds
                if fVal < bounds:
                    stack += self.__expand_node(node, agent)
        if node.location == agent.goal:
            return self.__back_trace(node)
        else:
            return None

    # Used to backtrace from the goal to the parent.
    # Uses a deque and pushes to the front, so path is in correct order.
    def __back_trace(self, node):
        n = node
        path = deque()
        while n:
            path.appendleft(n.location)
            n = n.parent
        return path

    # Generates all children for the provided node.
    # Uses the agent to perform move actions.
    def __expand_node(self, node, agent):
        children = []
        for action in Agent.Actions:
            newLoc = agent.act(node.location, (action.value)) 
            newGVal = node.gVal + 1
            children.append(self.Node(newLoc, node, newGVal))
        return children


class Collision_Detector:
    def detect_collisions(self, paths):
        return dtc_colisns(paths)

class IDCBS_Solver:

    def find_solution(self, problem, agentSolver, collisionDetector=None):
        nodes = []
        node = self.Node(problem.nAgents)
        for i in range(problem.nAgents):
            agent = Agent()
            agent.start = problem.starts[i]
            agent.goal = problem.goals[i]
            agent.hVals = problem.hVals[i]
            node.paths[i] = agentSolver.find_path(agent)
        nodes.append(node)
        if collisionDetector:
            node.collisions = collisionDetector.detect_collisions(node.paths)
        return nodes[-1]

    class Node:
        def __init__(self, nAgents) -> None:
            self.paths = [[] for i in range(nAgents)]
            self.collisions = None
            self.constraits = None
            pass


# Kept this for testing reasons.
# Also an example of how it works.

#from single_agent_planner import compute_heuristics
#test_map = [[False for i in range(20)] for j in range(20)]
#start = [(1,1), (1,19)]
#goal = [(15,15), (19,1)]
#problem = MAPF_Problem(start, goal, test_map, compute_heuristics)
#solver = IDA_Star()
#idcbs = IDCBS_Solver()
#node = idcbs.find_solution(problem, IDA_Star(), Collision_Detector())