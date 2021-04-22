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
    # def __init__(self) -> None:
        # self.start = None
        # self.goal = None
        # self.hVals = None
        # self.constraints = None
        # pass

    def __init__(self, start, goal, hVals, id, constraints=[]) -> None:
       self.start = start
       self.goal = goal
       self.hVals = hVals
       self.id = id
       self.constraints = constraints

    class Actions(Enum):
        UP = (-1,0)
        DOWN = (1,0)
        LEFT = (0,-1)
        RIGHT =(0,1)
        STAY = (0,0)

    def act(self, location, action):
        return location[0] + action[0], location[1] + action[1]


from single_agent_planner import build_constraint_table, is_constrained
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
        constraintTable = build_constraint_table(agent.constraints, agent.id)
        if not bounds:
            bounds = agent.hVals[root]
        path = None
        while not path:
            if limit and bounds >= limit:
                break
            path = self.__low_level_search(agent, bounds, constraintTable)
            bounds += step
        return path

    # DFS, runs until bounds are reached, or until goal.
    # Backtraces path from goal to build final result.
    def __low_level_search(self, agent, bounds, constraintTable):
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
                    stack += self.__expand_node(node, agent, constraintTable)
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
    def __expand_node(self, node, agent, constraintTable):
        children = []
        for action in Agent.Actions:
            newLoc = agent.act(node.location, (action.value)) 
            newGVal = node.gVal + 1
            if not is_constrained(node.location, newLoc, newGVal, constraintTable):
                children.append(self.Node(newLoc, node, newGVal))
        return children




class IDCBS_Solver:
    def __init__(self) -> None:
        self.nodesGenerated = 0
        pass
    # TODO Add proper constraint handling, and generate more nodes in depth
    #   first order to find solution.
    #
    # problem is a MAPF_Problem object defined above.
    # agentSolver is a class that has a find_path(agent) function. It finds an
    #   optimal path for a single agent. agent is an Agent class from above
    #   used to package data and provide an interface for what the agent can
    #   do.
    # collisionDetector is a class that has a function detect_collisions(paths)
    #   returns the list of collisions from the paths in the node. Paths is a
    #   list of all agents paths for the current iteration.
    # constraintGenerator is a class that has a function generate_constraints(node)
    #   that returns a list of constraints. Node is defined inside this class. Used
    #   node instead of collisions mainly because we can include MDDs and other data
    #   in the node later.
    def find_solution(self, problem, agentSolver, collisionDetector=None, constraintGenerator=None):
        return self.__idcbs(problem, agentSolver, collisionDetector, constraintGenerator)

    # Node for the DFS search tree.
    class Node:
        def __init__(self, nAgents) -> None:
            self.paths = [[] for i in range(nAgents)]
            self.collisions = []
            self.constraints = []
            pass

    def __push(self, stack, node):
        stack.append(node)
        self.nodesGenerated += 1
    
    def __idcbs(self, problem, agentSolver, collisionDetector, constraintGenerator):
        node = self.Node(problem.nAgents)
        for i in range(problem.nAgents):
            agent = Agent(problem.starts[i], \
                        problem.goals[i], \
                        problem.hVals[i], \
                        i)
            node.paths[i] = agentSolver.find_path(agent)
        nodeStack = [node]
        while nodeStack:
            node = nodeStack.pop()
            node.collisions = collisionDetector.detect_collisions(node.paths)
            if not node.collisions:
                return node.paths
            constraints = constraintGenerator.generate_constraints(node)
            for i in constraints:
                child = self.Node(problem.nAgents)
                child.paths = list(node.paths)
                child.constraints = node.constraints + [i]
                agentID = i['agent']
                agent = Agent(problem.starts[agentID], \
                            problem.goals[agentID], \
                            problem.hVals[agentID], \
                            agentID, \
                            constraints=child.constraints)
                child.paths[agentID] = agentSolver.find_path(agent)
                if child.paths[agentID]:
                    self.__push(nodeStack, child)