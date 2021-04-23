import idcbs

# Used berkes detect_collisions function.
from cbs import detect_collisions as dtc_colisns
class Collision_Detector:
    def detect_collisions(self, paths):
        return dtc_colisns(paths)


# This is just an example of how to make a constraint_generator.
# The internals can be anything, as long as the generate_constraints(self, node)
# function returns a list of constraints.
from cbs import disjoint_splitting
class Disjoint_Splitter:
    def generate_constraints(self, node):
        constraints = []
        for i in node.collisions:
            constraints += disjoint_splitting(i)
        return constraints


# This is just an example of how to make a constraint_generator.
# The internals can be anything, as long as the generate_constraints(self, node)
# function returns a list of constraints.
from cbs import standard_splitting
class Regular_Splitter:
    def generate_constraints(self, node):
        constraints = []
        for i in node.collisions:
            constraints += standard_splitting(i)
        return constraints

# This is just an example of how to make a constraint_generator.
# The internals can be anything, as long as the generate_constraints(self, node)
# function returns a list of constraints.
from cbs import standard_splitting
class Goofy_Splitter:
    def generate_constraints(self, node):
        constraints = []
        for i in node.collisions:
            if i['agent'] == 0:
                constraints += standard_splitting(i)
        return constraints

#def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints : list):

height = 5
width = 5
test_map = [[False for i in range(height)] for j in range(width)]
from single_agent_planner import a_star
class Standard_Solver:
    def find_path(self, agent):
        return a_star(test_map, agent.start, agent.goal, agent.hVals, agent.id, agent.constraints)



#  Kept this for testing reasons.
#  Also an example of how it works.

from single_agent_planner import compute_heuristics
start = [(0,2), (2,0)]
goal = [(3,3), (0,3)]
problem = idcbs.MAPF_Problem(start, goal, test_map, compute_heuristics)
solver = idcbs.IDCBS_Solver()
# solution = solver.find_solution(problem, idcbs.IDA_Star(), Collision_Detector(), Goofy_Splitter())
# solution = solver.find_solution(problem, idcbs.IDA_Star(), Collision_Detector(), Constraint_Generator())
solution = solver.find_solution(problem, Standard_Solver(), Collision_Detector(), Regular_Splitter())

# prints heuristics for the map in the correct shape
#for i in range(height):
#    for j in range(width):
#        if problem.hVals[0][(i,j)]:
#            print("{:3}".format(problem.hVals[0][(i,j)]), end='')
#    print()

print(solution[0])
print(solution[1])
print(solver.nodesGenerated)