import idcbs
from mdd import classify_collisions, generate_mdd

from single_agent_planner import compute_heuristics
height = 5
width = 5
test_map = [[False for i in range(height)] for j in range(width)]
start = [(0,2), (2,0)]
goal = [(3,3), (0,3)]
problem = idcbs.MAPF_Problem(start, goal, test_map, compute_heuristics)
solver = idcbs.IDCBS_Solver()
solution = solver.find_solution(problem, idcbs.IDA_Star(), Collision_Detector(), Constraint_Generator())

# prints heuristics for the map in the correct shape
#for i in range(height):
#    for j in range(width):
#        if problem.hVals[0][(i,j)]:
#            print("{:3}".format(problem.hVals[0][(i,j)]), end='')
#    print()

print(solution[0])
print(solution[1])
print(solver.nodesGenerated)