#!/usr/bin/python
import argparse
import glob
import idcbs
from pathlib import Path
from cbs import CBSSolver
from independent import IndependentSolver
from prioritized import PrioritizedPlanningSolver
from visualize import Animation


from single_agent_planner import get_sum_of_cost, get_location, compute_heuristics, a_star

SOLVER = "CBS"

class Standard_Solver:
    def __init__(self, myMap):
        self.myMap = myMap
        
    def find_path(self, agent):
        return a_star(self.myMap, agent.start, agent.goal, agent.hVals, agent.id, agent.constraints)

class Goofy_single_agent_planner:
    def __init__(self, map):
        self.map = map

    # a_star(my_map, start_loc, goal_loc, h_values, agent, constraints : list):
    def find_path(self, agent):
        return a_star(self.map, agent.start, agent.goal, agent.hVals, agent.id, agent.constraints)


def print_mapf_instance(my_map, starts, goals):
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)


def print_locations(my_map, locations):
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)


def import_mapf_instance(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # #rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
    # #agents
    line = f.readline()
    num_agents = int(line)
    # #agents lines with the start/goal positions
    starts = []
    goals = []
    for a in range(num_agents):
        line = f.readline()
        sx, sy, gx, gy = [int(x) for x in line.split(' ')]
        starts.append((sx, sy))
        goals.append((gx, gy))
    f.close()
    return my_map, starts, goals

# This is just an example of how to make a constraint_generator.
# The internals can be anything, as long as the generate_constraints(self, node)
# function returns a list of constraints.
from cbs import disjoint_splitting, standard_splitting
from mdd import generate_mdd
class Constraint_Generator:
    def __init__(self, h_vals, starts):
        self.h_vals = h_vals
        self.starts = starts

    def generate_constraints(self, node):
        mdds = {}
        constraints = []

        for i in node.collisions:
            agents = [i['a1'], i['a2']]
            for agent in agents:
                if mdds.get(agent) is None:
                    start = self.starts[agent]
                    maxCost = self.h_vals[agent][start]
                    mdd = generate_mdd(start, self.h_vals[agent], maxCost)
                    while mdd is None:
                        maxCost += 1
                        mdd = generate_mdd(self.starts[agent], self.h_vals[agent], maxCost)
                    mdds[agent] = mdd

            constraints += disjoint_splitting(i)
        return constraints

    # Deprecated
    def generate_constraints_single(self, collision):
        return standard_splitting(collision)

#  Kept this for testing reasons.
#  Also an example of how it works.


from cbs import detect_collisions as dtc_colisns
class Collision_Detector:
    def detect_collisions(self, paths):
        return dtc_colisns(paths)
    
    def count_collision(self, path1, path2):
        count = 0
        max_time = max(len(path1),len(path2))
        for time in range(max_time):
            # Check Vertex collision
            path_location_left = get_location(path1, time)
            path_location_right = get_location(path2, time)
    
            if path_location_right == path_location_left:
                count += 1
    
            # Check if next time is valid
            if time+1 > max_time:
                return count
    
            # Check Edge collision
            path_location_left_dst = get_location(path1, time+1)
            path_location_right_dst = get_location(path2, time+1)
            if path_location_left == path_location_right_dst and path_location_left_dst == path_location_right:
                count += 1
    
        return count    
    
    def count_collisions(self, paths):
        number_of_paths = len(paths)
        count = 0
        for path_index in range(len(paths)):
            for other_path_index in range(path_index+1,number_of_paths):
                count = self.count_collision(paths[path_index], paths[other_path_index])
        return count


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    parser.add_argument('--batch', action='store_true', default=False,
                        help='Use batch output instead of animation')
    parser.add_argument('--disjoint', action='store_true', default=False,
                        help='Use the disjoint splitting')
    parser.add_argument('--solver', type=str, default=SOLVER,
                        help='The solver to use (one of: {CBS,Independent,Prioritized}), defaults to ' + str(SOLVER))

    args = parser.parse_args()


    result_file = open("results.csv", "w", buffering=1)

    for file in sorted(glob.glob(args.instance)):

        print("***Import an instance***")
        my_map, starts, goals = import_mapf_instance(file)
        print_mapf_instance(my_map, starts, goals)

        if args.solver == "CBS":
            print("***Run CBS***")
            print(file)
            cbs = CBSSolver(my_map, starts, goals)
            paths = cbs.find_solution(args.disjoint)
        elif args.solver == "Independent":
            print("***Run Independent***")
            solver = IndependentSolver(my_map, starts, goals)
            paths = solver.find_solution()
        elif args.solver == "Prioritized":
            print("***Run Prioritized***")
            solver = PrioritizedPlanningSolver(my_map, starts, goals)
            paths = solver.find_solution()
        elif args.solver == "IDCBS":
            print("***Run IDCBS***")
            problem = idcbs.MAPF_Problem(starts, goals, my_map, compute_heuristics)
            solver = idcbs.IDCBS_Solver()

            goofy_solver = Goofy_single_agent_planner(my_map)
            paths = solver.find_solution(problem, goofy_solver, Collision_Detector(), Constraint_Generator(problem.hVals, problem.starts))

        else:
            raise RuntimeError("Unknown solver!")
        # problem = idcbs.MAPF_Problem(start, goal, test_map, compute_heuristics)
        # solver = idcbs.IDCBS_Solver()
        # solution = solver.find_solution(problem, idcbs.IDA_Star(), Collision_Detector(), Constraint_Generator())
        cost = get_sum_of_cost(paths)
        result_file.write("{},{}\n".format(file, cost))


        if not args.batch:
            print("***Test paths on a simulation***")
            animation = Animation(my_map, starts, goals, paths)
            # animation.save("output.mp4", 1.0)
            animation.show()
    result_file.close()
