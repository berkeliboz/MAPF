#!/usr/bin/python
import argparse
import glob
import idcbs
import idcbs_plugins
from pathlib import Path
from cbs import CBSSolver
from independent import IndependentSolver
from prioritized import PrioritizedPlanningSolver
from visualize import Animation
from single_agent_planner import get_sum_of_cost
import time

SOLVER = "CBS"

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

            from single_agent_planner import compute_heuristics

            problem = idcbs.MAPF_Problem(starts, goals, my_map, compute_heuristics)
            mapfSolver = idcbs.IDCBS_Solver()
            singleAgentSolver = idcbs_plugins.Standard_Solver(my_map)
            #collisionDetector = idcbs_plugins.Collision_Detector_A()
            collisionDetector = idcbs_plugins.Collision_Detector_B()
            #constraintGenerator = idcbs_plugins.Basic_Constraint_Generator()
            #constraintGenerator = idcbs_plugins.Disjoint_Constraint_Generator() # TODO: Handle the pruning cases
            constraintGenerator = idcbs_plugins.MDD_Optmized_cnstrnt_gnr8tr(problem.hVals, problem.starts)
            startTime = time.perf_counter()
            paths = mapfSolver.find_solution(problem, singleAgentSolver, collisionDetector, constraintGenerator)
            stopTime = time.perf_counter()
            print("Solution time: {}".format(stopTime - startTime))
            print("Nodes generated: {}".format(mapfSolver.nodesGenerated))
            print("Nodes expanded: {}".format(mapfSolver.nodesExpanded))
            print("Maximum DFS bounds: {}".format(mapfSolver.maximumDFSBounds))
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
