## Basic A* single agent planner.
from single_agent_planner import a_star, get_location
class Standard_Solver:
    def __init__(self, myMap):
        self.myMap = myMap
        
    def find_path(self, agent):
        return a_star(self.myMap, agent.start, agent.goal, agent.hVals, agent.id, agent.constraints)


# from single_agent_planner import get_sum_of_cost, get_location, compute_heuristics, a_star

# Basic constraint generator
from cbs import standard_splitting
class Basic_Constraint_Generator:
    def generate_constraints(self, node):
        return standard_splitting(node.collisions[0])

# Disjoint constraint generator
from cbs import disjoint_splitting
class Disjoint_Constraint_Generator:
    def generate_constraints(self, node):
        return disjoint_splitting(node.collisions[0])

# Constraint generator with MDD guided optimizations.
from mdd import generate_mdd, classify_constraints
class MDD_Optmized_cnstrnt_gnr8tr: # :P
    def __init__(self, h_vals, starts):
        self.h_vals = h_vals
        self.starts = starts

    def generate_constraints(self, node):
        mdds = {}
        constraints = []
        constraints_sorted = []
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
            constraints += standard_splitting(i)
            constraints_sorted = classify_constraints(mdds, constraints)
        return [constraints_sorted[0], constraints_sorted[1]]

    # Deprecated
    def generate_constraints_single(self, collision):
        return standard_splitting(collision)


# Collision detector with relaxed count_collision heuristic assuming agents
# can pass through eachother.
from cbs import detect_collisions as dtc_colisns
class Collision_Detector_A:
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



# Collision detector with stricter count_collision heuristic.
# Counts initial collisions only.
from cbs import detect_collisions as dtc_colisns
class Collision_Detector_B:
    def detect_collisions(self, paths):
        return dtc_colisns(paths)
    

    def count_collisions(self, paths):
        return len(dtc_colisns(paths))