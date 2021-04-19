import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    max_time = max(len(path1),len(path2))
    for time in range(max_time):

        # Check Vertex collision
        path_location_left = get_location(path1, time)
        path_location_right = get_location(path2, time)

        if path_location_right == path_location_left:
            return {'loc': [path_location_left], 'timestep': time}

        # Check if next time is valid
        if time+1 > max_time:
            return None

        # Check Edge collision
        path_location_left_dst = get_location(path1, time+1)
        path_location_right_dst = get_location(path2, time+1)
        if path_location_left == path_location_right_dst and path_location_left_dst == path_location_right:
            return {'loc': [path_location_left,path_location_right], 'timestep': time+1}

    return None

def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    result = []
    number_of_paths = len(paths)
    for path_index in range(len(paths)):
        for other_path_index in range(path_index+1,number_of_paths):
            collision_information = detect_collision(paths[path_index], paths[other_path_index])
            if collision_information:
                result.append({'a1': path_index, 'a2': other_path_index, 'loc': collision_information['loc'], 'timestep': collision_information['timestep']})
    return result

def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    return [{'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False},
                {'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep'],'positive': False}]

def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly
    agent = random.randint(0,1)
    if agent == 0:
        constraint1 = {'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': True}
        constraint2 = {'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False}
    else:
        loc = collision['loc'][:]
        loc.reverse()
        constraint1 = {'agent': collision['a2'], 'loc': loc, 'timestep': collision['timestep'], 'positive': True}
        constraint2 = {'agent': collision['a2'], 'loc': loc, 'timestep': collision['timestep'], 'positive': False}
    return [constraint1, constraint2]

#
# Please insert this function into "cbs.py" before "class CBSSolver"
# is defined.
#

def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst

class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)
        self.total_opened = 0
        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        self.total_opened+=1
        print(self.total_opened)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=False):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        print(root['collisions'])

        # Task 3.2: Testing
        for collision in root['collisions']:
            print(standard_splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit

        while len(self.open_list) > 0:
            # get node N with lowest f from openlist
            node_P = self.pop_node()
            collision = detect_collisions(node_P['paths'])
            
            # if N has no conflicts, return solution (goal node)
            if len(collision) == 0:
                self.print_results(node_P)
                return node_P['paths']

            collision = collision[0]
            if disjoint:
                constraints = disjoint_splitting(collision)
            else:
                constraints = standard_splitting(collision)

            """
            classify conflicts into types
            C <- choose conflict(N)
            Children <- []
            for each agent in C,
                generate child node N'
                if N'.cost = N.cost and N' has less conflicts, then:
                    N.solution <- N'.solution
                    N.conflicts <- N'.conflicts
                    Children <- [N]
                    break
                insert N' into Children
            insert Children into openlist             
                    
        return no solution
            """


            for constraint in constraints:
                node_Q = {'cost': 0,'constraints': node_P['constraints'] + [constraint] ,'paths': list(node_P['paths']),'collisions': []}
                agent = constraint['agent']
                new_path = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent],
                    agent, node_Q['constraints'])
                if new_path is not None:
                    node_Q['paths'][agent] = new_path
                    pruned = False
                    if constraint['positive']:
                        pruned = False
                        violate_agents = paths_violate_constraint(constraint, node_Q['paths'])
                        for violate_agent_idx in violate_agents:
                            node_Q['constraints'].append({'agent': violate_agent_idx, 'loc': constraint['loc'],
                                                     'timestep': constraint['timestep'], 'positive': False})
                            possible_path = a_star(self.my_map, self.starts[violate_agent_idx],
                                               self.goals[violate_agent_idx], self.heuristics[violate_agent_idx],
                                               violate_agent_idx, node_Q['constraints'])
                            if possible_path:
                                node_Q['paths'][violate_agent_idx] = possible_path
                            else:
                                pruned = True
                                break
                    if not pruned:
                        node_Q['collisions'] = detect_collisions(node_Q['paths'])
                        node_Q['cost'] = get_sum_of_cost(node_Q['paths'])
                        self.push_node(node_Q)

        self.print_results(root)
        return root['paths']







    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
