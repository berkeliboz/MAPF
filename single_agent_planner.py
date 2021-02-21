import heapq

def move(loc, dir):
    directions = [(0, 0), (0, 1), (1, 0), (0, -1), (-1, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(5):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints : list, agent):
    ##############################
    # Task 1.2/1.3: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.
    constraint_table = dict()

    for index in constraints:
        for set_value in index:
            constraint = {
                'agent' : agent,
                'time' : set_value[0],
                'location' : set_value[1]
            }
        if constraint_table.get(set_value[0]):
            constraint_table.get(set_value[0]).append(constraint)
        else:
            const_list = list()
            const_list.append(constraint)
            constraint_table[set_value[0]] = const_list

    #Min Sum Costs + 1.5 answer
    # add_constraint_to_table(constraint_table, agent,
    # {
    #     'agent': 1,
    #     'time': 2,
    #     'location': (1, 4),
    # })
    # add_constraint_to_table(constraint_table, agent,
    # {
    #     'agent': 1,
    #     'time': 2,
    #     'location': (1, 3)
    # })
    # add_constraint_to_table(constraint_table, agent,
    # {
    #     'agent': 1,
    #     'time': 2,
    #     'location': (1, 2)
    # })

    return constraint_table

def add_constraint_to_table(constraint_table, agent, constraint):
    local_agent = constraint['agent']
    if agent != local_agent:
        return

    location = constraint['time']
    if constraint_table.get(location):
        constraint_table[location].append(constraint)
    else:
        constraint_table[location] = list()
        constraint_table[location].append(constraint)
    return

def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path

def calculate_constraint(constraint,curr_loc, next_loc):
    location_constraints = constraint['location']

    len = location_constraints.count(1)
    if len == 0 and \
            (location_constraints[0] == curr_loc and location_constraints[1] == next_loc) \
            or (location_constraints[1] == curr_loc and location_constraints[0] == next_loc):
        return True
    else:
        if location_constraints == next_loc:
            return True
    return False

def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.
    if constraint_table:
        constraint_list = constraint_table.get(next_time)
        if not constraint_list:
            constraint_list = constraint_table.get(-1)
        for constraint in constraint_list:
            if calculate_constraint(constraint,curr_loc,next_loc):
                return True
    return False


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints : list):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    constraint_table = build_constraint_table(constraints, agent)
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None}
    push_node(open_list, root)
    closed_list[(root['loc'],root['g_val'])] = root
    while len(open_list) > 0:
        curr = pop_node(open_list)
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        if curr['loc'] == goal_loc:

            ###GOAL CHECK BEGIN####
            #######################
            time_bound = False
            key_list = constraint_table.keys()
            for key in key_list:
                contraint_list = list(constraint_table[key])
                for constraints_collection in contraint_list:
                    if type(constraints_collection) != dict:
                        for constraint in constraints_collection:
                            if constraint.get("location")[0] == goal_loc:
                                if constraint.get("time") >= curr['g_val']:
                                    time_bound = True
                                    break
                    else:
                        if constraints_collection.get('location') == goal_loc:
                            if constraints_collection.get("time") >= curr['g_val']:
                                time_bound = True
                                break
            if not time_bound:
                return get_path(curr)
            #####################
            ###GOAL CHECK END####

        for dir in range(5):
            child_loc = move(curr['loc'], dir)
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            if is_constrained(curr['loc'],child_loc,curr['g_val'] + 1, constraint_table):
                continue
            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'parent': curr}
            if (child['loc'], child['g_val']) in closed_list:
                existing_node = closed_list[(child['loc'], child['g_val'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['g_val'])] = child
                    # constraints.append({
                    #         (child['g_val'], child['loc'])
                    # })

                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['g_val'])] = child
                # constraints.append({
                #     (child['g_val'], child['loc'])
                # })
                push_node(open_list, child)

    return None  # Failed to find solutions
