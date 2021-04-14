from typing import Dict, Tuple
from single_agent_planner import move
from collections import deque

# Helper that returns a list of all reachable positions from a given location.
def __next_locations(location):
    return [move(location, i) for i in range(5)]

# Pre:
# Heuristics must be computed (use the one provided in the project)
# Args:
# start: Tuple containing the root location for the mdd.
# heuristics: Heuristics provided as a dictionary lookup using locations.
# Return:
# A dictionary where locations are keys. Each entry contains the nodes
# depth and children.
# Description:
# BFS search over the heuristic lookup.
def gen_mdd(start: Tuple, heuristics: Dict) -> Dict:
    mdd = {}
    queue = deque([start])
    depth = 0
    currH = heuristics[start]
    while queue:
        currLoc = queue.popleft()

        # Depth must increase for h to decrease. Because this is BFS, it will
        # be strictly decreasing. So I used that to determine when depth changes.
        if heuristics[currLoc] < currH:
            depth += 1
            currH = heuristics[currLoc]

        mdd[currLoc] = {'depth': depth, 'children': []}
        next_locs = __next_locations(currLoc)
        for loc in next_locs:
            if loc in heuristics:
                if heuristics[loc] < heuristics[currLoc]:
                    mdd[currLoc].append(loc)
                    queue.append(loc)
    return mdd