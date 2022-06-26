# Ex.NO.:02
# Date:28.04.2022
# Breadth First Search
## AIM

To develop an algorithm to find the route from the source to the destination point using breadth-first search.

## THEORY
Breadth-first search, also known as BFS, finds shortest paths from a given source vertex to all other vertices, in terms of the number of edges in the paths.

## DESIGN STEPS

### STEP 1:
Identify a location in the google map:

### STEP 2:
Select a specific number of nodes with distance

### STEP 3:
Import required packages.

### STEP 4:
Include each node and its distance separately in the dictionary data structure.

### STEP 5:
End of program.


## ROUTE MAP
#### Example map

![map](https://user-images.githubusercontent.com/75235209/166288584-cd7c87d6-5563-476f-b507-966ac57e3513.PNG)


## PROGRAM
```python
Student name : Swetha.k.p
Reg.no : 212220230053
 ```
 ```python
 %matplotlib inline
import matplotlib.pyplot as plt
import random
import math
import sys
from collections import defaultdict, deque, Counter
from itertools import combinations

class Problem(object):
    def __init__(self, initial=None, goal=None, **kwds): 
        self.__dict__.update(initial=initial, goal=goal, **kwds) 
        
    def actions(self, state):        
        raise NotImplementedError
    def result(self, state, action): 
        raise NotImplementedError
    def is_goal(self, state):        
        return state == self.goal
    def action_cost(self, s, a, s1): 
        return 1
    
    def __str__(self):
        return '{0}({1}, {2})'.format(
            type(self).__name__, self.initial, self.goal)
            
class Node:
    def __init__(self, state, parent=None, action=None, path_cost=0):
        self.__dict__.update(state=state, parent=parent, action=action, path_cost=path_cost)

    def __str__(self): 
        return '<{0}>'.format(self.state)
    def __len__(self): 
        return 0 if self.parent is None else (1 + len(self.parent))
    def __lt__(self, other): 
        return self.path_cost < other.path_cost
        
failure = Node('failure', path_cost=math.inf) 
cutoff  = Node('cutoff',  path_cost=math.inf)

def expand(problem, node):
    "Expand a node, generating the children nodes."
    s = node.state
    for action in problem.actions(s):
        s1 = problem.result(s, action)
        cost = node.path_cost + problem.action_cost(s, action, s1)
        yield Node(s1, node, action, cost)
        

def path_actions(node):
    "The sequence of actions to get to this node."
    if node.parent is None:
        return []  
    return path_actions(node.parent) + [node.action]


def path_states(node):
    "The sequence of states to get to this node."
    if node in (cutoff, failure, None): 
        return []
    return path_states(node.parent) + [node.state]
    
FIFOQueue = deque

def breadth_first_search(problem):
    "Search shallowest nodes in the search tree first."
    node = Node(problem.initial)
    if problem.is_goal(problem.initial):
        return node
    frontier = FIFOQueue([node])
    reached = {problem.initial}
    while frontier:
        node = frontier.pop()
        for child in expand(problem, node):
            s = child.state
            if problem.is_goal(s):
                return child
            if s not in reached:
                reached.add(s)
                frontier.appendleft(child)
    return failure
    
class RouteProblem(Problem):
    """A problem to find a route between locations on a `Map`.
    Create a problem with RouteProblem(start, goal, map=Map(...)}).
    States are the vertexes in the Map graph; actions are destination states."""
    
    def actions(self, state): 
        """The places neighboring `state`."""
        return self.map.neighbors[state]
    
    def result(self, state, action):
        """Go to the `action` place, if the map says that is possible."""
        return action if action in self.map.neighbors[state] else state
    
    def action_cost(self, s, action, s1):
        """The distance (cost) to go from s to s1."""
        return self.map.distances[s, s1]
    
    def h(self, node):
        "Straight-line distance between state and the goal."
        locs = self.map.locations
        return straight_line_distance(locs[node.state], locs[self.goal])
        
 class Map:
    """A map of places in a 2D world: a graph with vertexes and links between them. 
    In `Map(links, locations)`, `links` can be either [(v1, v2)...] pairs, 
    or a {(v1, v2): distance...} dict. Optional `locations` can be {v1: (x, y)} 
    If `directed=False` then for every (v1, v2) link, we add a (v2, v1) link."""

    def __init__(self, links, locations=None, directed=False):
        if not hasattr(links, 'items'): # Distances are 1 by default
            links = {link: 1 for link in links}
        if not directed:
            for (v1, v2) in list(links):
                links[v2, v1] = links[v1, v2]
        self.distances = links
        self.neighbors = multimap(links)
        self.locations = locations or defaultdict(lambda: (0, 0))

        
def multimap(pairs) -> dict:
    "Given (key, val) pairs, make a dict of {key: [val,...]}."
    result = defaultdict(list)
    for key, val in pairs:
        result[key].append(val)
    return result
    
poonamalle_nearby_locations = Map({('Sriperumbatur', 'saveetha hosiptal'):  5,('saveetha hospital', 'chembarambakkam'):3,
                                  ('chembarambakkam', 'poonamalle bridge'):  3,('poonamalle bridge', 'saveetha dental'):3,
                                  ('poonamalle bridge', 'poonamalle busstand'): 3,('saveetha dental', 'iyyanpanthangal'):  3,
                                  ('poonamalle busstand', 'iyyanpanthangal'):3,('iyyanthangal', 'ramachandra hospital'): 6,
                                  ('ramachandra hospital', 'guindy'):  6,
                                  ('guindy', 'porur'):  8,('porur', 't.nagar'): 8,('t.nagar', 'theynampet'): 4,
                                  ('theynampet', 'nandham'):  4,('nandham', 'santhom'):  4,('nandham', 'kathipara bridge'): 4,
                                  ('t.nagar', 'kathipara  bridge'):  6,('porur', 'alwarthirunagar'):  4,
                                  ('alwarthirunagar', 'kathipara bridge'): 5,('kathipara bridge', 'university of madras'):6,
                                  ('porur', 'koyembedu'):  7,('saveetha dental', 'madhuravol'): 6,
                                  ('madhuravol', 'forum mall'):  6,('forum mall', 'koyembedu'):  3,
                                  ('koyembedu', 'cresent art gallery'): 5,('cresent art gallery', 'egmore'):5,
                                  ('egmore', 'university of madras'): 6,('university of madras', 'marina beach'):6})

r0 = RouteProblem('Sriperumbatur', 'marina beach', map=poonamalle_nearby_locations)
r1 = RouteProblem('t.nagar', 'kathipara bridge', map=poonamalle_nearby_locations)
r2 = RouteProblem('university of madras', 'forum mall', map=poonamalle_nearby_locations)
r3 = RouteProblem('koyembedu', 'guindy', map=poonamalle_nearby_locations)
r4 = RouteProblem('theynampet', 'nandham', map=poonamalle_nearby_locations)
print(r1)
print(r2)
print(r3)
print(r4)

goal_state_path=breadth_first_search(r4)
path_states(goal_state_path) 
print("GoalStateWithPath:{0}".format(goal_state_path))
print("Total Distance={0} Kilometers".format(goal_state_path.path_cost))

```

## OUTPUT:
![map2](https://user-images.githubusercontent.com/75235209/166288377-53f4958f-0abd-4315-bfb3-af05f8c03871.PNG)


## SOLUTION JUSTIFICATION:
Route follow the minimum distance between locations using breadth-first search.
</br>
</br>
</br>

## RESULT:
Thus an algorithm to find the route from the source to the destination point using breadth-first search is developed and executed successfully.
