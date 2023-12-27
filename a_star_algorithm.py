from math import sqrt

# List of all edges
all_edges = list(range(15))

# List of all end (unloading) nodes
end_nodes = [0,11,12,13,14]

# List of all directly connected paths
# All direct paths have a distance of 5
all_paths = [
    (0, 4),
    (4, 7),
    (7, 11),
    (7, 8),
    (8, 12),
    (8, 9),
    (9, 13),
    (9, 10),
    (10, 14),
    (9, 5),
    (5, 2),
    (10, 6),
    (6, 3),
    (2, 3),
    (1, 2),
    (0, 1)
]
# Add return versions of the paths
for (a,b) in all_paths.copy():
    all_paths.append((b,a))

# Costs of all paths
# All direct paths have a distance of 5
distance_cost = 5
all_path_costs = {
    x: distance_cost for x in all_paths
}

# Dictionary of nodes and their relative coordinates
all_coords = {
    0: (0, 15),
    1: (5, 15),
    2: (10, 15),
    3: (15, 15),
    4: (0, 10),
    5: (10, 10),
    6: (15, 10),
    7: (0, 5),
    8: (5, 5),
    9: (10, 5),
    10: (15, 5),
    11: (0, 0),
    12: (5, 0),
    13: (10, 0),
    14: (15, 0),
}

# Dijkstar Algorithm, used in A* as g()
def dijkstar(start_node, end_node):
    # Ensure proper formatting
    start_node = int(start_node)
    end_node = int(end_node)

    # Skip if the source and destination are the same
    if start_node == end_node:
        return 0

    # Initial vertex distances are set to infinity
    dijk_vertices = {
        x: 999 for x in all_edges if x != start_node
    }
    # Updating known direct path costs
    for (a,b) in all_paths:
        if a == start_node:
            dijk_vertices[b] = all_path_costs[(a,b)]

    # List that contains unchecked nodes
    not_checked = list(dijk_vertices.keys())
    # The loop will continue until all nodes have been checked
    while not_checked:
        # Selection of unchecked node with the lowest cost
        min_node = -1
        min_val = 9999
        for node in not_checked.copy():
            node_val = dijk_vertices[node]
            if node_val < min_val:
                min_val = node_val
                min_node = node
        # Updating vertex distances whenever a lower value is found
        for (a,b) in all_paths:
            if a == min_node and b != start_node:
                target_val = dijk_vertices[b]
                new_val = dijk_vertices[a] + all_path_costs[(a,b)]
                if new_val < target_val:
                    dijk_vertices[b] = new_val
        # Take off the unchecked list
        not_checked.remove(min_node)

    return dijk_vertices[end_node]

# Heuristic Algorithm, used in A* as h()
# A simple Euclidean distance calculator
def heuristic(start_node, end_node):
    # Ensure proper formatting
    start_node = int(start_node)
    end_node = int(end_node)

    # Skip if the source and destination are the same
    if start_node == end_node:
        return 0

    # Obtain the coordinates for the given nodes
    start_coord = all_coords[start_node]
    end_coord = all_coords[end_node]
    # Obtain Euclidian distances
    result = (start_coord[0] - end_coord[0]) ** 2 + \
             (start_coord[1] - end_coord[1]) ** 2
    result = sqrt(result)
    return result

# A* Algorithm
def a_star(start_node, end_node):
    # Ensure proper formatting
    start_node = int(start_node)
    end_node = int(end_node)

    # Signal to end search when target is reached
    target_reached = False
    # The A* path
    current_path = []
    # List of all checked nodes
    passed = [start_node]
    # Intermediate node checked through the loop
    latest_node = start_node
    while not target_reached:
        # Placeholder variable to find the minimum value
        min_score = 9999
        best_node = -1
        for (a,b) in all_paths:
            # Finding relevant paths that do not include checked nodes
            if a == latest_node and b not in passed:
                middle_node = b
                g = dijkstar(latest_node, middle_node)
                h = heuristic(middle_node, end_node)
                # The A* score
                f = g + h
                # Recording the node with the best A* score
                if f < min_score:
                    min_score = f
                    best_node = b
        # If stuck on a dead-end, return to previous node
        if best_node == -1:
            latest_node = passed[-2]
            current_path.pop(-1)
            continue
        # Add the new intermediate node to the path
        current_path.append((latest_node, best_node))
        passed.append(best_node)
        latest_node = best_node
        # If the target has been found, end the search
        if best_node == end_node:
            target_reached = True

    return current_path

# Testing the algorithm
a = 1
b = 14
optimal_path = a_star(a,b)
print(optimal_path)