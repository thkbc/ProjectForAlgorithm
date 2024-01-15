import heapq
import random

def read_data(file_path):
    matrix_a = []   
    matrix_b = []
    matrix_c = []
    matrix_d = []

    try:
        with open(file_path, 'r') as file:
            current_matrix = matrix_a

            for line in file:
                line = line.strip()

                if not line:
                    current_matrix = matrix_a
                    continue

                if line.startswith("Adjacency:"):
                    current_matrix = matrix_a
                elif line.startswith("Bandwidth:"):
                    current_matrix = matrix_b
                elif line.startswith("Delay:"):
                    current_matrix = matrix_c
                elif line.startswith("Reliability:"):
                    current_matrix = matrix_d
                else:
                    values = list(map(float, line.split(':')))
                    if all(val.is_integer() for val in values):
                        values = list(map(int, values))
                    current_matrix.append(values)

    except FileNotFoundError:
        print(f"File not found: {file_path}")

    return matrix_a, matrix_b, matrix_c, matrix_d

def read_req(file_path):
    reqs = []

    try:
        with open(file_path, 'r') as file:
            for line in file:
                values = list(map(float, line.strip().split()))
                if all(val.is_integer() for val in values):
                    values = list(map(int, values))

                if len(values) < 3:
                    print(f"Ignoring invalid line: {line}")
                    continue

                src, dest, bw = map(int, values[:3])
                delay_threshold, reliability_threshold = 0.0, 0.0

                if len(values) > 3:
                    delay_threshold = values[3]

                if len(values) > 4:
                    reliability_threshold = values[4]

                reqs.append((src, dest, bw, delay_threshold, reliability_threshold))

    except FileNotFoundError:
        print(f"File not found: {file_path}")

    return reqs

def convert_to_adj_list(adj_matrix):
    adj_list = {}

    for i, weights in enumerate(adj_matrix):
        neighbors = {}
        for j, weight in enumerate(weights):
            if weight > 0:
                neighbors[j] = weight
        adj_list[i] = neighbors

    return adj_list

def dijkstra_algo(adj_list, start, bw_constraint):
    distances = {vertex: float('infinity') for vertex in adj_list}
    distances[start] = 0
    priority_queue = [(0, start)]

    paths = {vertex: [] for vertex in adj_list}
    
    while priority_queue:
        cur_dist, cur_vertex = heapq.heappop(priority_queue)

        if cur_dist > distances[cur_vertex]:
            continue

        for neighbor, weight in adj_list[cur_vertex].items():
            if weight <= bw_constraint:
                dist = cur_dist + weight

                if dist < distances[neighbor]:
                    distances[neighbor] = dist
                    heapq.heappush(priority_queue, (dist, neighbor))
                    paths[neighbor] = paths[cur_vertex] + [cur_vertex]

    return distances, paths

def bellman_ford_algo(adj_list, start, bw_constraint):
    distances = {vertex: float('infinity') for vertex in adj_list}
    distances[start] = 0

    for _ in range(len(adj_list) - 1):
        for cur_vertex, neighbors in adj_list.items():
            for neighbor, weight in neighbors.items():
                if weight <= bw_constraint:
                    dist = distances[cur_vertex] + weight
                    if dist < distances[neighbor]:
                        distances[neighbor] = dist

    paths = {vertex: [] for vertex in adj_list}
    return distances, paths

def a_star_algo(adj_list, start, goal, bw_constraint, heuristic=None):
    distances = {vertex: float('infinity') for vertex in adj_list}
    distances[start] = 0
    priority_queue = [(0, start)]

    paths = {vertex: [] for vertex in adj_list}

    while priority_queue:
        cur_dist, cur_vertex = heapq.heappop(priority_queue)

        if cur_vertex == goal:
            break

        for neighbor, weight in adj_list[cur_vertex].items():
            if weight <= bw_constraint:
                dist = distances[cur_vertex] + weight

                if dist < distances[neighbor]:
                    distances[neighbor] = dist
                    if heuristic:
                        priority = dist + heuristic(neighbor, goal)
                    else:
                        priority = dist
                    heapq.heappush(priority_queue, (priority, neighbor))
                    paths[neighbor] = paths[cur_vertex] + [cur_vertex]

    return distances, paths

def custom_heuristic(current, goal):
    return 0  # A trivial heuristic for now

def handle_req(adj_list, delay_matrix, reliability_matrix, req):
    src, dest, bw_constraint, delay_threshold, reliability_threshold = req
    distances, paths = a_star_algo(adj_list, src, dest, bw_constraint, custom_heuristic)

    valid_paths = []

    for dest, path in paths.items():
        if dest == dest:
            for node in path:
                delay = delay_matrix[node][dest]
                reliability = reliability_matrix[node][dest]

                if delay <= delay_threshold and reliability >= reliability_threshold:
                    valid_paths.append(path)
                    break

    return valid_paths

def choose_algo(algo_name):
    if algo_name == "dijkstra":
        return dijkstra_algo
    elif algo_name == "bellman_ford":
        return bellman_ford_algo
    elif algo_name == "a_star":
        return a_star_algo

def simple_schedule_algo(reqs):
    return reqs

def main():
    req_file_path = "requirements.txt"

    matrix_a, matrix_b, matrix_c, matrix_d = read_data("matrices.txt")
    adj_list = convert_to_adj_list(matrix_a)

    reqs = read_req(req_file_path)

    algo_names = ["dijkstra", "bellman_ford", "a_star"]

    for algo_name in algo_names:
        algo = choose_algo(algo_name)

        if algo_name == "a_star":
            schedule = algo(adj_list, 0, 5, 39, custom_heuristic)
        else:
            schedule = algo(adj_list, 0, 5)

        print(f"{algo_name.capitalize()} Algorithm Results:")
        dists, paths = schedule
        print("Distances:", dists)
        print("Paths:", paths)

    final_result = simple_schedule_algo(reqs)

    print("Final Result:")
    for entry in final_result:
        print(entry)

if __name__ == "__main__":
    main()
