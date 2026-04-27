import os
import numpy as np

def reconstruct_path(prev, goal_node):
    path = [goal_node]
    while prev[path[0]] != -1:
        path.insert(0, prev[path[0]])
    return path


def dijkstra_adjacency(adj_mat, start_node, goal_node):
    """
    Finds the shortest path in a graph using Dijkstra's algorithm.

    Parameters
    ----------
    adj_mat : np.ndarray
        NxN adjacency/cost matrix.
        adj_mat[i, j] > 0 means edge i -> j with that cost.
        adj_mat[i, j] <= 0 means no edge.
    start_node : int
        Start node index (0-based).
    goal_node : int
        Goal node index (0-based).

    Returns
    -------
    path : list[int]
        Shortest path from start_node to goal_node.
    total_cost : float
        Total cost of the shortest path.
    dist : np.ndarray
        Shortest distance from start_node to every node.
    prev : np.ndarray
        Predecessor of each node in the shortest path tree.
    """
    n = adj_mat.shape[0]

    if adj_mat.shape[1] != n:
        raise ValueError("adj_mat must be square.")

    if start_node < 0 or start_node >= n or goal_node < 0 or goal_node >= n:
        raise ValueError("start_node and goal_node must be valid node indices.")

    dist = np.full(n, np.inf)
    prev = np.full(n, -1, dtype=int)
    visited = np.zeros(n, dtype=bool)

    dist[start_node] = 0.0

    while True:
        unvisited = np.where(~visited)[0]
        if len(unvisited) == 0:
            break

        current = unvisited[np.argmin(dist[unvisited])]

        if np.isinf(dist[current]):
            break

        if current == goal_node:
            break

        visited[current] = True

        neighbors = np.where(adj_mat[current, :] > 0)[0]

        for neighbor in neighbors:
            if visited[neighbor]:
                continue

            alt = dist[current] + adj_mat[current, neighbor]

            if alt < dist[neighbor]:
                dist[neighbor] = alt
                prev[neighbor] = current

    if np.isinf(dist[goal_node]):
        path = []
        total_cost = np.inf
    else:
        path = reconstruct_path(prev, goal_node)
        total_cost = dist[goal_node]

    return path, total_cost, dist, prev


def plan_path(start_loc, goal_loc, adjacency, node_loc, node_id, tol):
    """
    Parameters
    ----------
    start_loc : array-like, shape (3,)
    goal_loc : array-like, shape (3,)
    adjacency : np.ndarray, shape (N, N)
    node_loc : np.ndarray, shape (N, 3)
    node_id : np.ndarray, shape (N,)
    tol : float

    Returns
    -------
    path : np.ndarray, shape (M, 4)
        Path coordinates with a trailing zero in the 4th column.
    """
    start_loc = np.asarray(start_loc, dtype=float)
    goal_loc = np.asarray(goal_loc, dtype=float)

    # --- Find closest node to start ---
    dist_start = np.linalg.norm(node_loc - start_loc, axis=1)
    idx_start = np.argmin(dist_start)

    # --- Find closest node to goal ---
    dist_goal = np.linalg.norm(node_loc - goal_loc, axis=1)
    idx_goal = np.argmin(dist_goal)

    # IMPORTANT:
    # In MATLAB, node_id often starts at 1, but Python arrays use 0-based indexing.
    # Since adjacency/node_loc are indexed by row position, use idx_start/idx_goal directly.
    path_idx, total_cost, dist, prev = dijkstra_adjacency(
        adjacency, idx_start, idx_goal
    )

    # --- Convert node indices to path coordinates ---
    if len(path_idx) == 0:
        path = np.empty((0, 4))
    else:
        path = np.zeros((len(path_idx), 4))
        for k, idx in enumerate(path_idx):
            path[k, :] = np.hstack((node_loc[idx, :], 0.0))

    # --- Add start_loc at beginning if first point is far away ---
    if len(path) == 0 or np.linalg.norm(path[0, 0:3] - start_loc) > tol:
        path = np.vstack((np.hstack((start_loc, 0.0)), path))

    # --- Add goal_loc at end if last point is far away ---
    if len(path) == 0 or np.linalg.norm(path[-1, 0:3] - goal_loc) > tol:
        path = np.vstack((path, np.hstack((goal_loc, 0.0))))

    return path


if __name__ == "__main__":
    # Get script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Build full path to npz file
    file_path = os.path.join(script_dir, 'city_voxel_map.npz')

    # Load
    data = np.load(file_path)

    adjacency = data["adjacency"]
    node_id = data["node_id"]
    node_loc = data["node_loc"]
    occupied = data["occupied"]  # loaded in case you still want it elsewhere

    # Example start/goal
    start_loc = np.array([0, 0, 3])
    goal_loc = np.array([-2.50305, 29.6703, 3])
    tol = 0.5

    path = plan_path(start_loc, goal_loc, adjacency, node_loc, node_id, tol)

    print("Path:")
    print(path)