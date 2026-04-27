function [path] = plan_path(start_loc, goal_loc, adjacency, node_loc, node_id, tol)

    % --- Find closest node to start ---
    dist_start = vecnorm(node_loc - start_loc, 2, 2);   % Euclidean distance
    [~, idx] = min(dist_start);
    id_closest_to_start = node_id(idx);

    % --- Find closest node to goal ---
    dist_goal = vecnorm(node_loc - goal_loc, 2, 2);
    [~, idx] = min(dist_goal);
    id_closest_to_goal = node_id(idx);

    % --- Find shortest path in graph ---
    path_id  = dijkstra_adjacency(adjacency, id_closest_to_start, id_closest_to_goal);

    % --- Convert node IDs to path coordinates ---
    path = zeros(length(path_id), 4);
    for k = 1:length(path_id)
        id = path_id(k);
        path(k, :) = [node_loc(id, :), 0];
    end

    % --- Add start_loc at beginning if first point is far away ---
    if isempty(path) || norm(path(1,1:3) - start_loc) > tol
        path = [start_loc, 0; path];
    end

    % --- Add goal_loc at end if last point is far away ---
    if isempty(path) || norm(path(end,1:3) - goal_loc) > tol
        path = [path; goal_loc, 0];
    end

end

function [path, totalCost, dist, prev] = dijkstra_adjacency(adjMat, startNode, goalNode)
% DIJKSTRA_ADJACENCY
% Finds the shortest path in a graph using Dijkstra's algorithm.
%
% Inputs:
%   adjMat    - NxN adjacency/cost matrix
%               adjMat(i,j) > 0 means edge i->j with that cost
%               adjMat(i,j) <= 0 means no edge
%   startNode - scalar start node index
%   goalNode  - scalar goal node index
%
% Outputs:
%   path      - shortest path from startNode to goalNode
%   totalCost - total cost of the shortest path
%   dist      - shortest distance from startNode to every node
%   prev      - predecessor of each node in the shortest path tree

    n = size(adjMat, 1);

    if size(adjMat, 2) ~= n
        error('adjMat must be square.');
    end

    if startNode < 1 || startNode > n || goalNode < 1 || goalNode > n
        error('startNode and goalNode must be valid node indices.');
    end

    % Initialize
    dist = inf(n,1);
    prev = zeros(n,1);
    visited = false(n,1);

    dist(startNode) = 0;

    while true
        % Find unvisited node with smallest tentative distance
        unvisited = find(~visited);
        if isempty(unvisited)
            break;
        end

        [~, idx] = min(dist(unvisited));
        current = unvisited(idx);

        % If smallest distance is inf, remaining nodes are unreachable
        if isinf(dist(current))
            break;
        end

        % Stop early if goal reached
        if current == goalNode
            break;
        end

        visited(current) = true;

        % Neighbors of current node
        neighbors = find(adjMat(current,:) > 0);

        for k = 1:length(neighbors)
            neighbor = neighbors(k);

            if visited(neighbor)
                continue;
            end

            alt = dist(current) + adjMat(current, neighbor);

            if alt < dist(neighbor)
                dist(neighbor) = alt;
                prev(neighbor) = current;
            end
        end
    end

    % Reconstruct path
    if isinf(dist(goalNode))
        path = [];
        totalCost = inf;
    else
        path = reconstruct_path(prev, goalNode);
        totalCost = dist(goalNode);
    end
end

function path = reconstruct_path(prev, goalNode)
    path = goalNode;
    while prev(path(1)) ~= 0
        path = [prev(path(1)), path]; %#ok<AGROW>
    end
end