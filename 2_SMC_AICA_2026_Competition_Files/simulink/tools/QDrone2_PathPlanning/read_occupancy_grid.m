clear; clc;
data = readmatrix('occupancy_grid.txt');

node_id       = data(:,1);
node_loc      = data(:,2:4);
occupied = data(:,5);

N = length(node_id);
adjacency = zeros(N, N);

size = 3;


for i = 1:N
    for j = (i+1):N
        dist = norm(node_loc(i, :) - node_loc(j, :), 2);
        if dist < (2*size - 0.01)
            risk = 1000*(occupied(i) + occupied(j))/2;
            adjacency(i, j) = risk*dist + dist;
            adjacency(j, i) = risk*dist + dist;
        end
    end
end

clearvars -except adjacency node_id node_loc occupied
save city_voxel_map