clear; clc;
load city_voxel_map.mat

tol = 0.1; % Tolerance distance to add start_loc and target_loc
linear_velocity = 1.5; % m/s
yaw_velocity = 1.0; % rad/s


start_loc = [0, 0, 3]; 
target1_loc = [-2.50305, 29.6703, 3];

qdrone2_wp1 = plan_path(start_loc, target1_loc, adjacency, node_loc, node_id, tol);

% Manual modification if needed
qdrone2_wp1(:, 3) = 3;

qdrone2_t1 = profile_ramp(qdrone2_wp1, linear_velocity, yaw_velocity);


clearvars -except qdrone2_wp1 qdrone2_t1;
save qdrone2_plans