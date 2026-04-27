import os
import numpy as np

script_dir = os.path.dirname(os.path.abspath(__file__))
input_file = os.path.join(script_dir, "occupancy_grid.txt")
output_file = os.path.join(script_dir, "city_voxel_map.npz")

data = np.loadtxt(input_file, delimiter=",")

node_id = data[:, 0]
node_loc = data[:, 1:4]
occupied = data[:, 4]

N = len(node_id)
adjacency = np.zeros((N, N))

size_val = 3

for i in range(N):
    for j in range(i + 1, N):
        dist = np.linalg.norm(node_loc[i] - node_loc[j])
        if dist < (2 * size_val - 0.01):
            risk = 1000 * (occupied[i] + occupied[j]) / 2
            weight = risk * dist + dist
            adjacency[i, j] = weight
            adjacency[j, i] = weight

np.savez_compressed(
    output_file,
    adjacency=adjacency,
    node_id=node_id,
    node_loc=node_loc,
    occupied=occupied
)

print(f"Saved: {output_file}")
print(f"adjacency shape: {adjacency.shape}")
print(f"file size: {os.path.getsize(output_file) / 1024**2:.2f} MB")