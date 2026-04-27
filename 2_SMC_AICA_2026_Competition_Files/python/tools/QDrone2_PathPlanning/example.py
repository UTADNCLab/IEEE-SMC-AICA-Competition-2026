# YOU CAN USE AS IS OR MODIFY THIS FILE FOR PATH PLANNING

import os
import numpy as np

# --- Import your previously defined functions ---
# Make sure these are in the same file or imported properly
from plan_path import plan_path   # adjust filename if needed
from profile_ramp import profile_ramp  # adjust filename if needed


if __name__ == "__main__":
    # --- Paths ---
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_file = os.path.join(script_dir, "city_voxel_map.npz")
    output_file = os.path.join(script_dir, "qdrone2_plans.npz")

    # --- Load voxel map ---
    data = np.load(data_file)
    adjacency = data["adjacency"]
    node_loc = data["node_loc"]
    node_id = data["node_id"]

    # --- Parameters ---
    tol = 0.1
    linear_velocity = 1.5
    yaw_velocity = 1.0

    start_loc = np.array([0.0, 0.0, 3.0])
    target1_loc = np.array([-2.50305, 29.6703, 3.0])

    # --- Plan path ---
    qdrone2_wp1 = plan_path(
        start_loc,
        target1_loc,
        adjacency,
        node_loc,
        node_id,
        tol
    )

    # --- Manual modification (force z = 3) ---
    qdrone2_wp1[:, 2] = 3.0

    # --- Time profile ---
    qdrone2_t1 = profile_ramp(
        qdrone2_wp1,
        linear_velocity,
        yaw_velocity
    )

    # --- Save results ---
    np.savez(
        output_file,
        qdrone2_wp1=qdrone2_wp1,
        qdrone2_t1=qdrone2_t1
    )

    print(qdrone2_wp1)
    print(qdrone2_t1)

    print("Saved:", output_file)
    print("Waypoints shape:", qdrone2_wp1.shape)
    print("Time vector shape:", qdrone2_t1.shape)