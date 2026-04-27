import numpy as np


def normalize_path(path):
    """
    Convert a loaded path entry into a 2D numpy array of shape (N, 2).

    Handles:
    - None
    - empty arrays/lists
    - single point stored as shape (2,)
    - flattened coordinates stored as shape (2N,)
    - already-correct shape (N, 2)
    """
    if path is None:
        return None

    path = np.array(path, dtype=float)

    if path.size == 0:
        return None

    if path.ndim == 1:
        if path.size == 2:
            path = path.reshape(1, 2)
        elif path.size % 2 == 0:
            path = path.reshape(-1, 2)
        else:
            raise ValueError(f"Invalid 1D path with odd number of elements: shape={path.shape}")

    elif path.ndim == 2:
        if path.shape[1] == 2:
            pass
        elif path.shape[0] == 2:
            path = path.T
        else:
            raise ValueError(f"Invalid 2D path shape: {path.shape}")

    else:
        raise ValueError(f"Unsupported path dimensions: ndim={path.ndim}, shape={path.shape}")

    return path


# -------------------------------------------------------------------------
# Load qcar2_paths
# -------------------------------------------------------------------------
qcar2_paths = np.load(
    r"tools\QCar2_PathPlanning\qcar2_paths.npy",
    allow_pickle=True
)

N = qcar2_paths.shape[0]

# Preallocate output like MATLAB cell(N)
qcar2_pathposes = np.empty((N, N), dtype=object)

# -------------------------------------------------------------------------
# Compute [x, y, yawRad] for each path
# -------------------------------------------------------------------------
for i in range(N):
    for j in range(N):
        path_raw = qcar2_paths[i, j]
        path = normalize_path(path_raw)

        if path is None:
            qcar2_pathposes[i, j] = None
            continue

        if path.shape[0] > 1:
            dx = np.diff(path[:, 0])
            dy = np.diff(path[:, 1])

            yawRad = np.arctan2(dy, dx)
            yawRad = np.append(yawRad, yawRad[-1])  # match waypoint length
            yawRad = np.unwrap(yawRad)              # avoid jumps near ±pi

            qcar2_pathposes[i, j] = np.column_stack((path, yawRad))
        else:
            qcar2_pathposes[i, j] = np.column_stack((path, np.array([0.0])))

# -------------------------------------------------------------------------
# Find maximum number of rows
# -------------------------------------------------------------------------
maxRows = 0

for i in range(N):
    for j in range(N):
        thisPath = normalize_path(qcar2_paths[i, j])
        if thisPath is not None:
            maxRows = max(maxRows, thisPath.shape[0])

# -------------------------------------------------------------------------
# Pad all paths to have same number of rows by repeating last row
# Empty paths become NaN(maxRows, 3)
# -------------------------------------------------------------------------
for i in range(N):
    for j in range(N):
        thisPath = qcar2_pathposes[i, j]

        if thisPath is None:
            qcar2_pathposes[i, j] = np.full((maxRows, 3), np.nan)
            continue

        thisPath = np.array(thisPath, dtype=float)
        numRows = thisPath.shape[0]

        if numRows < maxRows:
            lastRow = thisPath[-1, :]
            padRows = np.tile(lastRow, (maxRows - numRows, 1))
            thisPath = np.vstack((thisPath, padRows))

        qcar2_pathposes[i, j] = thisPath

# -------------------------------------------------------------------------
# Save result
# -------------------------------------------------------------------------
np.save(
    r"tools\QCar2_PathPlanning\qcar2_pathposes.npy",
    qcar2_pathposes,
    allow_pickle=True
)

print("Saved to tools\\QCar2_PathPlanning\\qcar2_pathposes.npy")
print("Output shape:", qcar2_pathposes.shape)
print("Each entry shape:", qcar2_pathposes[0, 0].shape if qcar2_pathposes[0, 0] is not None else None)