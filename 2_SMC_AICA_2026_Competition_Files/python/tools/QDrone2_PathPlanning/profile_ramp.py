import numpy as np


def profile_ramp(wp, linear_vel, angular_vel):
    """
    Compute cumulative waypoint times based on linear and angular speed limits.

    Parameters
    ----------
    wp : np.ndarray, shape (N, 4)
        Waypoints as [x, y, z, yaw]
    linear_vel : float
        Maximum linear velocity
    angular_vel : float
        Maximum angular velocity

    Returns
    -------
    t : np.ndarray, shape (N,)
        Cumulative time vector
    """
    wp = np.asarray(wp, dtype=float)
    N = wp.shape[0]

    t = np.zeros(N)

    for k in range(1, N):
        # Linear distance
        dt_lin = np.linalg.norm(wp[k, 0:3] - wp[k - 1, 0:3]) / linear_vel

        # Angular distance (yaw)
        dt_ang = abs(wp[k, 3] - wp[k - 1, 3]) / angular_vel

        # Segment time must satisfy both
        dt = max(dt_lin, dt_ang)

        # Cumulative time
        t[k] = t[k - 1] + dt

    return t