# time_parameterization_centripetal.py
"""
Time parameterization utilities.

Functions:
  generate_time_parameterized_trajectory(path_xy, speed=0.2, max_acc=0.5, profile='constant')
    -> returns (list_of_tuples, ndarray Nx3)
       where each tuple is (x, y, t) and ndarray columns are x,y,t

Supported profiles:
  - 'constant' : constant forward speed (simple)
  - 'trapezoidal' : accelerate up to speed with max_acc, cruise, then decelerate to stop at goal

Notes:
  - path_xy: Nx2 numpy array (x,y) or list of (x,y)
  - speed: desired cruising speed in m/s
  - max_acc: maximum acceleration used by trapezoidal profile (m/s^2)
"""

import numpy as np

def _compute_arc_lengths(path_xy):
    p = np.asarray(path_xy, dtype=float)
    if p.shape[0] == 0:
        return np.array([])
    d = np.zeros(p.shape[0], dtype=float)
    for i in range(1, p.shape[0]):
        dx = p[i,0] - p[i-1,0]
        dy = p[i,1] - p[i-1,1]
        d[i] = d[i-1] + np.hypot(dx, dy)
    return d

def generate_time_parameterized_trajectory(path_xy, speed=0.2, max_acc=0.5, profile='constant'):
    """
    Create a time-parameterized trajectory for a given 2D path.

    Args:
      path_xy: Nx2 array-like of (x,y)
      speed: cruising speed (m/s)
      max_acc: max acceleration for trapezoidal profile (m/s^2)
      profile: "constant" or "trapezoidal"

    Returns:
      traj_list: list of (x, y, t)
      traj_array: Nx3 numpy array [[x,y,t], ...]
    """
    p = np.asarray(path_xy, dtype=float)
    n = p.shape[0]
    if n == 0:
        return [], np.zeros((0,3))
    if n == 1:
        return [(float(p[0,0]), float(p[0,1]), 0.0)], np.array([[p[0,0], p[0,1], 0.0]])

    # arc-length for each point
    s = _compute_arc_lengths(p)  # monotonic non-decreasing
    total_len = float(s[-1])

    if total_len <= 0.0:
        # degenerate: all points equal
        t_zero = np.zeros(n)
        traj = [(float(p[i,0]), float(p[i,1]), float(t_zero[i])) for i in range(n)]
        return traj, np.column_stack((p[:,0], p[:,1], t_zero))

    if profile == 'constant':
        times = s / float(max(1e-9, speed))
    elif profile == 'trapezoidal':
        # build trapezoidal speed profile along path length
        # compute distance needed to accelerate to speed: d_acc = 0.5 * v^2 / a
        v = float(speed)
        a = float(max_acc) if max_acc > 0 else 1e-6
        d_acc = 0.5 * v * v / a
        if 2 * d_acc >= total_len:
            # triangular profile: cannot reach cruising speed
            # peak speed vm such that total_len = 2 * (0.5 * vm^2 / a) => vm = sqrt(a * total_len)
            vm = np.sqrt(a * total_len)
            t_acc = vm / a
            # build times for each s_i by integrating piecewise formula
            times = np.zeros_like(s)
            for i, si in enumerate(s):
                if si <= 0.5 * total_len:
                    # in accel phase: s = 0.5 * a * t^2 => t = sqrt(2*s/a)
                    times[i] = np.sqrt(2.0 * si / a)
                else:
                    # decel phase: symmetric
                    sd = total_len - si
                    times[i] = (2.0 * (vm / a)) - np.sqrt(2.0 * sd / a)
        else:
            # full trapezoid: accel, cruise, decel
            t_acc = v / a
            d_acc = 0.5 * a * t_acc * t_acc  # should equal 0.5*v^2/a
            d_cruise = total_len - 2.0 * d_acc
            t_cruise = d_cruise / v
            # compute time at arc s:
            times = np.zeros_like(s)
            for i, si in enumerate(s):
                if si < d_acc:
                    # accel: s = 0.5 * a * t^2 -> t = sqrt(2*s/a)
                    times[i] = np.sqrt(2.0 * si / a)
                elif si < (d_acc + d_cruise):
                    # cruise: time = t_acc + (si - d_acc)/v
                    times[i] = t_acc + (si - d_acc) / v
                else:
                    # decel: distance into decel phase:
                    sd = total_len - si
                    # time from end: t_end_dec = sqrt(2*sd/a)
                    times[i] = t_acc + t_cruise + (v / a) - np.sqrt(2.0 * sd / a)
    else:
        raise ValueError("Unknown profile: choose 'constant' or 'trapezoidal'")

    # build output
    traj_array = np.column_stack((p[:,0], p[:,1], times))
    traj_list = [(float(traj_array[i,0]), float(traj_array[i,1]), float(traj_array[i,2])) for i in range(traj_array.shape[0])]
    return traj_list, traj_array
