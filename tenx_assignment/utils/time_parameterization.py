# time_parameterization.py
"""
Simple, human-friendly trajectory time-parameterization utilities.

Inputs:
- `samples`: ndarray (N,2) spatial samples along the path (uniformly spaced in arc length is ideal)
Outputs:
- times: ndarray (N,) timestamps (seconds)
- trajectory: list of tuples [(x,y,t), ...] ready to publish

Functions:
- time_parameterize_constant(samples, v)
- time_parameterize_trapezoidal(samples, v_max, a_max)
- compute_discrete_curvature(samples)
- compute_curvature_limited_speed(samples, v_max, a_lat_max, v_min)
- integrate_variable_speed_to_times(samples, v_profile)
- make_time_stamped_trajectory(samples, times)
"""

import numpy as np


def arc_lengths(samples):
    """Return cumulative arc lengths s for a sequence of points (N,2)."""
    diffs = np.diff(samples, axis=0)
    seg = np.linalg.norm(diffs, axis=1)
    s = np.concatenate(([0.0], np.cumsum(seg)))
    return s


def time_parameterize_constant(samples, v=0.2):
    """
    Constant-speed parameterization.
    t = s / v where s is cumulative arc-length.
    """
    if v <= 0:
        raise ValueError("v must be > 0")
    s = arc_lengths(samples)
    times = s / v
    return times


def time_parameterize_trapezoidal(samples, v_max=0.3, a_max=0.5):
    """
    Trapezoidal time parameterization based on total path length.
    Uses a single global trapezoid: accel -> cruise -> decel.
    Produces monotonic times for each sample point according to their arc-length.
    """
    if v_max <= 0 or a_max <= 0:
        raise ValueError("v_max and a_max must be > 0")

    s = arc_lengths(samples)
    S = s[-1]
    if S <= 0:
        return np.zeros(len(samples), dtype=float)

    t_acc = v_max / a_max
    s_acc = 0.5 * a_max * t_acc ** 2

    if 2 * s_acc >= S:
        # triangular profile (no cruise)
        t_acc = np.sqrt(S / a_max)
        t_total = 2 * t_acc

        def time_of_s(si):
            if si <= S / 2.0:
                return np.sqrt(2.0 * si / a_max)
            else:
                return t_total - np.sqrt(2.0 * (S - si) / a_max)
    else:
        s_cruise = S - 2.0 * s_acc
        t_cruise = s_cruise / v_max
        t_total = 2.0 * t_acc + t_cruise

        def time_of_s(si):
            if si < s_acc:
                return np.sqrt(2.0 * si / a_max)
            elif si <= s_acc + s_cruise:
                return t_acc + (si - s_acc) / v_max
            else:
                # deceleration phase
                return t_acc + t_cruise + (t_acc - np.sqrt(2.0 * (S - si) / a_max))

    times = np.array([time_of_s(si) for si in s])
    return times


def make_time_stamped_trajectory(samples, times):
    """
    Convert samples (N,2) and times (N,) into a list of tuples [(x,y,t), ...].
    """
    if len(samples) != len(times):
        raise ValueError("samples and times must have the same length")
    return [(float(x), float(y), float(t)) for (x, y), t in zip(samples, times)]


# ---------- curvature utilities (optional) ----------
def compute_discrete_curvature(samples):
    """
    Compute a simple discrete curvature estimate at each point using three-point formula.
    Returns kappa array (N,) with endpoints approximated by their neighbor.
    Formula: kappa = 4 * area(triangle) / (a * b * c)
    where a,b,c are side lengths. For nearly collinear points this yields small curvature.
    """
    pts = np.asarray(samples, dtype=float)
    N = len(pts)
    if N < 3:
        return np.zeros(N)

    kappa = np.zeros(N)
    # interior points
    for i in range(1, N - 1):
        p0 = pts[i - 1]
        p1 = pts[i]
        p2 = pts[i + 1]
        a = np.linalg.norm(p1 - p0)
        b = np.linalg.norm(p2 - p1)
        c = np.linalg.norm(p2 - p0)
        # triangle area via cross product
        area = abs(np.cross(p1 - p0, p2 - p0)) / 2.0
        denom = a * b * c
        if denom > 1e-12:
            kappa[i] = 4.0 * area / denom
        else:
            kappa[i] = 0.0
    # endpoints: copy neighbor curvature
    kappa[0] = kappa[1]
    kappa[-1] = kappa[-2]
    return kappa


def compute_curvature_limited_speed(samples, v_max=0.3, a_lat_max=0.5, v_min=0.05):
    """
    Given samples (N,2), compute a per-point max speed limited by lateral acceleration:
        v <= sqrt(a_lat_max / |kappa|)
    where kappa is curvature. Combine with global v_max and v_min.

    Returns:
        v_profile: (N,) allowed max speeds per point
    Notes:
        - For near-zero curvature, speed returned is v_max.
        - v_min enforced to avoid zero velocities.
    """
    kappa = compute_discrete_curvature(samples)
    v_profile = np.full_like(kappa, v_max, dtype=float)
    eps = 1e-6
    for i, k in enumerate(kappa):
        if abs(k) > eps:
            v_lim = np.sqrt(max(0.0, a_lat_max / (abs(k) + 1e-12)))
            v_profile[i] = min(v_profile[i], v_lim)
    # enforce global bounds
    v_profile = np.clip(v_profile, v_min, v_max)
    return v_profile


def integrate_variable_speed_to_times(samples, v_profile):
    """
    Given samples (N,2) and a per-point speed v_profile (N,),
    compute timestamps by integrating with the trapezoidal rule between consecutive points:
       dt_i = 2 * ds / (v_i + v_{i+1})
    This gives times per sample point.
    """
    pts = np.asarray(samples, dtype=float)
    v = np.asarray(v_profile, dtype=float)
    N = len(pts)
    if N == 0:
        return np.array([])
    if N == 1:
        return np.array([0.0])
    seg_ds = np.linalg.norm(np.diff(pts, axis=0), axis=1)  # (N-1,)
    times = np.zeros(N, dtype=float)
    for i in range(0, N - 1):
        vi = max(1e-6, v[i])
        vj = max(1e-6, v[i + 1])
        ds = seg_ds[i]
        dt = 2.0 * ds / (vi + vj)  # trapezoidal integration of 1/v over ds
        times[i + 1] = times[i] + dt
    return times


# ----------------- Example usage (no plotting) -----------------
if __name__ == "__main__":
    # create a simple straight+curve example
    import numpy as np

    samples = np.array([
        [0.0, 0.0],
        [0.5, 0.1],
        [1.0, 0.05],
        [1.6, 0.4],
        [2.2, 1.0],
        [3.0, 1.2],
        [4.0, 1.0],
    ])

    # 1) constant speed
    times_const = time_parameterize_constant(samples, v=0.3)
    traj_const = make_time_stamped_trajectory(samples, times_const)
    print("First 3 points (constant v):", traj_const[:3])

    # 2) trapezoidal global profile
    times_trap = time_parameterize_trapezoidal(samples, v_max=0.4, a_max=0.6)
    traj_trap = make_time_stamped_trajectory(samples, times_trap)
    print("Last point time (trapezoidal):", traj_trap[-1][2])

    # 3) curvature-limited speeds + integrate variable speed
    v_profile = compute_curvature_limited_speed(samples, v_max=0.6, a_lat_max=0.8, v_min=0.05)
    times_var = integrate_variable_speed_to_times(samples, v_profile)
    traj_var = make_time_stamped_trajectory(samples, times_var)
    print("Times with curvature limit (sample):", times_var)
