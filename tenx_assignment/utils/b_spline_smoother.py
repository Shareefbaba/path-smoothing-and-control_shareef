# bspline_smoother.py
"""
B-spline smoother utility (no plotting).
Requires: numpy, scipy

Functions:
  - bspline_smooth(waypoints, num_samples, smoothing, degree)
      -> returns (pts) ndarray shape (num_samples,2)
  - arc_length(pts) -> cumulative arc-length array (useful externally)
"""

import numpy as np

try:
    from scipy.interpolate import splprep, splev
except Exception as e:
    raise ImportError("scipy is required: pip install scipy") from e


def arc_length(pts):
    """Return cumulative arc-length for a sequence of points (pts Nx2)."""
    diffs = np.diff(pts, axis=0)
    seg_len = np.linalg.norm(diffs, axis=1)
    s = np.concatenate(([0.0], np.cumsum(seg_len)))
    return s


def bspline_smooth(waypoints, num_samples=200, smoothing=0.0, degree=3):
    """
    Fit a 2D B-spline to `waypoints` and return `num_samples` points sampled
    approximately uniformly by arc length.

    Args:
      waypoints: list or (N,2) array of (x,y)
      num_samples: number of output samples along the smoothed curve
      smoothing: scipy.splprep smoothing factor (s=0 interpolates)
      degree: spline degree (k) (use 3 for cubic if possible)

    Returns:
      samples: ndarray (num_samples, 2)
    """
    pts_in = np.asarray(waypoints, dtype=float)
    if pts_in.ndim != 2 or pts_in.shape[1] != 2:
        raise ValueError("waypoints must be shape (N,2)")
    if pts_in.shape[0] < 2:
        raise ValueError("need at least 2 waypoints")

    k = min(degree, pts_in.shape[0] - 1)

    # Fit parametric spline to x(u), y(u)
    tck, _ = splprep([pts_in[:, 0], pts_in[:, 1]], s=smoothing, k=k)

    # Dense sampling in parameter space to approximate arc-length mapping u->s
    dense = max(1000, num_samples * 6)
    u_dense = np.linspace(0.0, 1.0, dense)
    xy_dense = np.vstack(splev(u_dense, tck, der=0)).T  # (dense,2)

    s_dense = arc_length(xy_dense)
    total_len = s_dense[-1]
    if total_len <= 0 or not np.isfinite(total_len):
        # degenerate: return repeated first point
        return np.repeat(xy_dense[:1], num_samples, axis=0)

    # target arc lengths (uniform)
    s_targets = np.linspace(0.0, total_len, num_samples)

    # invert: get parameter u for each target arc-length by linear interpolation
    u_samples = np.interp(s_targets, s_dense, u_dense)

    xs, ys = splev(u_samples, tck, der=0)
    samples = np.vstack((xs, ys)).T
    return samples
