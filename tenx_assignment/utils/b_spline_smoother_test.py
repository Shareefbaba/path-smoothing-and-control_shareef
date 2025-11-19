# simple_bspline_smoother.py
"""
Simple B-spline path smoother (easy to explain in interviews).

Dependencies:
  - numpy
  - scipy (scipy.interpolate.splprep, splev)
  - (optional) matplotlib for plotting when run as __main__
"""

import numpy as np

try:
    from scipy.interpolate import splprep, splev
except Exception as e:
    raise ImportError("Install scipy: pip install scipy") from e


def bspline_smooth(waypoints, num_samples=200, smoothing=0.0, degree=3):
    """
    Fit a 2D B-spline to `waypoints` and return `num_samples` points sampled
    uniformly along the curve (approx. equal spacing in space).

    Args:
      waypoints: list or array of (x, y) pairs, length >= 2
      num_samples: how many points to return along the smooth curve
      smoothing: smoothing factor passed to scipy.splprep (0.0 = interpolate)
      degree: spline degree (3 = cubic is typical)

    Returns:
      samples: numpy array shape (num_samples, 2) containing (x, y) points
    """
    pts = np.asarray(waypoints, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 2:
        raise ValueError("waypoints must be shape (N, 2)")
    if pts.shape[0] < 2:
        raise ValueError("need at least 2 waypoints")

    # make sure degree is not larger than possible
    k = min(degree, pts.shape[0] - 1)

    # 1) Fit parametric B-spline to x(t), y(t)
    tck, u = splprep([pts[:, 0], pts[:, 1]], s=smoothing, k=k)

    # 2) Dense sampling in spline parameter space to approximate arc length
    dense = max(1000, num_samples * 6)
    u_dense = np.linspace(0.0, 1.0, dense)
    xy_dense = np.vstack(splev(u_dense, tck, der=0)).T  # (dense, 2)

    # 3) Compute cumulative distances along dense samples (arc-length)
    segs = np.linalg.norm(np.diff(xy_dense, axis=0), axis=1)
    s_dense = np.concatenate(([0.0], np.cumsum(segs)))
    total_length = s_dense[-1]
    if total_length <= 0:
        # degenerate case: all waypoints equal
        return np.repeat(xy_dense[:1], num_samples, axis=0)

    # 4) Create uniform arc-length targets and invert to parameter u
    s_targets = np.linspace(0.0, total_length, num_samples)
    u_samples = np.interp(s_targets, s_dense, u_dense)

    # 5) Evaluate spline at inverted parameters to get uniformly spaced points
    x_samples, y_samples = splev(u_samples, tck, der=0)
    samples = np.vstack((x_samples, y_samples)).T
    return samples


# --------------------------
# Minimal example / demo
# --------------------------
if __name__ == "__main__":
    import matplotlib.pyplot as plt

    waypoints = [
        (0.0, 0.0),
        (0.8, 0.2),
        (1.6, -0.1),
        (2.5, 0.6),
        (3.0, 1.2),
        (3.8, 1.1),
        (4.5, 0.4),
    ]

    smooth = bspline_smooth(waypoints, num_samples=300, smoothing=0.001)
    wp = np.array(waypoints)

    plt.figure(figsize=(7, 5))
    plt.plot(wp[:, 0], wp[:, 1], "o--", label="waypoints")
    plt.plot(smooth[:, 0], smooth[:, 1], "-", linewidth=2, label="B-spline (smoothed)")
    plt.axis("equal")
    plt.legend()
    plt.title("Simple B-spline Smoother")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.grid(True)
    plt.show()
