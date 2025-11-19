# bspline_smoother_simple.py
import numpy as np

try:
    from scipy.interpolate import splprep, splev
except Exception as e:
    raise ImportError("scipy is required. Install with `pip install scipy`") from e


def bspline_smooth_and_sample(
    waypoints,
    degree=3,
    smoothing=0.0,
    num_samples=200,
    return_theta=False,
    return_curvature=False,
):
    """
    Fit a parametric B-spline to given 2D waypoints, re-parameterize by arc-length,
    and return uniformly-spaced samples along the smoothed curve.

    Args:
        waypoints: iterable of (x,y) pairs (list/ndarray) with length >= 2
        degree: spline degree (k). Typical: 3 (cubic). Must be <= (N-1).
        smoothing: smoothing factor `s` passed to scipy.interpolate.splprep.
                   s=0 => interpolating spline; larger s => smoother fit.
        num_samples: number of output points along the smoothed curve.
        return_theta: if True, also return heading angles (radians) per sample.
        return_curvature: if True, also return curvature per sample.

    Returns:
        pts: (num_samples, 2) numpy array of (x,y) positions.
        (optional) thetas: (num_samples,) array of headings (atan2(dy,dx))
        (optional) curvatures: (num_samples,) curvature values

    Notes:
        - This uses a dense parameter-space sampling to approximate the u->s (parameter->arc-length)
          mapping, then inverts that mapping to sample uniformly by arc-length.
        - scipy is required (scipy.interpolate.splprep & splev).
    """
    pts_in = np.asarray(waypoints, dtype=float)
    if pts_in.ndim != 2 or pts_in.shape[1] != 2:
        raise ValueError("waypoints must be shape (N,2)")

    n_pts = pts_in.shape[0]
    if n_pts < 2:
        raise ValueError("need at least 2 waypoints")

    # adjust degree if too large
    k = min(degree, n_pts - 1)

    # fit parametric spline; splprep returns tck and u (parameter values for input points)
    tck, u_in = splprep([pts_in[:, 0], pts_in[:, 1]], s=smoothing, k=k)

    # dense sampling in parameter space (to estimate arc length accurately)
    dense_samples = max(2000, num_samples * 8)
    u_dense = np.linspace(0.0, 1.0, dense_samples)
    xy_dense = np.vstack(splev(u_dense, tck, der=0)).T  # (M,2)

    # approximate arc-length along dense samples
    diffs = np.diff(xy_dense, axis=0)
    seg_lengths = np.linalg.norm(diffs, axis=1)
    s_dense = np.concatenate(([0.0], np.cumsum(seg_lengths)))
    total_length = s_dense[-1]

    if total_length <= 0 or not np.isfinite(total_length):
        # degenerate: all points coincide
        pts_out = np.repeat(xy_dense[:1], num_samples, axis=0)
        outputs = (pts_out,)
        if return_theta:
            outputs += (np.zeros(num_samples,),)
        if return_curvature:
            outputs += (np.zeros(num_samples,),)
        return outputs[0] if len(outputs) == 1 else outputs

    # target arc-length samples (uniform)
    s_target = np.linspace(0.0, total_length, num_samples)

    # invert s(u) with linear interpolation: u = interp(s_target, s_dense, u_dense)
    u_samples = np.interp(s_target, s_dense, u_dense)

    # evaluate spline and derivatives at u_samples
    out0 = splev(u_samples, tck, der=0)
    out1 = splev(u_samples, tck, der=1)
    out2 = splev(u_samples, tck, der=2)
    pts = np.vstack(out0).T         # (num_samples, 2)
    d1 = np.vstack(out1).T         # (num_samples, 2)
    d2 = np.vstack(out2).T         # (num_samples, 2)

    results = (pts,)

    if return_theta or return_curvature:
        # headings (theta)
        thetas = np.arctan2(d1[:, 1], d1[:, 0])
        if return_theta:
            results += (thetas,)

        if return_curvature:
            # curvature kappa = (x' y'' - y' x'') / (x'^2 + y'^2)^(3/2)
            numer = d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]
            denom = (d1[:, 0] ** 2 + d1[:, 1] ** 2) ** 1.5
            with np.errstate(divide="ignore", invalid="ignore"):
                kappa = np.where(denom > 1e-12, numer / denom, 0.0)
            results += (kappa,)

    # unpack: return a single array if only pts requested, else tuple
    if len(results) == 1:
        return results[0]
    else:
        return results


# -----------------------------
# Quick example / minimal test
# -----------------------------
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

    pts = bspline_smooth_and_sample(waypoints, degree=3, smoothing=0.001, num_samples=300)
    print(f"sampled_pts.shape = {pts.shape}")

    # Plot waypoints + smoothed curve
    wp = np.array(waypoints)
    plt.figure(figsize=(8, 6))
    plt.plot(wp[:, 0], wp[:, 1], "o--", label="waypoints")
    plt.plot(pts[:, 0], pts[:, 1], "-", linewidth=2, label="B-spline (smoothed sample)")
    plt.axis("equal")
    plt.legend()
    plt.title("B-spline Smoother (C^2) — Uniformly sampled by arc length")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.grid(True)
    plt.show()
