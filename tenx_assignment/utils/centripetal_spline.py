# centripetal_spline.py
import numpy as np

def centripetal_catmull_rom(pts, num_samples=300, alpha=0.5, samples_per_segment=50):
    """
    Compute a centripetal Catmull-Rom spline through the input waypoints.

    Args:
        pts: sequence of (x,y) waypoints (Nx2)
        num_samples: total number of output samples along the whole curve (resampled uniformly)
        alpha: centripetal parameter (0.5 recommended)
        samples_per_segment: internal sampling per segment (used before final resampling)

    Returns:
        Nx2 numpy array of sampled (x,y) points (length == num_samples)
    """
    P = np.asarray(pts, dtype=float)
    if P.ndim != 2 or P.shape[1] != 2:
        raise ValueError("pts must be an Nx2 array-like")

    n = P.shape[0]
    if n == 0:
        return np.zeros((0,2))
    if n == 1:
        return P.copy()
    if n == 2:
        # straight line
        t = np.linspace(0.0, 1.0, num_samples)
        x = np.interp(t, [0.0, 1.0], [P[0,0], P[1,0]])
        y = np.interp(t, [0.0, 1.0], [P[0,1], P[1,1]])
        return np.column_stack((x,y))

    def tj(ti, pi, pj):
        return ti + np.linalg.norm(pj - pi)**alpha

    curve_pts = []

    # pad endpoints so first/last waypoint are interpolated
    pts_pad = np.vstack([P[0], P, P[-1]])

    # For each segment between P[i] and P[i+1] (i from 0..n-2), we compute local CR
    for i in range(1, len(pts_pad)-2):
        P0 = pts_pad[i-1]
        P1 = pts_pad[i]
        P2 = pts_pad[i+1]
        P3 = pts_pad[i+2]

        t0 = 0.0
        t1 = tj(t0, P0, P1)
        t2 = tj(t1, P1, P2)
        t3 = tj(t2, P2, P3)

        # sample between t1 and t2
        for t in np.linspace(t1, t2, samples_per_segment, endpoint=False):
            # Local centripetal Catmull-Rom interpolation
            # Compute intermediate points A1, A2, A3
            if (t1 - t0) == 0 or (t2 - t1) == 0 or (t3 - t2) == 0:
                # degeneracy guard: fallback to linear interp between P1 and P2
                tau = (t - t1) / max(1e-9, (t2 - t1))
                C = (1.0 - tau) * P1 + tau * P2
                curve_pts.append(C)
                continue

            A1 = ((t1 - t)/(t1 - t0)) * P0 + ((t - t0)/(t1 - t0)) * P1
            A2 = ((t2 - t)/(t2 - t1)) * P1 + ((t - t1)/(t2 - t1)) * P2
            A3 = ((t3 - t)/(t3 - t2)) * P2 + ((t - t2)/(t3 - t2)) * P3

            B1 = ((t2 - t)/(t2 - t0)) * A1 + ((t - t0)/(t2 - t0)) * A2
            B2 = ((t3 - t)/(t3 - t1)) * A2 + ((t - t1)/(t3 - t1)) * A3

            C = ((t2 - t)/(t2 - t1)) * B1 + ((t - t1)/(t2 - t1)) * B2
            curve_pts.append(C)

    # Also append the final last waypoint to ensure curve reaches the last point
    curve_pts.append(P[-1].copy())
    curve = np.asarray(curve_pts)

    # If curve is too short (edge cases), fallback to linear interpolation
    if curve.shape[0] < 2:
        t = np.linspace(0.0, 1.0, num_samples)
        x = np.interp(t, [0.0, 1.0], [P[0,0], P[-1,0]])
        y = np.interp(t, [0.0, 1.0], [P[0,1], P[-1,1]])
        return np.column_stack((x,y))

    # Resample uniformly along arc length to get exactly num_samples points
    seg_dists = np.linalg.norm(np.diff(curve, axis=0), axis=1)
    s = np.concatenate(([0.0], np.cumsum(seg_dists)))
    if s[-1] <= 0:
        return np.tile(curve[0], (num_samples,1))

    s_uniform = np.linspace(0.0, s[-1], num_samples)
    x_uniform = np.interp(s_uniform, s, curve[:,0])
    y_uniform = np.interp(s_uniform, s, curve[:,1])
    return np.column_stack((x_uniform, y_uniform))
