import numpy as np
import matplotlib.pyplot as plt

# Implement a simple centripetal Catmull–Rom spline
def centripetal_catmull_rom(points, num_samples=300, alpha=0.5):
    points = np.array(points, dtype=float)
    n = len(points)

    # Compute parameter t
    def tj(ti, pi, pj):
        return ti + (np.linalg.norm(pj - pi) ** alpha)

    # Build full spline with padding for endpoints
    P = [points[0], *points, points[-1]]

    curve = []
    for i in range(1, len(P) - 2):
        p0, p1, p2, p3 = P[i - 1], P[i], P[i + 1], P[i + 2]

        t0 = 0
        t1 = tj(t0, p0, p1)
        t2 = tj(t1, p1, p2)
        t3 = tj(t2, p2, p3)

        t = np.linspace(t1, t2, num_samples // (n - 1))

        # Interpolation
        A1 = (t1 - t)[:, None]/(t1 - t0)*p0 + (t - t0)[:, None]/(t1 - t0)*p1
        A2 = (t2 - t)[:, None]/(t2 - t1)*p1 + (t - t1)[:, None]/(t2 - t1)*p2
        A3 = (t3 - t)[:, None]/(t3 - t2)*p2 + (t - t2)[:, None]/(t3 - t2)*p3

        B1 = (t2 - t)[:, None]/(t2 - t0)*A1 + (t - t0)[:, None]/(t2 - t0)*A2
        B2 = (t3 - t)[:, None]/(t3 - t1)*A2 + (t - t1)[:, None]/(t3 - t1)*A3

        C  = (t2 - t)[:, None]/(t2 - t1)*B1 + (t - t1)[:, None]/(t2 - t1)*B2
        curve.append(C)

    return np.vstack(curve)


# Time parameterization (simple constant speed)
def generate_time_parameterized_trajectory(path, speed=0.2):
    d = np.sqrt(np.sum(np.diff(path, axis=0)**2, axis=1))
    t = np.insert(np.cumsum(d / speed), 0, 0)
    traj = np.column_stack((path, t))
    return traj, traj


# ------------------ Run the test ------------------
waypoints = [
    (0.0, 0.0),
    (-2.45, 1.54),
    (-3.7, 4.02),
    (-7.0, 0.0),
    (-7.3, 13.7)
]

smooth = centripetal_catmull_rom(waypoints, num_samples=300, alpha=0.5)
traj, arr = generate_time_parameterized_trajectory(smooth, speed=0.2)

# Plot
plt.figure(figsize=(6,6))
plt.plot(arr[:,0], arr[:,1])   # no color specified
plt.scatter(*zip(*waypoints))  # mark original waypoints
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Smoothed Trajectory Path")
plt.axis("equal")
plt.grid(True)

plt.show()
