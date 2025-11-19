#!/usr/bin/env python3
"""
test_point_by_point.py

- Builds a centripetal Catmull-Rom smoothed curve from waypoints.
- Shows a matplotlib plot with waypoints and the smooth curve.
- Lets you step through the smooth curve point-by-point in the terminal:
    - press Enter to advance 1 point
    - type 'a' + Enter to autoplay (advance with small delay)
    - type 'q' + Enter to quit
"""

import argparse
import time
import sys
import numpy as np
import matplotlib.pyplot as plt
from centripetal_spline import centripetal_catmull_rom

def interactive_step_through(points, autoplay=False, delay=0.05):
    """
    points: Nx2 array of points
    autoplay: if True, advance automatically with delay
    delay: seconds between points in autoplay
    """
    n = len(points)
    idx = 0
    print("\n--- POINT-BY-POINT MODE ---")
    print("Commands: [Enter] next, 'a' + Enter autoplay, 'q' + Enter quit\n")

    if autoplay:
        while idx < n:
            x, y = points[idx]
            print(f"[{idx+1}/{n}] x={x:.6f}, y={y:.6f}")
            idx += 1
            time.sleep(delay)
        print("Reached end.")
        return

    while idx < n:
        inp = input(f"[{idx+1}/{n}] x={points[idx,0]:.6f}, y={points[idx,1]:.6f}  -> (Enter/a/q) ")
        if inp.strip().lower() == 'q':
            print("Quit.")
            return
        if inp.strip().lower() == 'a':
            # autoplay mode
            autoplay = True
            idx += 1
            while idx < n:
                print(f"[{idx+1}/{n}] x={points[idx,0]:.6f}, y={points[idx,1]:.6f}")
                idx += 1
                time.sleep(delay)
            print("Reached end of curve.")
            return
        # default: step once
        idx += 1
    print("Reached end of curve.")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--method", choices=["catmull"], default="catmull",
                        help="smoothing method (only 'catmull' here)")
    parser.add_argument("--samples", type=int, default=300, help="total number of samples on curve")
    parser.add_argument("--autoplay", action="store_true", help="autoplay stepping")
    parser.add_argument("--delay", type=float, default=0.05, help="autoplay delay (s)")
    args = parser.parse_args()

    # Example / your waypoints
    waypoints = [
    (0.0, 0.0),
    (-2.45, 1.54),
    (-3.7, 4.02),
    (-7.0, 0.0),
    (-7.3, 13.7)
]

    pts = np.array(waypoints)
    smooth = centripetal_catmull_rom(pts, num_samples=args.samples, alpha=0.5)

    print("First 6 smooth points:")
    for p in smooth[:6]:
        print(f"  ({p[0]:.6f}, {p[1]:.6f})")
    print("...")

    print("Last 6 smooth points:")
    for p in smooth[-6:]:
        print(f"  ({p[0]:.6f}, {p[1]:.6f})")
    print("")

    # Plot
    plt.figure(figsize=(10,4))
    plt.plot(pts[:,0], pts[:,1], 'ro-', label='Waypoints')
    plt.plot(smooth[:,0], smooth[:,1], 'b-', linewidth=2, label='Catmull-Rom Smooth')
    plt.scatter([pts[0,0]], [pts[0,1]], c='g', s=60, label='Start')
    plt.scatter([pts[-1,0]], [pts[-1,1]], c='m', s=60, label='Goal')
    plt.axis('equal')
    plt.legend()
    plt.title('Catmull-Rom Spline: Waypoints vs Smooth Curve')
    plt.grid(True)
    plt.show(block=False)

    # interactive stepping
    try:
        interactive_step_through(smooth, autoplay=args.autoplay, delay=args.delay)
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        plt.close()

if __name__ == "__main__":
    main()
