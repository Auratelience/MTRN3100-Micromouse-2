#!/usr/bin/env -S uv run --script
"""Success rate and cost for both planners, over a fixed set of trips.

    python bench.py [seeds] [iters]

The Dubins planner is the one that matters -- its output is drivable -- but the
straight-line planner is kept alongside as the optimistic bound: it ignores
curvature, so wherever it succeeds and the Dubins one does not, the gap is
either a corridor too tight to turn in or a search that needs more samples.
"""

import sys
import time

import cv2
import numpy as np

import maze_grid as mg
import maze_map as mp
import rrt_star as rs
import segments as sg

PAIRS = [((1, 1), (7, 7)), ((1, 4), (7, 2)), ((2, 0), (6, 8)),
         ((0, 4), (8, 4)), ((4, 4), (1, 8))]


def main():
    seeds = int(sys.argv[1]) if len(sys.argv) > 1 else 4
    iters = int(sys.argv[2]) if len(sys.argv) > 2 else 2500

    img = cv2.imread("mazes/1.png")
    res = mg.solve(img)
    M = mp.build_map(img, res)
    W = M["frame"]
    world = rs.MazeWorld(
        M["wall_segments_mm"], M["cylinders"], M["deck_mm"], robot_radius_mm=40,
        posts_mm=M["post_centres_mm"], post_radius_mm=M["post_radius_mm"],
    )
    print(f"{int(M['posts']['present'].sum())} posts as obstacles, "
          f"{len(M['isolated_posts'])} of them free-standing")
    print(f"{seeds} seeds x {iters} iters per trip\n")

    tot = {"dubins": [0, 0, 0.0], "polyline": [0, 0, 0.0]}
    for a, b in PAIRS:
        s, g = W.cell_centre_mm(*a), W.cell_centre_mm(*b)
        if not (world.is_free(s)[0] and world.is_free(g)[0]):
            print(f"  skip {a}->{b}, endpoint blocked")
            continue
        # Distinguish "walled off" from "too tight once inflated": the cell graph
        # knows only about panels, so a pair it calls reachable that neither
        # planner can solve is a clearance problem, not a topology one.
        linked = b in mp.reachable(M["cells"], a)
        th = rs.best_heading(world, s)
        row = {}
        for name in ("dubins", "polyline"):
            ok, costs, nseg, t0 = 0, [], [], time.time()
            for sd in range(seeds):
                if name == "dubins":
                    r = rs.plan_dubins(world, (s[0], s[1], th), (g[0], g[1], None),
                                       max_iter=iters, seed=sd)
                else:
                    r = rs.plan(world, s, g, max_iter=iters, seed=sd)
                if r["path"] is not None:
                    ok += 1
                    costs.append(r["cost"])
                    nseg.append(len(r["segments"]) if name == "dubins" else len(r["path"]) - 1)
            dt = (time.time() - t0) / seeds
            tot[name][0] += ok
            tot[name][1] += seeds
            tot[name][2] += dt * seeds
            row[name] = (ok, costs, nseg, dt)
        if not any(r[0] for r in row.values()):
            print(f"  {a}->{b} unsolved; wall graph says "
                  f"{'reachable -- clearance, not topology' if linked else 'walled off'}")
        for name, (ok, costs, nseg, dt) in row.items():
            c = f"{np.mean(costs):6.0f}+-{np.std(costs):3.0f} mm" if costs else "     -- "
            n = f"{np.mean(nseg):4.1f}" if nseg else "  --"
            print(f"  {a}->{b} {name:9s} {ok}/{seeds}  {c}  {n} segments  {dt:5.1f} s")
        print()

    for name, (ok, n, t) in tot.items():
        print(f"{name:9s} TOTAL {ok}/{n}   {t / max(n, 1):.1f} s/run")


if __name__ == "__main__":
    main()
