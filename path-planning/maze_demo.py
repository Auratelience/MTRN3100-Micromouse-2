#!/usr/bin/env -S uv run --script
"""End to end: photo -> lattice -> walls + posts + obstacles -> drivable path.

    python maze_demo.py [image] [--from i,j] [--to i,j] [--r 40]

--from / --to take cell indices and use the cell centre, (180i+90, 180j+90).
--from-mm / --to-mm take world millimetres directly.

The default planner is Dubins RRT*, whose output is a list of straights and
arcs -- the firmware's Segment alphabet -- printed at the end as
appendSegment() calls ready to paste into the sketch.  ``--mode polyline`` runs
the old straight-line RRT* instead, which is faster but produces corners no
robot can drive.
"""

import argparse
import json

import cv2
import numpy as np

import maze_grid as mg
import maze_map as mp
import rrt_star as rs
import segments as sg


def ascii_maze(walls, posts=None):
    """The classic +---+ dump.  A node with no post prints as ``.``, so a maze
    with gaps in the lattice is visible at a glance."""
    V, H = walls["vertical"], walls["horizontal"]
    nx, ny = H.shape[0], V.shape[1]

    def node(i, j):
        if posts is None:
            return "+"
        return "+" if posts["present"][i, j] else "."

    out = []
    for j in range(ny):
        out.append(
            "".join(node(i, j) + ("---" if V[i, j] else "   ") for i in range(nx - 1))
            + node(nx - 1, j)
        )
        if j < ny - 1:
            out.append(
                "".join(("|" if H[i, j] else " ") + "   " for i in range(nx - 1))
                + ("|" if H[nx - 1, j] else " ")
            )
    return "\n".join(out)


def start_arrow(vis, W, start_mm, theta0, length_mm=140.0):
    """The pose the plan was built from, drawn where the robot has to be put.

    Map coordinates are image convention (+x east, +y south), so this needs no
    sign gymnastics: the arrow comes out pointing the way ``--theta0`` means on
    the photo, right for 0 and down for +90.  Both ends go through ``mm_to_px``
    rather than the tail plus a rotated pixel offset, because the floor
    homography is a perspective map -- a heading is only a direction after it
    has been projected, and near the frame edge the two differ visibly.
    """
    tip = start_mm + length_mm * np.array([np.cos(theta0), np.sin(theta0)])
    a, b = W.mm_to_px(np.stack([start_mm, tip]), "floor")
    cv2.arrowedLine(
        vis, np.int32(a), np.int32(b), (255, 0, 255), 3, cv2.LINE_AA, tipLength=0.3
    )
    cv2.circle(vis, np.int32(a), 4, (255, 0, 255), -1, cv2.LINE_AA)


def overlay(img, M, out=None, pose=None):
    W = M["frame"]
    vis = img.copy()
    cv2.polylines(
        vis,
        [np.int32(W.mm_to_px(M["deck_mm"], "floor"))],
        True,
        (255, 200, 0),
        1,
        cv2.LINE_AA,
    )
    for s in M["wall_segments_mm"]:
        a, b = W.mm_to_px(s, "floor")
        cv2.line(vis, np.int32(a), np.int32(b), (0, 0, 255), 2, cv2.LINE_AA)
    th = np.linspace(0, 2 * np.pi, 64)
    for c in M["post_centres_mm"]:
        ring = c + M["post_radius_mm"] * np.stack([np.cos(th), np.sin(th)], 1)
        cv2.polylines(
            vis,
            [np.int32(W.mm_to_px(ring, "floor"))],
            True,
            (255, 0, 255),
            1,
            cv2.LINE_AA,
        )
    for c in M["cylinders"]:
        ring = c["centre_mm"] + c["radius_mm"] * np.stack([np.cos(th), np.sin(th)], 1)
        cv2.polylines(
            vis,
            [np.int32(W.mm_to_px(ring, "floor"))],
            True,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
    if out is not None:
        V, par = np.asarray(out["nodes"])[:, :2], out["parent"]
        for k in range(1, len(V)):
            if par[k] >= 0:
                a, b = W.mm_to_px(np.stack([V[par[k]], V[k]]), "floor")
                cv2.line(vis, np.int32(a), np.int32(b), (190, 190, 190), 1, cv2.LINE_AA)
        if out["path"] is not None:
            p = W.mm_to_px(out["path"], "floor")
            cv2.polylines(vis, [np.int32(p)], False, (0, 255, 0), 2, cv2.LINE_AA)
            # arc/straight joins, so the segment structure is visible on the image
            for s in out.get("segments") or []:
                j = W.mm_to_px(s.end[None], "floor")[0]
                cv2.circle(vis, np.int32(j), 2, (0, 140, 255), -1, cv2.LINE_AA)
            cv2.circle(vis, np.int32(p[0]), 5, (255, 0, 255), -1)
            cv2.circle(vis, np.int32(p[-1]), 5, (255, 60, 0), -1)
    # last, so it stays readable over the tree and over a path that leaves the
    # start along the same heading -- and it is drawn whether or not there is a
    # path, since a wrong theta0 is a common reason there is not one
    if pose is not None:
        start_arrow(vis, W, np.asarray(pose[:2], float), float(pose[2]))
    return vis


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("image", nargs="?", default="mazes/4.png")
    # 0,1 deliberately is not the default: the deck's chamfer leaves that corner
    # cell a ~1 mm escape slot, which a straight-line planner threads and a
    # bounded-curvature one cannot.  See the README.
    ap.add_argument("--from", dest="src", default="1,1")
    ap.add_argument("--to", dest="dst", default="7,7")
    ap.add_argument("--from-mm", dest="src_mm", default=None)
    ap.add_argument("--to-mm", dest="dst_mm", default=None)
    ap.add_argument("--r", type=float, default=40.0, help="robot radius, mm")
    ap.add_argument("--mode", choices=("dubins", "polyline"), default="dubins")
    ap.add_argument(
        "--theta0", default="auto", help="start heading in deg, or 'auto' (default)"
    )
    ap.add_argument(
        "--theta1", default="free", help="goal heading in deg, or 'free' (default)"
    )
    ap.add_argument(
        "--turn-radius", type=float, default=30.0, help="arc radius the path uses, mm"
    )
    ap.add_argument("--iters", type=int, default=4000)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", default="map_overlay.png")
    ap.add_argument("--emit", default=None, help="write the appendSegment() block here")
    ap.add_argument("--json", dest="js", default=None, help="write segments as JSON")
    a = ap.parse_args()

    img = cv2.imread(a.image)
    assert img is not None, a.image
    res = mg.solve(img)
    M = mp.build_map(img, res)
    W = M["frame"]
    nb = M["walls"]["vertical"].size + M["walls"]["horizontal"].size

    print(
        f"lattice    {W.shape[0]}x{W.shape[1]} nodes, {res['pitch_px']:.2f} px "
        f"pitch, rms {res['rms_px'] * res['mm_per_px']:.1f} mm"
    )
    print(f"walls      {len(M['wall_segments_mm'])} panels of {nb} bonds")
    if M["walls"]["ambiguous"]:
        print(
            "  within 20 of threshold: "
            + ", ".join(
                f"{k[0]}({i},{j}) {s:.0f}" for k, i, j, s in M["walls"]["ambiguous"][:6]
            )
        )
    P = M["posts"]
    print(
        f"posts      {int(P['present'].sum())} of {P['present'].size} nodes "
        f"({int((P['present'] & P['detected']).sum())} by blue cap, "
        f"{int((P['present'] & ~P['detected']).sum())} by silhouette), "
        f"{len(M['isolated_posts'])} with no adjoining panel, "
        f"{P['pruned']} claims pruned off-deck or under a cylinder"
    )
    if P["ambiguous"]:
        print(
            f"  within 15 of threshold {P['thresh']:.0f}: "
            + ", ".join(f"({i},{j}) {s:.0f}" for i, j, s in P["ambiguous"][:6])
        )
    print(
        f"deck       {len(M['deck_mm'])}-gon, "
        f"{M['deck_mm'].min(0).round(0)} .. {M['deck_mm'].max(0).round(0)} mm"
    )
    for k, c in enumerate(M["cylinders"]):
        print(
            f"cylinder{k}  centre {np.round(c['centre_mm'], 1)} mm, "
            f"r {c['radius_mm']:.1f} mm (bound {c['radius_mm_bound']:.1f}), "
            f"silhouette rms {c['fit_rms_mm']:.1f} mm"
        )
    print()
    print(ascii_maze(M["walls"], P))
    print()

    world = rs.MazeWorld(
        M["wall_segments_mm"],
        M["cylinders"],
        M["deck_mm"],
        robot_radius_mm=a.r,
        posts_mm=M["post_centres_mm"],
        post_radius_mm=M["post_radius_mm"],
    )

    def parse(cell, mm):
        if mm is not None:
            return np.array([float(v) for v in mm.split(",")])
        i, j = (int(v) for v in cell.split(","))
        return W.cell_centre_mm(i, j)

    start, goal = parse(a.src, a.src_mm), parse(a.dst, a.dst_mm)
    theta0 = (
        rs.best_heading(world, start)
        if a.theta0 == "auto"
        else np.radians(float(a.theta0))
    )
    print(f"free width {world.free_width_mm:.0f} mm for r={a.r:.0f} mm")
    print(
        f"start      {np.round(start)} mm heading {np.degrees(theta0):.0f} deg"
        f"{' (auto)' if a.theta0 == 'auto' else ''}"
    )

    if a.mode == "polyline":
        out = rs.plan(world, start, goal, max_iter=a.iters, seed=a.seed)
    else:
        th1 = None if a.theta1 == "free" else np.radians(float(a.theta1))
        out = rs.plan_dubins(
            world,
            (start[0], start[1], theta0),
            (goal[0], goal[1], th1),
            rho=a.turn_radius,
            max_iter=a.iters,
            seed=a.seed,
        )

    if out["path"] is None:
        print(f"no path {np.round(start)} -> {np.round(goal)} in {out['iters']} it")
    else:
        cl = world.clearance(rs.resample(out["path"], 5.0))
        print(
            f"path       {len(out['path'])} waypoints, {out['cost']:.0f} mm "
            f"({out['raw_cost']:.0f} mm before shortcutting), "
            f"{len(out['nodes'])} nodes, step {out['step_mm']:.0f} mm"
        )
        print(f"clearance  {cl.min():.1f} mm minimum along the path")

    segs = out.get("segments")
    if segs:
        arcs = sum(s.is_arc for s in segs)
        print(
            f"segments   {len(segs)} ({len(segs) - arcs} straight, {arcs} arc "
            f"at r={out['rho']:.0f} mm), {sg.length(segs):.0f} mm; hold "
            f"cruiseVelocity <= {rs.speed_limit_mm_s(out['rho']):.0f} mm/s"
        )
        problems = sg.check(segs)
        if not problems:
            print("           firmware-representable, joins continuous")
        else:
            print(f"           {len(problems)} problem(s):")
            for p in problems[:8]:
                print("  PROBLEM  " + p)

        fw, _ = sg.to_firmware(segs, (start[0], start[1], theta0), local=True)
        block = sg.to_cpp(fw, note=f"{a.src} -> {a.dst}, r={a.turn_radius:.0f} mm")
        print()
        print(block)
        if a.emit:
            with open(a.emit, "w") as fh:
                fh.write(block + "\n")
            print("wrote", a.emit)
        if a.js:
            with open(a.js, "w") as fh:
                json.dump(
                    dict(
                        frame="robot: x forward, y left, mm; start pose (0,0,0)",
                        turn_radius_mm=out["rho"],
                        segments=sg.to_dicts(fw),
                    ),
                    fh,
                    indent=1,
                )
            print("wrote", a.js)

    cv2.imwrite(a.out, overlay(img, M, out, pose=(start[0], start[1], theta0)))
    print("wrote", a.out)


if __name__ == "__main__":
    main()
