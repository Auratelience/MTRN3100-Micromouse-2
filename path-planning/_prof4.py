import time
import cv2, numpy as np
import maze_grid as mg, maze_map as mp, rrt_star as rs

img = cv2.imread("mazes/1.png"); res = mg.solve(img); M = mp.build_map(img, res)
W = M["frame"]
world = rs.MazeWorld(M["wall_segments_mm"], M["cylinders"], M["deck_mm"],
    robot_radius_mm=40, posts_mm=M["post_centres_mm"], post_radius_mm=M["post_radius_mm"])
s = W.cell_centre_mm(1,1); g = W.cell_centre_mm(7,7); th = rs.best_heading(world, s)

print("=== scaling: time vs max_iter (same seed) ===")
prev=None
for it in (250,500,1000,2000,4000):
    t=time.perf_counter(); r=rs.plan_dubins(world,(s[0],s[1],th),(g[0],g[1],None),max_iter=it,seed=1)
    dt=time.perf_counter()-t
    ratio = f" x{dt/prev:.2f}" if prev else ""
    print(f"  iters {it:5d}  {dt:6.2f}s  {dt/it*1000:5.2f} ms/iter{ratio}  "
          f"nodes {len(r['nodes']):4d}  cost {r['cost'] if r['path'] is not None else float('nan'):.0f} mm")
    prev=dt

print("\n=== anytime: when is the first solution found, and what does the rest buy? ===")
for seed in (0,1,2):
    hist=[]
    orig = rs.MazeWorld.curve_valid
    r=None
    # re-run with stop_on_first to find the discovery iteration
    t=time.perf_counter(); rf=rs.plan_dubins(world,(s[0],s[1],th),(g[0],g[1],None),max_iter=4000,seed=seed,stop_on_first=True)
    tf=time.perf_counter()-t
    t=time.perf_counter(); rfull=rs.plan_dubins(world,(s[0],s[1],th),(g[0],g[1],None),max_iter=4000,seed=seed)
    tfull=time.perf_counter()-t
    if rf['path'] is None: print(f"  seed {seed}: no path"); continue
    print(f"  seed {seed}: first solution at iter {rf['iters']:4d} in {tf:5.2f}s cost {rf['cost']:.0f} mm"
          f"  |  full 4000 iters {tfull:5.2f}s cost {rfull['cost']:.0f} mm"
          f"  -> {tfull/tf:4.1f}x time for {100*(1-rfull['cost']/rf['cost']):+.1f}% cost")
