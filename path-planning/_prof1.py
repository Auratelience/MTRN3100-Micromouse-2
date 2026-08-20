import time, cProfile, pstats, io
import cv2, numpy as np
import maze_grid as mg, maze_map as mp, rrt_star as rs

t=time.perf_counter()
img = cv2.imread("mazes/1.png")
res = mg.solve(img)
M = mp.build_map(img, res)
print(f"vision       {time.perf_counter()-t:.2f} s")
W = M["frame"]
world = rs.MazeWorld(M["wall_segments_mm"], M["cylinders"], M["deck_mm"],
    robot_radius_mm=40, posts_mm=M["post_centres_mm"], post_radius_mm=M["post_radius_mm"])
print(f"walls {len(world.A)}  circles {world.circ.shape}  poly {None if world.poly is None else world.poly.shape}")

s = W.cell_centre_mm(1,1); g = W.cell_centre_mm(7,7)
th = rs.best_heading(world, s)

t=time.perf_counter()
r = rs.plan_dubins(world,(s[0],s[1],th),(g[0],g[1],None),max_iter=4000,seed=1)
dt=time.perf_counter()-t
print(f"plan_dubins  {dt:.2f} s  path={'ok' if r['path'] is not None else 'FAIL'} nodes={len(r['nodes'])} iters={r['iters']}")

pr=cProfile.Profile(); pr.enable()
rs.plan_dubins(world,(s[0],s[1],th),(g[0],g[1],None),max_iter=4000,seed=1)
pr.disable()
st=pstats.Stats(pr); st.sort_stats("tottime"); st.print_stats(28)
