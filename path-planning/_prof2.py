import time, collections
import cv2, numpy as np
import maze_grid as mg, maze_map as mp, rrt_star as rs

img = cv2.imread("mazes/1.png"); res = mg.solve(img); M = mp.build_map(img, res)
W = M["frame"]
world = rs.MazeWorld(M["wall_segments_mm"], M["cylinders"], M["deck_mm"],
    robot_radius_mm=40, posts_mm=M["post_centres_mm"], post_radius_mm=M["post_radius_mm"])

# instrument clearance: record len(P) per call, and who called it
import inspect
sizes=[]; callers=collections.Counter()
orig = rs.MazeWorld.clearance
def spy(self, P):
    Pa = np.atleast_2d(np.asarray(P,float))
    sizes.append(len(Pa))
    f = inspect.currentframe().f_back
    callers[f.f_code.co_name] += 1
    return orig(self, P)
rs.MazeWorld.clearance = spy

s = W.cell_centre_mm(1,1); g = W.cell_centre_mm(7,7)
th = rs.best_heading(world, s)
t=time.perf_counter()
r = rs.plan_dubins(world,(s[0],s[1],th),(g[0],g[1],None),max_iter=4000,seed=1)
print(f"plan {time.perf_counter()-t:.2f}s nodes={len(r['nodes'])}")
a=np.array(sizes)
print(f"clearance calls {len(a)}   total points {a.sum()}   mean {a.mean():.1f}  median {np.median(a):.0f}  p90 {np.percentile(a,90):.0f}  max {a.max()}")
print("size histogram:", {k:int(v) for k,v in zip(*np.unique(np.clip(a,0,64), return_counts=True))})
print("callers:", callers.most_common())
print(f"obstacles: walls={len(world.A)} circles={len(world.circ)} polyedges={len(world.poly)}")
print(f"=> distance evals = {a.sum()} pts x {len(world.A)+len(world.circ)+len(world.poly)} obstacles = {a.sum()*(len(world.A)+len(world.circ)+len(world.poly))/1e6:.1f} M")
