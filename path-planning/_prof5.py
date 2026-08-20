import time
import cv2, numpy as np
import maze_grid as mg, maze_map as mp, rrt_star as rs, dubins as db

img = cv2.imread("mazes/1.png"); res = mg.solve(img); M = mp.build_map(img, res)
W = M["frame"]
world = rs.MazeWorld(M["wall_segments_mm"], M["cylinders"], M["deck_mm"],
    robot_radius_mm=40, posts_mm=M["post_centres_mm"], post_radius_mm=M["post_radius_mm"])
s = W.cell_centre_mm(1,1); g = W.cell_centre_mm(7,7); th = rs.best_heading(world, s)

print("=== raw search cost (pre-shortcut) vs final, is the search monotone? ===")
for it in (1000,2000,4000):
    r=rs.plan_dubins(world,(s[0],s[1],th),(g[0],g[1],None),max_iter=it,seed=1)
    print(f"  iters {it:5d}  raw_cost {r['raw_cost']:7.0f}  final(after shortcut) {r['cost']:7.0f}")

print("\n=== per-phase attribution of _edge_valid calls (seed 1, 4000 iters) ===")
import collections, inspect
calls=collections.Counter(); tsp=collections.Counter()
orig=rs._edge_valid
def spy(q0,word,params,rho,ds,wd):
    who = inspect.currentframe().f_back.f_lineno
    t0=time.perf_counter(); out=orig(q0,word,params,rho,ds,wd); dt=time.perf_counter()-t0
    calls[who]+=1; tsp[who]+=dt
    return out
rs._edge_valid=spy
t=time.perf_counter(); r=rs.plan_dubins(world,(s[0],s[1],th),(g[0],g[1],None),max_iter=4000,seed=1)
tot=time.perf_counter()-t
rs._edge_valid=orig
LBL={}
src=open("rrt_star.py").read().splitlines()
for ln in calls:
    LBL[ln]=f"line {ln}"
named={664:"steer",716:"choose-parent",734:"rewire",748:"goal-connect",826:"shortcut_edges"}
print(f"  total plan {tot:.2f}s")
for ln,c in calls.most_common():
    print(f"  {named.get(ln,'line '+str(ln)):16s} {c:6d} calls  {tsp[ln]:6.2f}s  ({100*tsp[ln]/tot:4.1f}% of plan)")
print(f"  sum edge_valid {sum(tsp.values()):.2f}s = {100*sum(tsp.values())/tot:.0f}% of plan")
