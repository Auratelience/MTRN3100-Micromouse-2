import time, numpy as np, cv2
import maze_grid as mg, maze_map as mp, rrt_star as rs, dubins as db

img = cv2.imread("mazes/1.png"); res = mg.solve(img); M = mp.build_map(img, res)
W=M["frame"]
world = rs.MazeWorld(M["wall_segments_mm"], M["cylinders"], M["deck_mm"],
    robot_radius_mm=40, posts_mm=M["post_centres_mm"], post_radius_mm=M["post_radius_mm"])
s=W.cell_centre_mm(1,1); g=W.cell_centre_mm(7,7); th=rs.best_heading(world,s)

# record the Dubins connect length of every goal-connect edge check
lens=[]; before_first=[0]; found=[False]
orig=rs._edge_valid
import inspect
def spy(q0,word,params,rho,ds,wd):
    if inspect.currentframe().f_back.f_lineno==750:
        L=float(np.sum(params)*rho); lens.append(L)
        if not found[0]: before_first[0]+=1
    return orig(q0,word,params,rho,ds,wd)
rs._edge_valid=spy
r=rs.plan_dubins(world,(s[0],s[1],th),(g[0],g[1],None),max_iter=4000,seed=1)
rs._edge_valid=orig
L=np.array(lens)
print(f"goal-connect edge checks: {len(L)}")
print(f"  Dubins connect length: median {np.median(L):.0f} mm  mean {L.mean():.0f}  p90 {np.percentile(L,90):.0f}  max {L.max():.0f}")
for thr in (200,400,600,1000):
    print(f"  longer than {thr:4d} mm (hopeless, yet fully collision-checked): {100*(L>thr).mean():5.1f}%  ({(L>thr).sum()} checks)")
print(f"\nstart->goal straight-line distance is {np.linalg.norm(g-s):.0f} mm; maze is ~{world.hi[0]-world.lo[0]:.0f} mm across")

print("\n=== hypothesis test: add ONLY a distance gate to goal-connect ===")
src=open("rrt_star.py").read()
patched=src.replace(
"""        Lg, kg, pg = db.lengths(q_new[None], goal_poses, rho)
        for m in np.argsort(Lg):
            if c_new + Lg[m] >= best_cost:
                break""",
"""        Lg, kg, pg = db.lengths(q_new[None], goal_poses, rho)
        _gate = float(np.linalg.norm(q_new[:2] - goal_xy)) <= 2.0 * step_mm
        for m in np.argsort(Lg):
            if c_new + Lg[m] >= best_cost or not _gate:
                break""")
assert patched!=src
open("_rrt_patched.py","w").write(patched)
import importlib.util
spec=importlib.util.spec_from_file_location("_rrt_patched","_rrt_patched.py")
pm=importlib.util.module_from_spec(spec); spec.loader.exec_module(pm)
w2=pm.MazeWorld(M["wall_segments_mm"], M["cylinders"], M["deck_mm"],
    robot_radius_mm=40, posts_mm=M["post_centres_mm"], post_radius_mm=M["post_radius_mm"])
for seed in range(4):
    t=time.perf_counter(); a=rs.plan_dubins(world,(s[0],s[1],th),(g[0],g[1],None),max_iter=4000,seed=seed); ta=time.perf_counter()-t
    t=time.perf_counter(); b=pm.plan_dubins(w2,(s[0],s[1],th),(g[0],g[1],None),max_iter=4000,seed=seed); tb=time.perf_counter()-t
    ca = a['cost'] if a['path'] is not None else float('inf')
    cb = b['cost'] if b['path'] is not None else float('inf')
    print(f"  seed {seed}: current {ta:5.2f}s cost {ca:7.0f} | +distance gate {tb:5.2f}s cost {cb:7.0f}  -> {ta/tb:4.2f}x faster")
