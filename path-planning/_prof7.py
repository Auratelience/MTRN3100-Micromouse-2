import time, numpy as np, cv2, importlib.util
import maze_grid as mg, maze_map as mp, rrt_star as rs

img = cv2.imread("mazes/1.png"); res = mg.solve(img); M = mp.build_map(img, res)
W=M["frame"]
kw=dict(robot_radius_mm=40, posts_mm=M["post_centres_mm"], post_radius_mm=M["post_radius_mm"])
spec=importlib.util.spec_from_file_location("_rrt_patched","_rrt_patched.py")
pm=importlib.util.module_from_spec(spec); spec.loader.exec_module(pm)
w1=rs.MazeWorld(M["wall_segments_mm"],M["cylinders"],M["deck_mm"],**kw)
w2=pm.MazeWorld(M["wall_segments_mm"],M["cylinders"],M["deck_mm"],**kw)
PAIRS=[((1,1),(7,7)),((1,4),(7,2)),((2,0),(6,8)),((0,4),(8,4)),((4,4),(1,8))]
print("=== success rate + time, 5 trips x 3 seeds, 4000 iters ===")
T={'cur':0.0,'gate':0.0}; OK={'cur':0,'gate':0}; N=0
for a,b in PAIRS:
    s,g=W.cell_centre_mm(*a),W.cell_centre_mm(*b)
    if not (w1.is_free(s)[0] and w1.is_free(g)[0]): print(f"  skip {a}->{b}"); continue
    th=rs.best_heading(w1,s)
    row=[]
    for tag,mod,wd in (('cur',rs,w1),('gate',pm,w2)):
        ok=0; cs=[]; t0=time.perf_counter()
        for sd in range(3):
            r=mod.plan_dubins(wd,(s[0],s[1],th),(g[0],g[1],None),max_iter=4000,seed=sd)
            if r['path'] is not None: ok+=1; cs.append(r['cost'])
        dt=time.perf_counter()-t0
        T[tag]+=dt; OK[tag]+=ok
        row.append((tag,ok,np.mean(cs) if cs else float('nan'),dt))
    N+=3
    print(f"  {a}->{b}  current {row[0][1]}/3 {row[0][2]:6.0f}mm {row[0][3]:6.2f}s   |   gated {row[1][1]}/3 {row[1][2]:6.0f}mm {row[1][3]:5.2f}s")
print(f"\n  TOTAL current {OK['cur']}/{N} in {T['cur']:.1f}s   gated {OK['gate']}/{N} in {T['gate']:.1f}s   -> {T['cur']/T['gate']:.2f}x faster overall")
