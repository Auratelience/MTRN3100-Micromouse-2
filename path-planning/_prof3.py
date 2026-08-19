import time
import cv2, numpy as np
import maze_grid as mg, maze_map as mp, rrt_star as rs

img = cv2.imread("mazes/1.png"); res = mg.solve(img); M = mp.build_map(img, res)
W = M["frame"]
world = rs.MazeWorld(M["wall_segments_mm"], M["cylinders"], M["deck_mm"],
    robot_radius_mm=40, posts_mm=M["post_centres_mm"], post_radius_mm=M["post_radius_mm"])

t_by_size = {"one":[0.0,0], "many":[0.0,0]}
relevant = []   # (n_obstacles_actually_near, total)
orig = rs.MazeWorld.clearance
def spy(self, P):
    Pa = np.atleast_2d(np.asarray(P,float))
    t0=time.perf_counter(); out = orig(self, P); dt=time.perf_counter()-t0
    k = "one" if len(Pa)==1 else "many"
    t_by_size[k][0]+=dt; t_by_size[k][1]+=1
    # how many walls/circles are within reach of this query's bbox?
    lo,hi = Pa.min(0), Pa.max(0)
    R = self.wall_clear + 1.0
    wm = ((np.minimum(self.A,self.B)[:,0]<=hi[0]+R)&(np.maximum(self.A,self.B)[:,0]>=lo[0]-R)&
          (np.minimum(self.A,self.B)[:,1]<=hi[1]+R)&(np.maximum(self.A,self.B)[:,1]>=lo[1]-R)).sum()
    Rc = self.circ_clear
    cm = ((self.circ[:,0]-self.circ[:,2]-Rc<=hi[0])&(self.circ[:,0]+self.circ[:,2]+Rc>=lo[0])&
          (self.circ[:,1]-self.circ[:,2]-Rc<=hi[1])&(self.circ[:,1]+self.circ[:,2]+Rc>=lo[1])).sum()
    relevant.append((wm+cm, len(self.A)+len(self.circ), len(Pa)))
    return out
rs.MazeWorld.clearance = spy

s = W.cell_centre_mm(1,1); g = W.cell_centre_mm(7,7)
th = rs.best_heading(world, s)
t=time.perf_counter()
r = rs.plan_dubins(world,(s[0],s[1],th),(g[0],g[1],None),max_iter=4000,seed=1)
tot=time.perf_counter()-t
print(f"plan total (instrumented) {tot:.2f}s")
for k,(dt,n) in t_by_size.items():
    print(f"  clearance[{k:4s}] {n:6d} calls  {dt:6.2f}s  {dt/max(n,1)*1e6:7.1f} us/call")
rel=np.array(relevant)
w = rel[:,2]  # weight by points
print(f"obstacles tested: {rel[0,1]} always")
print(f"obstacles actually near bbox: mean {rel[:,0].mean():.1f}, median {np.median(rel[:,0]):.0f}, p90 {np.percentile(rel[:,0],90):.0f}")
print(f"point-weighted: near {(rel[:,0]*w).sum()/w.sum():.1f} of {rel[0,1]}  -> {(rel[:,0]*w).sum()/(rel[:,1]*w).sum()*100:.1f}% of work is on obstacles that could matter")
