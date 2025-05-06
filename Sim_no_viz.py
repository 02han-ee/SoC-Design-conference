# sim_no_viz.py
"""Minimal, head‑less re‑implementation of the original p5.js LiDAR
simulator in **pure Python** for Colab / GPU execution.

🔹  Visualization & UI are stripped – the script runs batches of
    simulations, records per‑frame metrics, and writes a CSV for each
    experiment automatically.
🔹  Core logic (world generation, robot kinematics, LiDAR ray‑casting,
    DR / KF / PF / ICP‑like tracking, RMSE & power estimation) mirrors
    the original JS as closely as possible.
🔹  NumPy is the default math backend; PyTorch can be enabled for GPU
    acceleration by setting USE_TORCH = True.

Usage (Colab):
```
!python sim_no_viz.py             # runs default experiment sweep

# or customise
!python sim_no_viz.py --algos PF KF --resolutions 1 5 10 --frames 9000
```
The script drops `<algo>_R<res>.csv` files in the working directory.
"""

from __future__ import annotations
import math, random, argparse, csv, time, pathlib, itertools
from dataclasses import dataclass, field
from typing import List, Tuple

import numpy as np
import pandas as pd

# ───────────────────────────── Backend toggle ──────────────────────────────
USE_TORCH = False  # set True to run on GPU via torch
if USE_TORCH:
    import torch
    T = torch.tensor
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
else:
    T = np.asarray  # fall‑back helper
    device = None

# ───────────────────────────── Geometry helpers ────────────────────────────
Vec = Tuple[float, float]

def vec_add(a: Vec, b: Vec) -> Vec: return (a[0]+b[0], a[1]+b[1])
def vec_sub(a: Vec, b: Vec) -> Vec: return (a[0]-b[0], a[1]-b[1])
def vec_scale(a: Vec, s: float) -> Vec: return (a[0]*s, a[1]*s)

def dist2(a: Vec, b: Vec) -> float:
    dx, dy = a[0]-b[0], a[1]-b[1]
    return dx*dx + dy*dy

def dist(a: Vec, b: Vec) -> float:
    return math.sqrt(dist2(a, b))

def lerp(a: float, b: float, t: float) -> float:
    return a + (b-a)*t

def angle_diff(a: float, b: float) -> float:
    d = (a-b) % (2*math.pi)
    if d > math.pi:
        d -= 2*math.pi
    if d < -math.pi:
        d += 2*math.pi
    return d

# ───────────────────────────── World primitives ────────────────────────────
@dataclass
class Boundary:
    a: Vec
    b: Vec

# Fast param‑form ray/segment intersect (same as JS version)

def ray_cast(origin: Vec, direction: Vec, wall: Boundary, max_dist: float) -> Tuple[bool, Vec, float]:
    (x1, y1), (x2, y2) = wall.a, wall.b
    x3, y3 = origin
    x4, y4 = origin[0] + direction[0]*max_dist, origin[1] + direction[1]*max_dist

    den = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)
    if abs(den) < 1e-9:
        return False, (0, 0), max_dist
    t = ((x1-x3)*(y3-y4) - (y1-y3)*(x3-x4)) / den
    u = -((x1-x2)*(y1-y3) - (y1-y2)*(x1-x3)) / den
    if 0 < t < 1 and 0 < u < 1:
        px = x1 + t*(x2-x1)
        py = y1 + t*(y2-y1)
        d = dist(origin, (px, py))
        return True, (px, py), d
    return False, (0, 0), max_dist

# ───────────────────────── Robot agent ─────────────────────────────────────
@dataclass
class Robot:
    pos: Vec
    heading: float = 0.0

    # internal state
    est_pos: Vec = field(default_factory=lambda: (0.0, 0.0))
    kf_state: np.ndarray | None = None  # [x,y,theta]
    kf_P: np.ndarray | None = None
    particles: List[Tuple[float, float, float, float]] | None = None  # x,y,theta,weight
    last_scan: List[Vec] = field(default_factory=list)

    max_lidar_dist: float = 150.0

    def update_motion(self, dt: float, auto_phase: int):
        # simple scripted auto behaviour (same as JS)
        if auto_phase == 0:
            v = 2.0; w = 0.0
        elif auto_phase == 1:
            v = 0.0; w = -0.03
        else:
            v = 2.0; w = 0.0
        self.heading += w * dt
        dx = v*dt* math.cos(self.heading)
        dy = v*dt* math.sin(self.heading)
        self.pos = (self.pos[0]+dx, self.pos[1]+dy)

    # --- LiDAR scan -------------------------------------------------------
    def scan(self, walls: List[Boundary], resolution: int) -> List[Vec]:
        pts = []
        for a_deg in range(-45, 46, resolution):
            ang = math.radians(a_deg) + self.heading
            dir_vec = (math.cos(ang), math.sin(ang))
            best_d = self.max_lidar_dist
            best_pt = None
            for w in walls:
                hit, pt, d = ray_cast(self.pos, dir_vec, w, self.max_lidar_dist)
                if hit and d < best_d:
                    best_d, best_pt = d, pt
            pts.append(best_pt if best_pt else (self.pos[0]+dir_vec[0]*self.max_lidar_dist,
                                                self.pos[1]+dir_vec[1]*self.max_lidar_dist))
        return pts

# ───────────────────────── Experiments & metrics ───────────────────────────

BASE_POWER = 5
RAY_COST = 0.05
PF_COST = 0.3
ICP_COST = 0.6

WINDOW = 30

@dataclass
class MetricsAccum:
    se_sum: float = 0.0
    n: int = 0
    def update_rmse(self, true_p: Vec, est_p: Vec):
        dx, dy = true_p[0]-est_p[0], true_p[1]-est_p[1]
        self.se_sum += dx*dx + dy*dy
        self.n += 1
    @property
    def rmse(self):
        return math.sqrt(self.se_sum / self.n) if self.n else 0.0

# ───────────────────────── KF utilities ------------------------------------

def kf_predict(state: np.ndarray, P: np.ndarray, v: float, omega: float, dt: float):
    x, y, th = state
    # motion
    pred = np.array([x + v*dt*math.cos(th),
                     y + v*dt*math.sin(th),
                     th + omega*dt])
    # Jacobian A
    A = np.eye(3)
    A[0,2] = -v*dt*math.sin(th)
    A[1,2] =  v*dt*math.cos(th)
    Q = np.diag([0.1,0.1,0.05])
    P_pred = A @ P @ A.T + Q
    return pred, P_pred

def kf_update(pred: np.ndarray, P_pred: np.ndarray, z: Vec):
    H = np.array([[1,0,0],[0,1,0]])
    R = np.diag([10,10])
    y = np.array(z) - H @ pred
    S = H @ P_pred @ H.T + R
    K = P_pred @ H.T @ np.linalg.inv(S)
    upd = pred + K @ y
    P_upd = (np.eye(3) - K @ H) @ P_pred
    return upd, P_upd

############################################################################

def icp_pose(prev_pts: List[Vec], curr_pts: List[Vec]) -> Tuple[float,float,float]:
    """두 스캔의 대응점(각도별 동일 인덱스)을 이용해
       2D rigid transform (dx,dy,dθ) 을 구한다."""
    A = np.array(prev_pts)   # N×2
    B = np.array(curr_pts)
    cA, cB = A.mean(axis=0), B.mean(axis=0)
    AA, BB = A - cA, B - cB
    H = AA.T @ BB
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:      # 반사 보정
        Vt[1,:] *= -1
        R = Vt.T @ U.T
    t = cB - R @ cA
    theta = math.atan2(R[1,0], R[0,0])
    return t[0], t[1], theta

############################################################################

# ───────────────────────── Main simulation routine ————————————————

def run_single_experiment(resolution: int, algo: str, n_frames: int = 9000,
                          csv_path: pathlib.Path | None = None):
    # 1. world generation (fixed seed for reproducibility)
    random.seed(0)
    walls: List[Boundary] = []
    W,H = 600,600
    # outer box
    walls += [Boundary((0,0),(W,0)), Boundary((W,0),(W,H)),
              Boundary((W,H),(0,H)), Boundary((0,H),(0,0))]
    # 5 random internal walls
    for _ in range(5):
        x1,y1 = random.uniform(50,W-50), random.uniform(50,H-50)
        x2,y2 = random.uniform(50,W-50), random.uniform(50,H-50)
        walls.append(Boundary((x1,y1),(x2,y2)))

    bot = Robot(pos=(W/4, H/2))
    bot.est_pos = bot.pos

    metrics = MetricsAccum()

    rows = []
    phase = 0; phase_time=0.0
    for frame in range(n_frames):
        dt = 1/60.0  # fixed 60 FPS equivalent
        phase_time += dt*1000
        if phase==0 and phase_time>2000: phase,phase_time=1,0
        elif phase==1 and phase_time>1000: phase,phase_time=2,0
        elif phase==2 and phase_time>2000: phase,phase_time=0,0

        # 1) Truth motion
        bot.update_motion(dt, phase)

        # 2) Sensor
        scan_pts = bot.scan(walls, resolution)

        # 3) Estimation algorithm
        if algo=="DR":
            noise = 0.1
            dx = 2*dt*math.cos(bot.heading) + random.gauss(0,noise)
            dy = 2*dt*math.sin(bot.heading) + random.gauss(0,noise)
            bot.est_pos = (bot.est_pos[0]+dx, bot.est_pos[1]+dy)

        elif algo=="KF":
            if bot.kf_state is None:
                bot.kf_state = np.array([*bot.est_pos, bot.heading])
                bot.kf_P = np.eye(3)
            pred, P_pred = kf_predict(bot.kf_state, bot.kf_P, 2, 0, dt)
            cx = sum(p[0] for p in scan_pts)/len(scan_pts)
            cy = sum(p[1] for p in scan_pts)/len(scan_pts)
            bot.kf_state, bot.kf_P = kf_update(pred, P_pred, (cx,cy))
            bot.est_pos = (bot.kf_state[0], bot.kf_state[1])

        elif algo=="PF":
            N = 1000
            if bot.particles is None:
                bot.particles = [(bot.est_pos[0], bot.est_pos[1], bot.heading, 1.0/N) for _ in range(N)]
            new_particles=[]
            for x,y,th,w in bot.particles:
                th2 = th + random.gauss(0,0.01)
                x2 = x + 2*dt*math.cos(th2) + random.gauss(0,0.5)
                y2 = y + 2*dt*math.sin(th2) + random.gauss(0,0.5)
                new_particles.append([x2,y2,th2,1.0])
            # weight by centroid error
            cx = sum(p[0] for p in scan_pts)/len(scan_pts)
            cy = sum(p[1] for p in scan_pts)/len(scan_pts)
            sigma=10
            w_sum=0
            for p in new_particles:
                e = dist((p[0],p[1]), (cx,cy))
                p[3] = math.exp(-(e*e)/(2*sigma*sigma))
                w_sum += p[3]
            if w_sum==0: w_sum=1
            for p in new_particles: p[3]/=w_sum
            # resample
            cumsum=np.cumsum([p[3] for p in new_particles])
            resampled=[]
            for i in range(N):
                r=random.random()
                idx=np.searchsorted(cumsum,r)
                resampled.append(new_particles[min(idx,N-1)].copy())
            bot.particles = resampled
            mx = sum(p[0] for p in bot.particles)/N
            my = sum(p[1] for p in bot.particles)/N
            bot.est_pos = (lerp(bot.est_pos[0], mx, 0.1), lerp(bot.est_pos[1], my, 0.1))

        # ICP/GraphSLAM placeholder (heavy) – skip heavy refine
        # ... could implement simplified centroid alignment as in JS

        elif algo == "ICP":
            if bot.last_scan:                       # 이전 스캔이 있을 때만 정합
                dx, dy, dth = icp_pose(bot.last_scan, scan_pts)
                bot.est_pos = (bot.est_pos[0] + dx,
                               bot.est_pos[1] + dy)
                bot.heading += dth                 # 추정 heading 갱신
            # 초기 프레임은 DR 처럼 약간 움직여 두면 수렴이 빨라짐


        # 4) Metrics
        metrics.update_rmse(bot.pos, bot.est_pos)
        num_rays = len(scan_pts)
        power = (BASE_POWER + num_rays * RAY_COST +
         (PF_COST * num_rays if algo == 'PF' else 0) +
         (ICP_COST          if algo == 'ICP' else 0))


        rows.append({
            'frame': frame,
            'rmse': metrics.rmse,
            'power': power,
            'trueX': bot.pos[0],
            'trueY': bot.pos[1],
            'estX': bot.est_pos[0],
            'estY': bot.est_pos[1],
            'algo': algo,
            'resolution': resolution,
        })
        bot.last_scan = scan_pts

    df = pd.DataFrame(rows)
    if csv_path is None:
        csv_path = pathlib.Path(f"{algo}_R{resolution}.csv")
    df.to_csv(csv_path, index=False)
    print(f"Saved → {csv_path}  (RMSE={metrics.rmse:.2f})")

## ───────────────────────── Entry‑point CLI ─────────────────────────────────
#if __name__ == "__main__":
#    parser = argparse.ArgumentParser()
#    parser.add_argument('--algos', nargs='+', default=['PF'])
#    parser.add_argument('--resolutions', nargs='+', type=int, default=[1,5,10,15])
#    parser.add_argument('--frames', type=int, default=9000)
#    args = parser.parse_args()
#
#    for res, algo in itertools.product(args.resolutions, args.algos):
#        run_single_experiment(resolution=res, algo=algo, n_frames=args.frames)
#

# ───────────────────────── Entry‑point CLI ─────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    # 4 개 알고리즘을 기본으로 돌리도록 수정
    parser.add_argument('--algos', nargs='+',
                        default=['DR', 'KF', 'PF', 'ICP'])
    parser.add_argument('--resolutions', nargs='+', type=int,
                        default=[1, 5, 10, 15])
    parser.add_argument('--frames', type=int, default=9000)
    args = parser.parse_args()

    for res, algo in itertools.product(args.resolutions, args.algos):
        run_single_experiment(resolution=res, algo=algo,
                              n_frames=args.frames)

