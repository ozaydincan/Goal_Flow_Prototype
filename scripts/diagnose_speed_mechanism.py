import sys; sys.path.insert(0, "src")
from dataclasses import replace
import numpy as np, torch
import torch.nn.functional as F
from gfsem.planners.goalflow_planner import GoalFlowPlanner
from gfsem.sim.closed_loop import run_episode
from gfsem.sim.config import SceneConfig, SimConfig
from gfsem.sim.raster import rasterize
from gfsem.sim.scene import generate_scene
class Rec(GoalFlowPlanner):
    def reset(self):
        super().reset(); self.rows = []
    @torch.no_grad()
    def plan(self, obs, scene, true_pose, state):
        k = self.k
        res = super().plan(obs, scene, true_pose, state)
        torch.manual_seed(self.seed * 1_000_003 + k)
        bev = torch.from_numpy(rasterize(obs, self.rc))[None].to(self.dev)
        ego = torch.from_numpy(np.asarray(obs.ego_state, np.float32))[None].to(self.dev)
        r = self.model.plan(bev, ego, M=self.M, n_steps=1, goal_K=self.goal_K, safety=True,
                            safety_points=[self.filter_points], safety_margin=self.safety_margin)
        V = self.model.scorer.vocab
        sc = r["goal_score"][0]
        g = r["goals"][0, :, :2].norm(dim=-1)
        raw_end = r["samples_raw"][0][:, -1].norm(dim=-1)
        ex_end = r["samples"][0][:, -1].norm(dim=-1)
        self.rows.append(dict(v=float(obs.ego_state[0]), raw=float(raw_end.median()), ex=float(ex_end.median()),
                              goal_d=float(g.mean()), top_score=float(sc.max()), brake=bool(res.brake)))
        return res
out = {}
for v in (3.0, 2.0):
    P = Rec(circle_filter=True, safety_margin=0.12, edge_w=1.0, speed=v)
    cfg = SimConfig(scene=replace(SceneConfig.from_yaml(), spawn_speed=v, target_speed=v))
    rows = []
    for seed in range(6):
        run_episode(generate_scene(seed), P, "L0", cfg); rows += P.rows
    a = {k: np.median([r[k] for r in rows]) for k in ("v", "raw", "ex", "goal_d", "top_score")}
    a["brake"] = np.mean([r["brake"] for r in rows]); a["scans"] = len(rows)
    out[v] = a
    print(f"speed {v}: " + ", ".join(f"{k} {a[k]:.3f}" for k in a), flush=True)
print("vocab endpoint distance: median %.2f, p10 %.2f, p90 %.2f m" % tuple(np.percentile(P.model.scorer.vocab[:, :2].norm(dim=-1).cpu().numpy(), [50, 10, 90])))
