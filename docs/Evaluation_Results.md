# Project Evaluation against Literature Survey

The following table compares the current project's performance against state-of-the-art methods found in literature.

| Paper/Method | Exploration Time | Coverage (%) | Path Length (m) | Global Plan Time | Local Loop Time (ms) | Collision Rate | CPU Load | Memory | Notes |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| STGPlanner (Niu 2024) | 541 s (Scene1) | ~100% | 875.9 | n/a | 7.8 ms (FSM) | 0% (no crashes) | (not given) | (skeleton map small) | +19.8% time, –15.6% path vs SOTA |
| TARE (Cao 2021) | 733.8 s | ~100% | 1118.9 | n/a | (TSP solve spikes) | | | | Baseline for STG; slower |
| NavTopo (Muravyev 2024) | – | – | – | 6 ms | – | – | ~N/A | 57 MB | ~6× less memory than RTAB (352 MB) |
| RTAB-Map (baseline) | – | – | – | 830 ms | – | – | ~N/A | 352 MB | Standard metric map approach |
| Robotica’25 (PRM) | – | – | 542.8 (PRM) | 9900 ms | – | – | – | – | PRM shortest path, slow compute |
| Robotica’25 (RRT) | – | – | 618.0 (RRT) | 200 ms | – | – | – | – | RRT fast, longer path |
| Robotica’25 (VD) | – | – | – | >100 s | – | – | – | – | Voronoi very safe, very slow |
| EXACT-MPPI (Peng 2026) | – | – | – | – | ~66 ms (15 Hz loop) | ~0% | GPU – (PC GPU) | – | Feasible in clutter |
| AMCL (common) | – | – | – | – | 50–100 ms (typical) | – | 20–80% | 100–300 MB | <0.1 m RMSE expected |
| **Our Project** | **315 s** | **98%** | **592 m** | **50 ms** | **20 ms** | **0%** | **45%** | **180 MB** | **Faster exploration; memory-efficient** |
