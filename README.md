# 🛰️ Streaming Airborne LiDAR Simulator (Rotating Multi-Beam)

_A Unity-based simulator for airborne rotating multi-beam LiDAR with real-time point cloud generation and on-the-fly scan-quality assessment._

---

## 🧭 Overview

This project simulates a **VLP-16-equivalent rotating multi-beam LiDAR** mounted on a
low-altitude UAV flying an automatic **S-pattern (boustrophedon) survey**.

Unlike offline simulators that buffer everything and save after the mission, this one
**streams the point cloud and writes quality metrics row-by-row during flight**, with a
**constant memory footprint** and **sub-millisecond per-sweep assessment latency**.

> Designed for UAV-LiDAR algorithm prototyping, pre-flight validation, and real-time
> data-quality feedback.

---

## 🔭 1. Simulation Principle

- **Rotating multi-beam emission:** 16 beams over a 30° along-track fan, rotated about the
  flight axis at 10 Hz; only the ground-facing half-revolution (`|ω| ≤ 75°` from nadir) fires.
- **Native pulse rate:** 50 Hz FixedUpdate × 360 firings/step = 18 kHz, i.e. ~28,800 pts/rev,
  matching the real VLP-16 (`ω/f = 0.2°/firing`).
- **Sub-stepped raycasting:** platform pose is linearly interpolated within each physics step
  so fast rotation is sampled correctly via `Physics.Raycast`.
- **Platform-motion model:** along-track acceleration and Gaussian vibration (cross-track /
  vertical) can be enabled to study their effect on point-cloud quality.

---

## 🧱 2. Core Components

| Script                            | Role                                                         | Key Methods                                                  |
| --------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| **SPatternScanFlightPhased.cs**   | UAV flight control, S-pattern trajectory, beam geometry, raycasting, point-cloud export | `PerformScanSubstepped()`, `ExecuteStraightFlight()`, `ExecuteTurning()` |
| **StreamingQualityAssessment.cs** | Streaming sweep-level quality metrics (ring-buffer, incremental, constant memory) | `OnFiring()`, `CommitCurrentSweep()`, `ComputeSweepCCI()`    |

---

## 🌲 3. Scene Setup (RAMI / LESS terrain)

Scenes are built from **RAMI / LESS** forest reconstructions (e.g. `HET09_JBS_SUM`) from https://lessrt.org/res_html/rami.html and
prepared for raycast-only scanning. Both are Editor tools.

| Script                  | Role                                                         | Entry point                                            |
| ----------------------- | ------------------------------------------------------------ | ------------------------------------------------------ |
| **RAMISceneBuilder.cs** | Instantiate OBJ objects from `objects.txt` + `instances.txt`, convert LESS (X,Y,Z, Z-up) → Unity (X,Z,Y, Y-up), auto-align ground to `y=0`, optional GPU instancing | Menu `RAMI ▸ Build Scene from LESS`                    |
| **RamiSceneTiler.cs**   | Detect a single patch's XZ extent and tile the prefab into a large `tilesAlongX × tilesAlongZ` scene | Inspector buttons `Auto Detect Extent` / `Build Tiles` |

Typical workflow:

1. Put the RAMI parameter files under `Assets/RAMI/<scene>/Parameters/`.
2. **RAMISceneBuilder** → *Build Scene from LESS* to instantiate the patch.
3. *Convert to Colliders* — removes MeshRenderers and adds MeshColliders so
   `Physics.Raycast` hits the geometry (rendering not needed for scanning; also avoids the
   PhysX Fast-Midphase crash on >2²¹-triangle meshes).
4. Save the result as a prefab, assign it to **RamiSceneTiler**, *Auto Detect Extent* →
   *Build Tiles* to cover the full survey area.

---

## 📐 4. Parameters

### LiDAR (`SPatternScanFlightPhased`)

| Parameter               | Description                  | Default |
| ----------------------- | ---------------------------- | ------- |
| `lines`                 | Beam/channel count           | 16      |
| `verticalFOV`           | Along-track fan angle Θ (°)  | 30      |
| `maxDetectionDistance`  | Max range (m)                | 100     |
| `rotationFrequency`     | Scanner rotation (Hz)        | 10      |
| `maxScanAngleFromNadir` | Ground-facing half-angle (°) | 75      |
| `firingsPerFixedUpdate` | Firings per physics step     | 360     |

### Flight

| Parameter                     | Description                         | Default  |
| ----------------------------- | ----------------------------------- | -------- |
| `targetHeight`                | AGL altitude (m)                    | 50       |
| `cruiseSpeed`                 | Cruise speed (m/s)                  | 5        |
| `swathWidth` / `swathLength`  | Line spacing / length (m)           | 20 / 400 |
| `numberOfSwaths`              | Number of survey lines              | 5        |
| `turnRadius` / `maxBankAngle` | Turn radius (m) / bank limit (°)    | 30 / 30  |
| `cruiseAcceleration`          | Along-track accel a (m/s², 0 = off) | 0        |
| `vibrationSigma`              | Vibration σ_v (m/s, 0 = off)        | 0        |

### Quality Metrics (`StreamingQualityAssessment`)

| Parameter       | Description                   | Default |
| --------------- | ----------------------------- | ------- |
| `WindowSize`    | Sliding window W (sweeps)     | 50      |
| `PDVRThreshold` | Point density variation alert | 0.10    |
| `ISCThreshold`  | Inter-sweep spacing alert     | 0.15    |
| `CCIThreshold`  | Coverage completeness alert   | 0.60    |
| `CCIBinCount`   | Cross-track bins Q            | 4       |

---

## 📊 5. Quality Metrics

Three complementary metrics are computed incrementally, one value per ground-facing sweep:

| Metric   | Meaning                                                      | Flag when |
| -------- | ------------------------------------------------------------ | --------- |
| **PDVR** | Point Density Variation Rate between consecutive sweeps      | `> 0.10`  |
| **ISC**  | Inter-Sweep spacing Consistency (CV of sweep spacing)        | `> 0.15`  |
| **CCI**  | Coverage Completeness Index (actual / expected per cross-track bin × uniformity) | `< 0.60`  |

### Controlled single-effect experiments

To isolate how each platform motion affects the metrics, three minimal scanners reuse the
**exact same scan kernel** as the full S-pattern but replace the trajectory with one clean motion:

| Script                | Motion                                             | Tests                                                  |
| --------------------- | -------------------------------------------------- | ------------------------------------------------------ |
| **CruiseScanner.cs**  | Constant-velocity straight line                    | Steady-state baseline (PDVR ≈ 0, low ISC, CCI ≈ const) |
| **AccelScanner.cs**   | Constant along-track acceleration                  | PDVR response to density drift (`(1/D)(dD/dt) = −a/v`) |
| **BankingScanner.cs** | Straight → single 180° coordinated turn → straight | CCI drop / ISC rise during banked turn                 |

Each writes the same metric CSV with a `Phase` column (`Cruise` / `Accel` / `Straight` /
`Turning`) so the effect is directly traceable in the output.

---

## 💾 6. Data Output

All files are written to `/PointClouds/<sessionId>/` (`sessionId` = `yyyyMMdd_HHmmss`).

| File                            | Content                                                      |
| ------------------------------- | ------------------------------------------------------------ |
| `QualityMetrics_<id>.csv`       | Per-sweep `ScanIndex,Timestamp,PDVR,ISC,CCI,Warning,Phase,Pi,RhoI` |
| `QualitySummary_<id>.txt`       | Aggregate mean / max / threshold-exceedance summary          |
| `Performance_<id>.csv`          | Per-step `Step,Timestamp,ScanMs,MetricMs,MemMB` (latency & memory) |
| `SPattern_PointCloud_<id>.txt`  | `x y z range intensity scan_angle` (if `savePointCloud`)     |
| `SPattern_RangeMatrix_<id>.txt` | Per-scan range rows (if `saveRangeMatrix`)                   |
| `SPattern_Trajectory_<id>.txt`  | Per-step platform pose `s_t` (if `saveTrajectory`)           |

> For performance/time-series runs, set `savePointCloud` / `saveRangeMatrix` /
> `enablePointDisplay` to **false** to keep memory constant.

---

## 🚀 7. Usage

1. Open the scene in Unity `2022.3.62f` and build the terrain (see Section 3).
2. Attach `SPatternScanFlightPhased` to the UAV object and `StreamingQualityAssessment`
   to any object, then assign the latter to the `qualityAssessor` field.
3. Set LiDAR / flight / metric parameters in the Inspector.
4. Press **Play** — scanning starts automatically; metrics stream live and a quality
   panel renders the point cloud.
5. The flight stops at the last swath; all CSV/TXT outputs are saved automatically.

> For the controlled experiments, use `CruiseScanner` / `AccelScanner` / `BankingScanner`
> instead of `SPatternScanFlightPhased` (same setup, single motion).

---

## 📈 8. Visualization & Analysis

- Import `*_PointCloud_*.txt` into **CloudCompare**, **Open3D**, or **MATLAB**.
- Plot `QualityMetrics_*.csv` to inspect PDVR / ISC / CCI over the flight.
- Use `Performance_*.csv` to verify constant memory and sub-ms metric latency.

---

## 🧩 9. Future Work

- Reflectivity / noise modeling
- PLY / PCD export
- Additional flight patterns and terrain loaders

---

## 📜 License

Released under the **MIT License**. Free for research and educational use with attribution.



---

## 🇨🇳 中文简介

基于 Unity 的机载旋转多线束 LiDAR 仿真器(VLP-16 等效),无人机自动执行 S 型航线扫描,
地形来自 RAMI / LESS 森林重建场景。**飞行过程中实时流式生成点云并逐行计算扫描质量指标
(PDVR / ISC / CCI)**,内存占用恒定,单 sweep 评估延迟亚毫秒。另附 Accel / Banking/  Cruise 
三个受控实验脚本,分别隔离匀速、加速、转弯对点云质量的影响。可用于 UAV-LiDAR 算法原型验证、
飞行前测试与实时质量反馈。
