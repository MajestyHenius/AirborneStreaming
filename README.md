# Streaming Airborne LiDAR Simulator

A Unity implementation of rotating multi-beam airborne LiDAR simulation with
incremental sweep-level quality assessment. The flight model, ray casting and
assessment run together; metric rows and warnings become available during
acquisition without requiring the complete mission point cloud.

## Requirements and setup

- Unity **2022.3.62f2c1**, as recorded in `ProjectSettings/ProjectVersion.txt`.
- Git LFS for the existing model, texture, prefab and Unity asset files.
- Package versions are recorded in `Packages/manifest.json` and
  `Packages/packages-lock.json`.

```sh
git lfs install
git clone https://github.com/MajestyHenius/AirborneStreaming.git
cd AirborneStreaming
git lfs pull
```

Open the project with Unity Hub. Open one of these scenes directly from `Assets/Scenes`:

| Scene | Use |
| --- | --- |
| `RAMIS_HET51.unity` | HET51 forest survey |
| `RAMIS_HET09.unity` | HET09 forest survey |
| `RAMIS_HOM24.unity` | HOM24 forest survey |
| `PerlinTerrainHET51.unity` | Seeded, bare Perlin terrain, generated at runtime |

In the three forest scenes, the active `c-172` object runs
`SPatternScanFlightPhased`; its `qualityAssessor` points to
`StreamingQualityAssessment`. Press **Play** to start the survey. Point-cloud
display is disabled in these saved configurations; a black Game view does not
mean that scanning has stopped. Use the Scene view or the generated metric CSV
to inspect progress. Stop Play mode after the flight reports completion.

## Scanner and flight model

The scanner is an idealised configuration based on the VLP-16. Sixteen channel
directions share each firing-group pose and rotation phase. Their centres use
the equal-sector convention within a 30-degree fan; the implementation does not
apply individual sensor calibration or inter-channel firing delays.

The rigid-body flight is sampled by substep pose interpolation. At a 0.02 s
physics step, 360 firing groups per step give 18,000 groups/s. With 16 channels
and 10 revolutions/s, this represents **28,800 nominal channel-firing
opportunities per revolution**, not 28,800 valid returns. The 100 m ray range,
89-degree ground-facing gate and first ray-surface intersection determine the
valid returns. A sweep is indexed by a full revolution; its ground-facing
interval contributes the returns used for assessment.

The three forest scenes use the following saved configuration:

| Setting | Value |
| --- | --- |
| Channels / along-track fan | 16 / 30 degrees |
| Rotation / range / angular gate | 10 Hz / 100 m / 89 degrees |
| Physics step / firing groups per step | 0.02 s / 360 |
| Initial height above the reference plane | 50 m |
| Initial / accelerated speed | 5 / 15 m/s |
| Along-track acceleration | 2 m/s² |
| Strip count / length / spacing | 5 / 300 m / 60 m |
| Turn radius / lift multiplier | 30 m / 1.05 |
| Bank limit / bank smoothing | 30 degrees / 2 |
| Velocity-perturbation standard deviation / seed | 0 m/s / 1 |
| Assessment window / cross-track bins | 50 sweeps / 4 |
| PDVR / ISC / CCI warning thresholds | 0.10 / 0.15 / 0.60 |
| Independent expected counts | `useAbsoluteExpectation = true` |
| CCI diagnostic masks | `testDropoutFraction = 0`, `testDropBin = -1` |

When velocity perturbations are enabled, Gaussian increments are drawn at each
physics update. Keep the physics step and seed with the configuration when
comparing runs. Only one scanner component should be enabled on a flight object.

## Quality assessment

`StreamingQualityAssessment.OnFiring()` accumulates returns and commits a sweep
at the end of its ground-facing interval. The current sweep path computes:

- **PDVR:** the window mean of relative changes in adjacent sweep densities.
  Density uses valid-return count divided by nominal strip width and the
  horizontal displacement between sweep centroids.
- **ISC:** the coefficient of variation of those inter-sweep displacements.
- **CCI:** a valid-to-expected return ratio weighted by cross-track consistency.
  Expected counts come from channel geometry, reference-plane height and range,
  independently of observed returns. Eligible returns can come from foliage or
  other surfaces. CCI describes nominal-ray return completeness; it does not
  record which surface cells are occupied or measure ground visibility through
  vegetation.

The supplied scenes enable the independent-expectation CCI branch. The older
normalised-expectation and scan-line interfaces remain in the source for
compatibility; they are not the configuration of these scenes. Keep the
assessor's range and fan parameters consistent with the scanner.

The assessment maintains window summaries and cross-track accumulators,
requiring O(W + Q) state. Point-cloud export, range export, trajectory recording
and visualisation have separate storage lifecycles. In particular, enabling
point-cloud export retains text for later writing; it does not provide bounded
memory for the full application.

## Outputs and timing

Normal survey outputs are written under `PointClouds/<sessionId>/`:

| Output | Contents |
| --- | --- |
| `QualityMetrics_*.csv` | Sweep index, simulated timestamp, PDVR, ISC, CCI, warning, phase, return count and density |
| `QualitySummary_*.txt` | Aggregate metric and threshold statistics |
| `Performance_*.csv` | Legacy per-physics-step scan/assessment samples and managed-memory samples |
| `SPattern_Trajectory_*.txt` | Recorded Unity position, rotation and time, when trajectory recording is enabled |
| `SPattern_PointCloud_*.txt`, `SPattern_RangeMatrix_*.txt` | Optional buffered exports |

For explicit wall-clock timing, enable `enableSweepTiming` on the assessor
while using `SPatternScanFlightPhased`. `SweepTimingRecorder` writes
`PointClouds/TimingDiagnostics/<run>/SweepTiming.csv` using a monotonic
`Stopwatch` clock. It distinguishes:

- simulated collection interval;
- wall time from the first firing to computed metrics and warning;
- sweep-commit computation;
- accumulated firing-assessment plus commit computation.

The computed-result endpoint precedes result writing. These fields have different
meanings from `Performance_*.csv` and should not be used interchangeably.

## Terrain and auxiliary scripts

The bare-terrain scene uses `TerrainGenerator` through
`PerlinTerrainExperiment`: 420 m × 420 m, a 513 × 513 height map, seed 42,
20 m relief, noise scale 4, three octaves, persistence 0.5 and lacunarity 2.
The mean terrain elevation is translated to zero. The same phased flight and
scanner are used, with no trees. This scene records its height map, poses,
returns and a configuration manifest under `PointClouds/`.

`SPatternScanFlightPhasedLoop` provides repeated survey legs for long runs.
It is disabled in the forest scenes; its flight settings must be configured
explicitly before use. `CruiseScanner`, `AccelScanner`, `BankingScanner` and
older flight components are auxiliary examples with their own Inspector values.
They do not replace the three saved full-survey configurations above.

`RAMISceneBuilder` (`RAMI > Build Scene from LESS`) and `RamiSceneTiler`
prepare and tile forest geometry. RAMI/LESS source information is available
from [LESS](https://lessrt.org/res_html/rami.html). Existing model and third-party
asset attributions remain with their files.

## Repository contents

This repository contains application source, saved scenes, required assets and
project/package settings. Generated point clouds, output archives, local
diagnostic runners, temporary scenes and manuscript working notes are excluded.
Ordinary implementation comments are retained. Generated results belong outside
version control; the project `.gitignore` covers the standard output locations.

Project code is released under the [MIT License](LICENSE). Third-party assets
retain their accompanying licenses and notices.
