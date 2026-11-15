using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Linq;
using System;

public class StreamingQualityAssessment : MonoBehaviour
{

    public string CurrentPhase = "cruise";   // SPattern 每帧更新
    [Header("质量评估参数")]
    public int WindowSize = 50; // 滑动窗口大小（扫描线数）
    [SerializeField] private bool logAlertsToConsole = true;   // false: alerts go to the CSV only (no Editor console echo)
    // --- Performance instrumentation ---
    private long metricTicksThisSweep = 0;
    private long lastSweepMetricTicks = 0;
    public int CalculationStep = 10; // 每10条扫描线计算一次
    public float PDVRThreshold = 0.1f; // PDVR阈值
    public float ISCThreshold = 0.15f; // ISC阈值
    public float CCIThreshold = 0.6f; // CCI阈值


    [Header("几何参数（与 SPatternScanFlight 一致）")]
    public float MaxScanAngleFromNadir = 89f;   // 与主程序同名字段保持一致

    // === O(1) 增量更新状态 ===
    private float prevDensity = -1f;
    private Vector3 prevPosition;
    private bool hasPrevScan = false;

    // PDVR：滚动 r_i 序列 + 累加和
    private Queue<float> rHistory = new Queue<float>();
    private double pdvrSum = 0.0;

    // ISC：滚动 d_i 序列 + 一阶/二阶累加
    private Queue<float> dHistory = new Queue<float>();
    private double sumD = 0.0, sumD2 = 0.0;

    // CCI：滚动 per-scan CCI + 累加
    private Queue<float> cciHistory = new Queue<float>();
    private double sumCCI = 0.0;



    // === Sweep-level accumulator (per ground-facing half-revolution) ===
    //public float MaxScanAngleFromNadir = 89f;   // φ_max in degrees

    // 当前 sweep 的累积状态
    private int curSweepValidHits = 0;          // n_i^pts (论文 Eq.13)
    private int lastPi = 0;
    private float lastRhoI = 0f;
    private int curSweepTotalRays = 0;          // 用于 CCI 的 n_total
    //private List<float> curSweepCrossTrackPos = new List<float>();  // 用于 CCI 横向分bin
    [Header("CCI bins")]
    public int CCIBinCount = 4;            // Q
    public int CCIMinBinFirings = 2;       // τ: active-bin 最小 firing 数
    private int[] curSweepNTotal;          // n_total[k]
    private int[] curSweepNValid;          // n_valid[k]
    private float[] curSweepExpected;     // E[k]: expected pts per bin (tan-projection geometry)

    [Header("CCI with independent expected counts")]
    public bool useAbsoluteExpectation = true;   // true: expected counts from firing geometry only, independent of returns
    public float MaxDetectionDistance = 100f;      // must match the flight script
    public float VerticalFOVDeg = 30f;             // along-track fan angle, must match the flight script
    [Header("CCI validation: injected coverage gaps (off: 0 / -1)")]
    public float testDropoutFraction = 0f;         // drop this fraction of expected returns uniformly
    public int testDropBin = -1;                   // drop every return in this cross-track bin
    public int testSeed = 1;
    private System.Random testRng;


    private Vector3 curSweepFirstFiringPos;     // sweep 起点
    private Vector3 curSweepLastFiringPos;      // sweep 终点
    private float curSweepAltitude = 50f;       // 用于 w_i = 2h tan(φ_max)
    private bool inGroundFacingHalf = false;    // 当前是否在朝地半圈
    private bool prevInGroundFacingHalf = false;

    // 上一个已 commit 的 sweep 信息（用于计算 d_i, r_i）
    private Vector3 prevSweepCentroid;
    private float prevSweepDensity = -1f;
    private bool hasPrevSweep = false;

    // sweep 计数（独立于 scanCounter，scanCounter 仍是 firing 计数兼容旧接口）
    private int sweepCounter = 0;



    // 数据结构
    private Queue<ScanLineData> slidingWindow = new Queue<ScanLineData>();
    private List<QualityMetrics> metricsHistory = new List<QualityMetrics>();
    private StreamWriter metricsWriter;   // streaming CSV writer (no in-memory accumulation)
    private QualityMetrics lastMetric;     // only the latest, for GetLatest()
    private int mCount = 0;
    private double mPdvrSum = 0, mIscSum = 0, mCciSum = 0;
    private float mPdvrMax = 0, mIscMax = 0, mCciMin = 1f;
    private int mPdvrOver = 0, mIscOver = 0, mCciUnder = 0;
    private int scanCounter = 0;
    private string sessionId;

    // 扫描线数据结构
    private class ScanLineData
    {
        public int scanIndex;
        public float timestamp;
        public Vector3 position;
        public List<float> ranges;
        public int validHits;
        public float velocity;

        public ScanLineData(int index, float time, Vector3 pos, List<float> r, float vel)
        {
            scanIndex = index;
            timestamp = time;
            position = pos;
            ranges = new List<float>(r);
            validHits = r.Count(x => x > 0 && x < float.MaxValue);
            velocity = vel;
        }
    }

    // 质量指标结构
    private class QualityMetrics
    {
        public int scanIndex;
        public float timestamp;
        public float PDVR;
        public float ISC;
        public float CCI;
        public string warning;
        public string phase;
        public int Pi;
        public float rhoI;

        public string ToCSV()
        {
            return $"{scanIndex},{timestamp:F3},{PDVR:F4},{ISC:F4},{CCI:F4},\"{warning}\",{phase},{Pi},{rhoI:F4}";
        }
    }

    void Start()
    {
        sessionId = DateTime.Now.ToString("yyyyMMdd_HHmmss");
    }



    [Header("Revision 2 timing (off by default)")]
    public bool enableSweepTiming = false;
    private SweepTimingRecorder sweepTiming;

    public void OnFiring(List<float> ranges, Vector3 position, float omega,
        double firingSimTime = double.NaN, long firingWallTick = 0)
    {
        long _t0 = System.Diagnostics.Stopwatch.GetTimestamp();
        float omegaMax = MaxScanAngleFromNadir * Mathf.Deg2Rad;
        inGroundFacingHalf = Mathf.Abs(omega) <= omegaMax;

        bool traceTiming = enableSweepTiming && !double.IsNaN(firingSimTime) && firingWallTick > 0;
        if (traceTiming && sweepTiming == null)
            sweepTiming = new SweepTimingRecorder();
        if (traceTiming && inGroundFacingHalf && !prevInGroundFacingHalf)
            sweepTiming.Begin(firingSimTime, firingWallTick);
        bool traceCommit = traceTiming && !inGroundFacingHalf && prevInGroundFacingHalf;

        // 1. 进入朝地半圈：开新 sweep
        if (inGroundFacingHalf && !prevInGroundFacingHalf)
        {
            curSweepValidHits = 0;
            curSweepTotalRays = 0;
            if (curSweepNTotal == null || curSweepNTotal.Length != CCIBinCount)
            {
                curSweepNTotal = new int[CCIBinCount];
                curSweepNValid = new int[CCIBinCount];
                curSweepExpected = new float[CCIBinCount];
            }
            else
            {
                System.Array.Clear(curSweepNTotal, 0, CCIBinCount);
                System.Array.Clear(curSweepNValid, 0, CCIBinCount);
                System.Array.Clear(curSweepExpected, 0, CCIBinCount);
            }
            curSweepFirstFiringPos = position;
            curSweepAltitude = Mathf.Max(0.1f, position.y);
        }

        // 2. 在朝地半圈中：按 bin 累积 n_total / n_valid
        if (inGroundFacingHalf)
        {
            // cross-track bin 用 sweep-locked altitude 的固定 extent
            if (!useAbsoluteExpectation)
            {
                float swathHalf = curSweepAltitude * Mathf.Tan(omegaMax);
                if (swathHalf > 1e-6f)
                {
                    float crossTrackPos = curSweepAltitude * Mathf.Tan(omega);
                    int binIdx = Mathf.Clamp(
                        Mathf.FloorToInt((crossTrackPos + swathHalf) / (2f * swathHalf) * CCIBinCount),
                        0, CCIBinCount - 1);
                    int nRays = ranges.Count;
                    curSweepTotalRays += nRays;
                    foreach (var r in ranges)
                        if (r > 0f && r < float.MaxValue) { curSweepNValid[binIdx]++; curSweepValidHits++; }
                    // E[bin]: expected geometric weight, tan-projection density 1/(1+(y/h)^2)
                    float yk = crossTrackPos; float h = curSweepAltitude;
                    float wGeom = 1f / (1f + (yk * yk) / (h * h));
                    curSweepExpected[binIdx] += wGeom * nRays;
                }
            }
            else
            {
                int nRaysAbs = ranges.Count;
                curSweepTotalRays += nRaysAbs;          // unchanged point counting (feeds PDVR)
                foreach (var r in ranges)
                    if (r > 0f && r < float.MaxValue) curSweepValidHits++;
                AccumulateCCIAbsolute(ranges, omega);
            }
            curSweepLastFiringPos = position;
        }

        // 3. 离开朝地半圈：commit sweep
        if (traceTiming && inGroundFacingHalf)
            sweepTiming.AddAssessmentTicks(System.Diagnostics.Stopwatch.GetTimestamp() - _t0);
        if (!inGroundFacingHalf && prevInGroundFacingHalf)
        {
            if (traceCommit) sweepTiming.StartCommit(firingSimTime);
            CommitCurrentSweep();
        }

        prevInGroundFacingHalf = inGroundFacingHalf;
        metricTicksThisSweep += System.Diagnostics.Stopwatch.GetTimestamp() - _t0;
        // Separate diagnostic output, deliberately outside legacy timing.
        if (traceCommit) sweepTiming.WriteCompleted(sweepCounter, WindowSize, CurrentPhase);
    }



    // perf: metric-update time of the most recently committed sweep, in milliseconds.
    // This is the streaming overhead (ring buffer + incremental Delta updates),
    // excluding raycasting. Should be sub-millisecond and independent of W.
    public double LastSweepMetricMs =>
        lastSweepMetricTicks * 1000.0 / System.Diagnostics.Stopwatch.Frequency;

    private void CommitCurrentSweep()
    {
        sweepCounter++;
        // perf: freeze this sweep's accumulated metric time, reset for next sweep
        lastSweepMetricTicks = metricTicksThisSweep;
        metricTicksThisSweep = 0;
        int W = Mathf.Max(2, WindowSize);

        // === sweep centroid (水平投影) ===
        Vector3 centroid = (curSweepFirstFiringPos + curSweepLastFiringPos) * 0.5f;
        centroid.y = 0f;  // 水平投影，去除高度抖动

        // === w_i: 几何 swath 宽度 (论文 Eq.3) ===
        float angularWidth = 2f * curSweepAltitude * Mathf.Tan(MaxScanAngleFromNadir * Mathf.Deg2Rad);
        float rangeWidth = 2f * Mathf.Sqrt(Mathf.Max(0f,
            MaxDetectionDistance * MaxDetectionDistance - curSweepAltitude * curSweepAltitude));
        float wi = Mathf.Min(angularWidth, rangeWidth);

        if (hasPrevSweep)
        {
            // === d_i: 相邻 sweep nadir centroid 沿航迹距离 (论文 Eq.13) ===
            float di = Vector3.Distance(centroid, prevSweepCentroid);

            // === ρ_i: per-sweep density (论文 Eq.13) ===
            float rhoI = (wi > 1e-6f && di > 1e-6f)
                ? curSweepValidHits / (wi * di)
                : 0f;

            // --- PDVR 增量 (论文 Eq.15-16) ---
            if (prevSweepDensity > 1e-9f)
            {
                float r = Mathf.Abs(rhoI - prevSweepDensity) / prevSweepDensity;
                rHistory.Enqueue(r);
                pdvrSum += r;
                if (rHistory.Count > W - 1)
                    pdvrSum -= rHistory.Dequeue();
            }

            // --- ISC 增量 (论文 Eq.17-18) ---
            dHistory.Enqueue(di);
            sumD += di;
            sumD2 += (double)di * di;
            if (dHistory.Count > W - 1)
            {
                float dEv = dHistory.Dequeue();
                sumD -= dEv;
                sumD2 -= (double)dEv * dEv;
            }

            lastPi = curSweepValidHits; lastRhoI = rhoI;
            prevSweepDensity = rhoI;
        }
        else
        {
            prevSweepDensity = 0f;  // 首个 sweep 还无法算 d_i / ρ_i
        }

        // --- CCI 增量 (论文 Eq.19-20) ---
        float cciI = ComputeSweepCCI();
        cciHistory.Enqueue(cciI);
        sumCCI += cciI;
        if (cciHistory.Count > W)
            sumCCI -= cciHistory.Dequeue();

        prevSweepCentroid = centroid;
        hasPrevSweep = true;


        if (sweepCounter == 100)
        {
            float sweepDuration = 0.5f / (MaxScanAngleFromNadir / 90f);  // 半圈耗时近似
                                                                         // 更准确：直接用 firing 数推
            int firingsThisSweep = curSweepTotalRays / 16;  // 每 firing 16 ray
            Debug.Log($"[3.2.1 STATS] " +
                      $"pts/sweep={curSweepTotalRays} " +
                      $"firings/sweep={firingsThisSweep} " +
                      $"(half-rev)");
        }

        // 每 commit 一个 sweep 都输出（不再用 CalculationStep 节流）
        EmitCurrentMetricsSweepLevel();
    }


    private float ComputeSweepCCI()
    {
        if (useAbsoluteExpectation) return ComputeSweepCCIAbsolute();
        // Method A: h_k = N_k / E_k (actual / expected). Nominal -> ~1; gaps -> drop.
        if (curSweepNValid == null || curSweepExpected == null) return 0f;
        int sumN_all = 0; float sumE_all = 0f;
        for (int k = 0; k < CCIBinCount; k++) { sumN_all += curSweepNValid[k]; sumE_all += curSweepExpected[k]; }
        if (sumN_all == 0 || sumE_all < 1e-6f) return 0f;
        float eScale = sumN_all / sumE_all;
        var hRatios = new List<float>(CCIBinCount);
        int sumN_act = 0; float sumE_act = 0f;
        for (int k = 0; k < CCIBinCount; k++)
        {
            float Ek = curSweepExpected[k] * eScale;
            if (Ek >= CCIMinBinFirings) { hRatios.Add(curSweepNValid[k] / Ek); sumN_act += curSweepNValid[k]; sumE_act += Ek; }
        }
        if (hRatios.Count < 2 || sumE_act < 1e-6f)
            return Mathf.Clamp01((float)sumN_all / (sumE_all * eScale));
        float globalRatio = Mathf.Clamp01(sumN_act / sumE_act);
        float mu = 0f; for (int i = 0; i < hRatios.Count; i++) mu += hRatios[i]; mu /= hRatios.Count;
        if (mu < 1e-6f) return 0f;
        float var = 0f; for (int i = 0; i < hRatios.Count; i++) { float dx = hRatios[i]-mu; var += dx*dx; }
        var /= hRatios.Count; float sigma = Mathf.Sqrt(var);
        float uniformity = Mathf.Max(0f, 1f - sigma / mu);
        return globalRatio * uniformity;
    }


    // Expected counts from the firing geometry only. A channel is expected to return if
    // its flat-ground slant range h / (cos(omega) cos(theta_i)) is within MaxDetectionDistance;
    // returned and expected counts are taken over this same set of rays. Bins span the
    // range-limited swath +-sqrt(Rmax^2 - h^2) instead of h*tan(omegaMax).
    private void AccumulateCCIAbsolute(List<float> ranges, float omega)
    {
        float h = curSweepAltitude;
        float R = MaxDetectionDistance;
        if (h <= 0f || h >= R) return;
        float cosO = Mathf.Cos(omega);
        if (cosO <= 1e-6f) return;
        float swathHalf = Mathf.Sqrt(R * R - h * h);
        float y = h * Mathf.Tan(omega);
        if (Mathf.Abs(y) > swathHalf) return;
        int binIdx = Mathf.Clamp(
            Mathf.FloorToInt((y + swathHalf) / (2f * swathHalf) * CCIBinCount),
            0, CCIBinCount - 1);
        int n = ranges.Count;
        float fov = VerticalFOVDeg * Mathf.Deg2Rad;
        if (testRng == null) testRng = new System.Random(testSeed);
        for (int j = 0; j < n; j++)
        {
            // same beam-angle formula as the flight script (channel index j + 1)
            float theta = -((2f * (j + 1) - n - 1) / (2f * n)) * fov;
            float slant = h / (cosO * Mathf.Cos(theta));
            if (slant > R) continue;
            curSweepExpected[binIdx] += 1f;
            float r = ranges[j];
            bool returned = r > 0f && r < float.MaxValue;
            if (returned && binIdx == testDropBin) returned = false;
            if (returned && testDropoutFraction > 0f && testRng.NextDouble() < testDropoutFraction) returned = false;
            if (returned) curSweepNValid[binIdx]++;
        }
    }

    // No rescaling of expected counts. g = received / expected over the active bins,
    // so a uniform loss of returns lowers g. Fewer than two active bins returns g (not 1).
    private float ComputeSweepCCIAbsolute()
    {
        if (curSweepNValid == null || curSweepExpected == null) return 0f;
        var hRatios = new List<float>(CCIBinCount);
        float sumN = 0f, sumE = 0f;
        for (int k = 0; k < CCIBinCount; k++)
        {
            float Ek = curSweepExpected[k];
            if (Ek >= CCIMinBinFirings) { hRatios.Add(curSweepNValid[k] / Ek); sumN += curSweepNValid[k]; sumE += Ek; }
        }
        if (sumE < 1e-6f) return 0f;
        float g = Mathf.Clamp01(sumN / sumE);
        if (hRatios.Count < 2) return g;
        float mu = 0f; for (int i = 0; i < hRatios.Count; i++) mu += hRatios[i]; mu /= hRatios.Count;
        if (mu < 1e-6f) return 0f;
        float var = 0f; for (int i = 0; i < hRatios.Count; i++) { float dx = hRatios[i] - mu; var += dx * dx; }
        var /= hRatios.Count;
        float uniformity = Mathf.Max(0f, 1f - Mathf.Sqrt(var) / mu);
        return g * uniformity;
    }

    private void EmitCurrentMetricsSweepLevel()
    {
        int nR = rHistory.Count;
        int nD = dHistory.Count;
        int nC = cciHistory.Count;

        QualityMetrics m = new QualityMetrics
        {
            scanIndex = sweepCounter,           // 这里用 sweep index 而非 firing index
            timestamp = Time.fixedTime,
            PDVR = nR > 0 ? (float)(pdvrSum / nR) : 0f,
            ISC = ComputeISC(nD),
            CCI = nC > 0 ? (float)(sumCCI / nC) : 0f,
            phase = CurrentPhase,
            Pi = lastPi,
            rhoI = lastRhoI,
        };
        m.warning = GenerateWarning(m);
        // Result is computed here; metric CSV, console and diagnostic I/O follow.
        if (enableSweepTiming && sweepTiming != null) sweepTiming.ResultReady();
        WriteMetricLine(m);

        if (logAlertsToConsole && !string.IsNullOrEmpty(m.warning))
            Debug.LogWarning($"[Quality Alert at sweep {sweepCounter}] {m.warning}");
    }





    // 主要接口：添加新的扫描线
    public void AddScanLine(List<float> ranges, Vector3 position, float velocity)
    {
        scanCounter++;
        ScanLineData newScan = new ScanLineData(
            scanCounter, Time.fixedTime, position, ranges, velocity);

        int W = Mathf.Max(2, WindowSize);

        if (hasPrevScan)
        {
            // --- PDVR 增量（Eq.(13)）---
            float currDensity = CalculatePointDensity(newScan, prevPosition);
            if (prevDensity > 1e-9f)
            {
                float r = Mathf.Abs(currDensity - prevDensity) / prevDensity;
                rHistory.Enqueue(r);
                pdvrSum += r;
                if (rHistory.Count > W - 1)
                    pdvrSum -= rHistory.Dequeue();
            }
            prevDensity = currDensity;

            // --- ISC 增量（Eq.(15)(16)）---
            float d = Vector3.Distance(position, prevPosition);
            dHistory.Enqueue(d);
            sumD += d;
            sumD2 += (double)d * d;
            if (dHistory.Count > W - 1)
            {
                float dEv = dHistory.Dequeue();
                sumD -= dEv;
                sumD2 -= (double)dEv * dEv;
            }
        }
        else
        {
            // 首条扫描线：仅初始化 prevDensity，不能算 r_i
            prevDensity = CalculatePointDensity(newScan, position); // d=0，会得到 0
        }

        // --- CCI 增量（Eq.(20)）---
        float scanCCI = ComputeScanCCI(newScan);
        cciHistory.Enqueue(scanCCI);
        sumCCI += scanCCI;
        if (cciHistory.Count > W)
            sumCCI -= cciHistory.Dequeue();

        prevPosition = position;
        hasPrevScan = true;

        // 仍保留 slidingWindow 供调试/导出（不再用于重算）
        slidingWindow.Enqueue(newScan);
        while (slidingWindow.Count > WindowSize) slidingWindow.Dequeue();

        if (scanCounter % CalculationStep == 0 && scanCounter >= 10)
            EmitCurrentMetrics();
    }

    // 计算所有质量指标
    private void EmitCurrentMetrics()
    {
        int nR = rHistory.Count;
        int nD = dHistory.Count;
        int nC = cciHistory.Count;

        QualityMetrics m = new QualityMetrics
        {
            scanIndex = scanCounter,
            timestamp = Time.fixedTime,
            PDVR = nR > 0 ? (float)(pdvrSum / nR) : 0f,
            ISC = ComputeISC(nD),
            CCI = nC > 0 ? (float)(sumCCI / nC) : 0f,
            phase = CurrentPhase,
        };
        m.warning = GenerateWarning(m);
        WriteMetricLine(m);

        if (logAlertsToConsole && !string.IsNullOrEmpty(m.warning))
            Debug.LogWarning($"[Quality Alert at scan {scanCounter}] {m.warning}");
    }

    private float ComputeISC(int n)
    {
        if (n <= 0) return 0f;
        double mean = sumD / n;
        if (mean <= 1e-9) return 0f;
        double var = sumD2 / n - mean * mean;
        if (var < 0) var = 0;
        return (float)(Math.Sqrt(var) / mean);
    }

    private float ComputeScanCCI(ScanLineData scan)
    {
        if (scan.ranges.Count == 0) return 0f;
        float hitRate = (float)scan.validHits / scan.ranges.Count;
        float lateral = CalculateLateralUniformity(scan.ranges);
        return hitRate * lateral;
    }

    // 辅助函数：计算点密度
    // 论文 Eq.(11): ρ_i = n_i^pts / (w_i * d_i)
    private float CalculatePointDensity(ScanLineData scan, Vector3 prevPos)
    {
        // w_i：当前高度下的几何扫幅，实时反映 ε_z 引起的局部扰动
        float h = Mathf.Max(0.1f, scan.position.y);
        float w = 2f * h * Mathf.Tan(MaxScanAngleFromNadir * Mathf.Deg2Rad);

        // d_i：与上一条扫描线的真实三维距离（与 ISC 一致）
        float d = Vector3.Distance(scan.position, prevPos);
        if (d < 1e-6f || w < 1e-6f) return 0f;

        return scan.validHits / (w * d);
    }

    // 辅助函数：计算横向分布均匀性
    private float CalculateLateralUniformity(List<float> ranges)
    {
        if (ranges.Count < 4) return 1.0f;

        // 将射线分成4个区域，检查每个区域的有效击中
        int sectorsCount = 4;
        int raysPerSector = ranges.Count / sectorsCount;
        List<int> sectorHits = new List<int>();

        for (int i = 0; i < sectorsCount; i++)
        {
            int hits = 0;
            int startIdx = i * raysPerSector;
            int endIdx = (i == sectorsCount - 1) ? ranges.Count : (i + 1) * raysPerSector;

            for (int j = startIdx; j < endIdx; j++)
            {
                if (ranges[j] > 0 && ranges[j] < float.MaxValue)
                    hits++;
            }

            sectorHits.Add(hits);
        }

        // 计算均匀性
        float mean = (float)sectorHits.Average();
        if (mean == 0) return 0;

        float std = Mathf.Sqrt(sectorHits.Select(x => (x - mean) * (x - mean)).Average());
        return 1.0f - (std / mean);
    }

    // 生成警告信息
    private string GenerateWarning(QualityMetrics metrics)
    {
        List<string> warnings = new List<string>();

        if (metrics.PDVR > PDVRThreshold)
        {
            warnings.Add($"密度变化过大(PDVR={metrics.PDVR:F3})");
        }

        if (metrics.ISC > ISCThreshold)
        {
            warnings.Add($"扫描间距不一致(ISC={metrics.ISC:F3})");
        }

        if (metrics.CCI < CCIThreshold)
        {
            warnings.Add($"覆盖质量不足(CCI={metrics.CCI:F3})");
        }

        return string.Join("; ", warnings);
    }

    // 保存质量评估结果
    // Open streaming metrics CSV. Call once at scan start.
    public void OpenMetricsStream(string outputDir)
    {
        if (!Directory.Exists(outputDir)) Directory.CreateDirectory(outputDir);
        string f = Path.Combine(outputDir, $"QualityMetrics_{sessionId}.csv");
        metricsWriter = new StreamWriter(f);
        metricsWriter.WriteLine("ScanIndex,Timestamp,PDVR,ISC,CCI,Warning,Phase,Pi,RhoI");
    }

    // Write one metric row immediately + update accumulators. No List growth.
    private void WriteMetricLine(QualityMetrics m)
    {
        lastMetric = m;
        mCount++;
        mPdvrSum += m.PDVR; mIscSum += m.ISC; mCciSum += m.CCI;
        if (m.PDVR > mPdvrMax) mPdvrMax = m.PDVR;
        if (m.ISC > mIscMax) mIscMax = m.ISC;
        if (m.CCI < mCciMin) mCciMin = m.CCI;
        if (m.PDVR > PDVRThreshold) mPdvrOver++;
        if (m.ISC > ISCThreshold) mIscOver++;
        if (m.CCI < CCIThreshold) mCciUnder++;
        if (metricsWriter != null)
        {
            metricsWriter.WriteLine(m.ToCSV());
            if ((mCount & 63) == 0) metricsWriter.Flush();
        }
    }


    public void SaveMetrics(string outputDir)
    {
        // CSV already written streaming; just close it.
        if (metricsWriter != null) { metricsWriter.Flush(); metricsWriter.Close(); metricsWriter = null; }
        if (mCount == 0) return;

        string summaryFile = Path.Combine(outputDir, $"QualitySummary_{sessionId}.txt");
        using (StreamWriter writer = new StreamWriter(summaryFile))
        {
            writer.WriteLine("=== Quality summary ===");
            writer.WriteLine($"Total sweeps: {scanCounter}");
            writer.WriteLine($"Metric count: {mCount}");
            writer.WriteLine("\n--- PDVR ---");
            writer.WriteLine($"mean: {mPdvrSum / mCount:F4}");
            writer.WriteLine($"max:  {mPdvrMax:F4}");
            writer.WriteLine($"over threshold: {mPdvrOver}");
            writer.WriteLine("\n--- ISC ---");
            writer.WriteLine($"mean: {mIscSum / mCount:F4}");
            writer.WriteLine($"max:  {mIscMax:F4}");
            writer.WriteLine($"over threshold: {mIscOver}");
            writer.WriteLine("\n--- CCI ---");
            writer.WriteLine($"mean: {mCciSum / mCount:F4}");
            writer.WriteLine($"min:  {mCciMin:F4}");
            writer.WriteLine($"under threshold: {mCciUnder}");
        }
        Debug.Log($"[QA] metrics saved to {outputDir}");
    }





    // 清理资源
    public void Clear()
    {
        slidingWindow.Clear();
        metricsHistory.Clear();
        scanCounter = 0;
        rHistory.Clear(); dHistory.Clear(); cciHistory.Clear();
        pdvrSum = sumD = sumD2 = sumCCI = 0;
        prevDensity = -1f; hasPrevScan = false;

        // sweep-level state reset
        sweepCounter = 0;
        curSweepValidHits = 0;
        curSweepTotalRays = 0;
        //curSweepCrossTrackPos.Clear();
        inGroundFacingHalf = false;
        prevInGroundFacingHalf = false;
        prevSweepDensity = -1f;
        hasPrevSweep = false;
        if (curSweepNTotal != null)
        {
            System.Array.Clear(curSweepNTotal, 0, curSweepNTotal.Length);
            System.Array.Clear(curSweepNValid, 0, curSweepNValid.Length);
        }
    }


    private void OnDisable()
    {
        if (sweepTiming != null) { sweepTiming.Dispose(); sweepTiming = null; }
    }

    // 供实时显示读取最近的指标
    public struct MetricSnapshot { public float pdvr, isc, cci; }

    public MetricSnapshot GetLatestMetrics()
    {
        if (mCount == 0)
            return new MetricSnapshot { pdvr = 0, isc = 0, cci = 0 };
        var m = lastMetric;
        return new MetricSnapshot { pdvr = m.PDVR, isc = m.ISC, cci = m.CCI };
    }

}
