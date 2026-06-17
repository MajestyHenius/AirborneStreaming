using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Text;
using System;

/// <summary>
/// 3.3 用 - 纯匀速直线 (Steady-state cruise baseline)。
///
/// 测什么: 三指标 (PDVR/ISC/CCI) 在恒速直线段的稳态基线值。
///         论文 3.3 "Steady-state cruise" 那段的 XX 来自这里。
/// 运动:   匀速, 无加速, 无振动, 无转弯。单条直线。
/// 参数:   HET09 场景, W=50, cruiseSpeed=5 m/s, 50m AGL (tab:trajectory)。
/// 预期:   PDVR≈0 (无密度漂移), ISC 很小 (间距均匀), CCI≈常数 (近1)。
///         CSV 的 Phase 列全是 "Cruise"。
///
/// 扫描内核与 SPatternScanFlight 完全一致 (照搬, 未改)。
/// 只把"平台运动"换成最简匀速直线。
/// </summary>
public class CruiseScanner : MonoBehaviour
{
    [Header("===== LiDAR (VLP-16 等效, 同 tab:scanner) =====")]
    [SerializeField] private int lines = 16;
    [SerializeField] private float verticalFOV = 30f;
    [SerializeField] private float maxDetectionDistance = 100f;

    [Header("===== 旋转扫描 =====")]
    [SerializeField] private float rotationFrequency = 10f;
    [SerializeField] private float maxScanAngleFromNadir = 75f;
    [SerializeField] private float initialRotationPhase = 0f;
    [SerializeField] private int firingsPerFixedUpdate = 360;
    [SerializeField] private StreamingQualityAssessment qualityAssessor;

    [Header("===== 平台运动 (tab:trajectory) =====")]
    [SerializeField] private float cruiseSpeed = 5f;       // m/s
    [SerializeField] private float targetHeight = 50f;     // AGL, m
    [SerializeField] private float flightLength = 400f;    // 直线长度 m

    [Header("===== 输出 =====")]
    [SerializeField] private string outputPrefix = "Cruise";
    [SerializeField] private bool savePointCloud = false;  // off: metrics only, constant memory
    [SerializeField] private bool saveRangeMatrix = false;  // off by default; metrics dont need it

    // ---- internal ----
    private bool isScanning = false;
    private bool isCompleted = false;
    private Vector3 startPosition;
    private Vector3 flightDirection = Vector3.left;
    private float traveledDistance = 0f;

    private Vector3 prevScanPosition;
    private Quaternion prevScanRotation;
    private bool hasPrevPose = false;
    private int totalPointsCollected = 0;
    private int scanCount = 0;

    private float[] beamSinTheta;
    private float[] beamCosTheta;
    private Vector3[] lastScanOrigins;
    private Vector3[] lastScanHitPoints;
    private bool[] lastScanHasHit;

    private List<List<float>> rangeMatrix = new List<List<float>>();
    private StringBuilder pointCloudData = new StringBuilder();
    private StringBuilder trajectoryData = new StringBuilder();
    private string sessionId;
    private StreamWriter pcWriter;       // streaming point-cloud writer (no in-memory accumulation)
    private StreamWriter trajWriter;     // streaming trajectory writer
    private string outputDirCached;
    private int flushCounter = 0;
    private Rigidbody rb;

    // 模型修正, 与 SPattern 一致: LookRotation(dir,up) * Euler(0,90,0)
    private static readonly Quaternion ModelYawCorrection = Quaternion.Euler(0f, 90f, 0f);
    Quaternion BuildPlatformRotation(Vector3 dir)
    {
        return Quaternion.LookRotation(dir, Vector3.up) * ModelYawCorrection;
    }

    void Start()
    {
        sessionId = DateTime.Now.ToString("yyyyMMdd_HHmmss");

        rb = GetComponent<Rigidbody>();
        if (rb == null) rb = gameObject.AddComponent<Rigidbody>();
        rb.useGravity = false; rb.drag = 0f; rb.angularDrag = 0f;

        CalculateBeamAngles();

        startPosition = new Vector3(flightLength / 2f, targetHeight, 0f);
        transform.position = startPosition;
        flightDirection = Vector3.left;
        transform.rotation = BuildPlatformRotation(flightDirection);
        rb.velocity = flightDirection * cruiseSpeed;

        if (qualityAssessor != null)
        {
            qualityAssessor.MaxScanAngleFromNadir = maxScanAngleFromNadir;
            qualityAssessor.CurrentPhase = "Cruise";
        }

        outputDirCached = Path.Combine(Application.dataPath, "..", "PointClouds", sessionId + "_" + outputPrefix);
        if (!Directory.Exists(outputDirCached)) Directory.CreateDirectory(outputDirCached);
        if (savePointCloud)
            pcWriter = new StreamWriter(Path.Combine(outputDirCached, $"{outputPrefix}_PointCloud_{sessionId}.txt"));
        trajWriter = new StreamWriter(Path.Combine(outputDirCached, $"{outputPrefix}_Trajectory_{sessionId}.txt"));
        if (qualityAssessor != null) qualityAssessor.OpenMetricsStream(outputDirCached);
        trajectoryData.AppendLine("# t px py pz qx qy qz qw");
        isScanning = true;
        Debug.Log($"[Cruise] start, v={cruiseSpeed} h={targetHeight} len={flightLength}");
    }

    void CalculateBeamAngles()
    {
        beamSinTheta = new float[lines];
        beamCosTheta = new float[lines];
        for (int i = 1; i <= lines; i++)
        {
            float thetaDeg = -((2f * i - lines - 1) / (2f * lines)) * verticalFOV;
            float thetaRad = thetaDeg * Mathf.Deg2Rad;
            beamSinTheta[i - 1] = Mathf.Sin(thetaRad);
            beamCosTheta[i - 1] = Mathf.Cos(thetaRad);
        }
    }

    float ComputeOmegaAt(float t)
    {
        float raw = 2f * Mathf.PI * rotationFrequency * t + initialRotationPhase * Mathf.Deg2Rad;
        return Mathf.Repeat(raw + Mathf.PI, 2f * Mathf.PI) - Mathf.PI;
    }

    Vector3 GetLocalBeamDirection(int i, float cosOmega, float sinOmega)
    {
        return new Vector3(beamSinTheta[i], -beamCosTheta[i] * cosOmega, -beamCosTheta[i] * sinOmega);
    }

    float CalculateIntensity(RaycastHit hit, float range)
    {
        float baseIntensity = 100f;
        float distanceAttenuation = Mathf.Exp(-range * 0.01f);
        float incidentAngle = Vector3.Angle(hit.normal, -hit.point.normalized);
        float angleEffect = Mathf.Cos(incidentAngle * Mathf.Deg2Rad);
        return baseIntensity * distanceAttenuation * angleEffect;
    }

    void FixedUpdate()
    {
        if (!isScanning || isCompleted) return;

        // 纯匀速: 速度与姿态恒定
        rb.velocity = flightDirection * cruiseSpeed;
        transform.rotation = BuildPlatformRotation(flightDirection);

        if (qualityAssessor != null) qualityAssessor.CurrentPhase = "Cruise";

        traveledDistance = (transform.position - startPosition).magnitude;
        if (traveledDistance >= flightLength) { CompleteFlight(); return; }

        PerformScanSubstepped();
        if (++flushCounter >= 50) { flushCounter = 0; pcWriter?.Flush(); trajWriter?.Flush(); }
        RecordTrajectorySample();

        prevScanPosition = transform.position;
        prevScanRotation = transform.rotation;
        hasPrevPose = true;
    }

    void PerformScanSubstepped()
    {
        int N = Mathf.Max(1, firingsPerFixedUpdate);
        float dt = Time.fixedDeltaTime;
        float t0 = Time.fixedTime - dt;

        Vector3 startPos = hasPrevPose ? prevScanPosition : transform.position;
        Quaternion startRot = hasPrevPose ? prevScanRotation : transform.rotation;
        Vector3 endPos = transform.position;
        Quaternion endRot = transform.rotation;

        if (lastScanOrigins == null || lastScanOrigins.Length != lines)
        {
            lastScanOrigins = new Vector3[lines];
            lastScanHitPoints = new Vector3[lines];
            lastScanHasHit = new bool[lines];
        }

        float omegaMax = maxScanAngleFromNadir * Mathf.Deg2Rad;

        for (int k = 0; k < N; k++)
        {
            float u = (k + 1f) / N;
            float tSub = t0 + u * dt;
            float omega = ComputeOmegaAt(tSub);
            bool isGroundFacing = Mathf.Abs(omega) <= omegaMax;

            Vector3 scanPosition = Vector3.Lerp(startPos, endPos, u);
            Quaternion scanRotation = Quaternion.Slerp(startRot, endRot, u);

            List<float> currentScanRanges = new List<float>(lines);

            if (isGroundFacing)
            {
                scanCount++;
                float cosOmega = Mathf.Cos(omega);
                float sinOmega = Mathf.Sin(omega);
                for (int i = 0; i < lines; i++)
                {
                    Vector3 localDir = GetLocalBeamDirection(i, cosOmega, sinOmega);
                    Vector3 worldDirection = scanRotation * localDir;
                    RaycastHit hit;
                    bool hasHit = Physics.Raycast(scanPosition, worldDirection, out hit, maxDetectionDistance);
                    float range = hasHit ? hit.distance : -1f;
                    currentScanRanges.Add(range);

                    if (k == N - 1)
                    {
                        lastScanOrigins[i] = scanPosition;
                        lastScanHasHit[i] = hasHit;
                        lastScanHitPoints[i] = hasHit ? hit.point : scanPosition + worldDirection * maxDetectionDistance;
                    }
                    if (hasHit)
                    {
                        Vector3 hitPoint = hit.point;
                        float scanAngleDeg = omega * Mathf.Rad2Deg;
                        float intensity = CalculateIntensity(hit, range);
                        if (savePointCloud)
                        pcWriter.WriteLine(
                            $"{hitPoint.x:F3} {hitPoint.y:F3} {hitPoint.z:F3} " +
                            $"{range:F3} {intensity:F2} {scanAngleDeg:F2}");
                        totalPointsCollected++;
                    }
                }
                if (saveRangeMatrix) rangeMatrix.Add(currentScanRanges);
            }

            if (qualityAssessor != null)
                qualityAssessor.OnFiring(currentScanRanges, scanPosition, omega);
        }
    }

    void RecordTrajectorySample()
    {
        float t = Time.fixedTime;
        Vector3 p = transform.position;
        Quaternion q = transform.rotation;
        trajWriter.WriteLine($"{t:F4} {p.x:F4} {p.y:F4} {p.z:F4} {q.x:F6} {q.y:F6} {q.z:F6} {q.w:F6}");
    }

    void CompleteFlight()
    {
        isCompleted = true;
        rb.velocity = Vector3.zero;
        Debug.Log($"[Cruise] done. pts={totalPointsCollected} sweeps={scanCount}");
        SaveScanData();
#if UNITY_EDITOR
        UnityEditor.EditorApplication.isPlaying = false;   // reach end -> auto stop Play
#endif
    }

    void OnApplicationQuit()
    {
        if (totalPointsCollected > 0 && !isCompleted) SaveScanData();
    }

    void SaveScanData()
    {
        pcWriter?.Flush(); pcWriter?.Close(); pcWriter = null;
        trajWriter?.Flush(); trajWriter?.Close(); trajWriter = null;
        string outputDir = outputDirCached;



        if (saveRangeMatrix)
        using (StreamWriter w = new StreamWriter(Path.Combine(outputDir, $"{outputPrefix}_RangeMatrix_{sessionId}.txt")))
        {
            w.WriteLine($"# Scans: {rangeMatrix.Count}");
            w.WriteLine($"# Lines per scan: {lines}");
            foreach (var scan in rangeMatrix)
                w.WriteLine(string.Join(" ", scan.ConvertAll(r => r.ToString("F3"))));
        }

        if (qualityAssessor != null) qualityAssessor.SaveMetrics(outputDir);
        Debug.Log($"[Cruise] saved to {outputDir}");
#if UNITY_EDITOR
        UnityEditor.AssetDatabase.Refresh();
#endif
    }
}
