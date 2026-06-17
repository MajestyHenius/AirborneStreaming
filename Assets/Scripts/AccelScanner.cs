using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Text;
using System;

/// <summary>
/// 3.3 用 - 纯加速直线 (Acceleration)。
///
/// 测什么: PDVR 对密度漂移的检测, 验证论文 Eq.(density_rate): (1/D)(dD/dt) = -a/v。
///         给定恒定 a, 速度 v(t)=v0+at 上升 -> 点密度下降 -> PDVR 应反映 a/v。
/// 运动:   匀速起步 -> 恒定加速度 a 加速到 targetSpeedAfterAccel -> 保持。直线, 无振动无转弯。
/// 参数:   HET09, W=50, 起步 5 m/s, a=1 m/s² (tab:trajectory), 50m AGL。
///         targetSpeedAfterAccel = 加速上限 (默认 10, 见下方 TODO 确认)。
/// 预期:   加速段 PDVR 明显 > 0 且与 a/v 一致; 到达上限转匀速后 PDVR 回落到≈0。
///         CSV Phase 列: 加速段 "Accel", 到达上限后 "Cruise"。
///
/// 扫描内核与 SPatternScanFlight / CruiseScanner 完全一致 (照搬, 未改)。
/// </summary>
public class AccelScanner : MonoBehaviour
{
    [Header("===== LiDAR (VLP-16 等效) =====")]
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
    [SerializeField] private float initialSpeed = 5f;             // v0, m/s
    [SerializeField] private float acceleration = 1f;            // a, m/s²
    // TODO 确认: 加速上限。论文里应有此值, 用户确认后改。暂填 10。
    [SerializeField] private float targetSpeedAfterAccel = 10f;  // 加速到此速度后保持
    [SerializeField] private float targetHeight = 50f;          // AGL, m
    [SerializeField] private float flightLength = 400f;         // 直线总长 m
    [SerializeField] private float preAccelCruiseLength = 40f;  // 加速前先匀速这么远(留baseline)

    [Header("===== 输出 =====")]
    [SerializeField] private string outputPrefix = "Accel";
    [SerializeField] private bool savePointCloud = false;  // off: metrics only, constant memory
    [SerializeField] private bool saveRangeMatrix = false;  // off by default; metrics dont need it

    private bool isScanning = false;
    private bool isCompleted = false;
    private Vector3 startPosition;
    private Vector3 flightDirection = Vector3.left;
    private float traveledDistance = 0f;
    private float accelTime = 0f;          // 进入加速段后的累计时间 (Eq.14 的 t)
    private bool inAccelPhase = false;
    private bool accelDone = false;

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

    private static readonly Quaternion ModelYawCorrection = Quaternion.Euler(0f, 90f, 0f);
    Quaternion BuildPlatformRotation(Vector3 dir) => Quaternion.LookRotation(dir, Vector3.up) * ModelYawCorrection;

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
        rb.velocity = flightDirection * initialSpeed;

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
        trajectoryData.AppendLine("# t px py pz qx qy qz qw  (Accel run)");
        isScanning = true;
        Debug.Log($"[Accel] start v0={initialSpeed} a={acceleration} target={targetSpeedAfterAccel}");
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
        => new Vector3(beamSinTheta[i], -beamCosTheta[i] * cosOmega, -beamCosTheta[i] * sinOmega);

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

        traveledDistance = (transform.position - startPosition).magnitude;

        // 阶段判定: 先匀速 preAccelCruiseLength, 然后进入加速, 到达目标速度后转匀速
        float currentSpeed;
        if (traveledDistance < preAccelCruiseLength)
        {
            currentSpeed = initialSpeed;
            if (qualityAssessor != null) qualityAssessor.CurrentPhase = "Cruise";
        }
        else
        {
            if (!inAccelPhase && !accelDone) { inAccelPhase = true; accelTime = 0f; }

            if (inAccelPhase)
            {
                accelTime += Time.fixedDeltaTime;
                float raw = initialSpeed + acceleration * accelTime;   // v(t)=v0+at
                if (raw >= targetSpeedAfterAccel)
                {
                    currentSpeed = targetSpeedAfterAccel;
                    inAccelPhase = false; accelDone = true;
                }
                else currentSpeed = raw;
                if (qualityAssessor != null) qualityAssessor.CurrentPhase = accelDone ? "Cruise" : "Accel";
            }
            else
            {
                currentSpeed = targetSpeedAfterAccel;  // 加速完成后保持
                if (qualityAssessor != null) qualityAssessor.CurrentPhase = "Cruise";
            }
        }

        rb.velocity = flightDirection * currentSpeed;
        transform.rotation = BuildPlatformRotation(flightDirection);

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
        Debug.Log($"[Accel] done. pts={totalPointsCollected} sweeps={scanCount}");
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
        Debug.Log($"[Accel] saved to {outputDir}");
#if UNITY_EDITOR
        UnityEditor.AssetDatabase.Refresh();
#endif
    }
}
