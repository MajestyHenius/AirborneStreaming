using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Text;
using System;

/// <summary>
/// 3.3 用 - 纯转弯 (Banking turn)。
///
/// 测什么: CCI/ISC 对 banked turn 的响应 (3.3 banking 重点)。
///         直飞 -> 单次 180° 协调转弯 (roll 注入) -> 直飞。
///         图里能看到进入/退出转弯时 CCI 下降、ISC 变化。
/// 运动:   straight(L) -> turn(180°, R=30, bank≤30°) -> straight(L)。匀速, 无加速无振动。
/// 参数:   HET09, W=50, v=5 m/s, 50m AGL, R=30m, maxBank=30° (tab:trajectory)。
///         前后直飞各 straightLen 米 (默认 50)。
/// 预期:   直飞段三指标稳态; 转弯段 CCI 明显下降 (覆盖变化), ISC 升高 (间距不均);
///         退出转弯后恢复。CSV Phase 列: "Straight" / "Turning"。
///
/// 扫描内核与 SPatternScanFlight 完全一致 (照搬, 未改)。
/// 转弯逻辑搬自 SPattern.ExecuteTurning (协调转弯), 裁成单次 180°。
/// 注意: 此脚本用运动学直接驱动 (非 SPattern 的 AddForce 物理), 以保证单次转弯
///       轨迹干净可复现; bank 角仍按 atan2(v²/(Rg)) 计算并注入姿态, 几何效果一致。
/// </summary>
public class BankingScanner : MonoBehaviour
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
    [SerializeField] private float cruiseSpeed = 5f;       // m/s
    [SerializeField] private float targetHeight = 50f;     // AGL, m
    [SerializeField] private float straightLen = 50f;      // 转弯前后各直飞 m
    [SerializeField] private float turnRadius = 30f;       // R, m
    [SerializeField] private float maxBankAngle = 30f;     // deg
    [SerializeField] private float bankSmoothSpeed = 2f;   // bank 过渡速度 (同 SPattern)

    [Header("===== 输出 =====")]
    [SerializeField] private string outputPrefix = "Banking";
    [SerializeField] private bool savePointCloud = true;  // off: metrics only, constant memory
    [SerializeField] private bool saveRangeMatrix = false;  // off by default; metrics dont need it

    private enum Phase { StraightIn, Turning, StraightOut, Done }
    private Phase phase = Phase.StraightIn;

    private bool isScanning = false;
    private bool isCompleted = false;

    // 直飞段
    private Vector3 flightDirection = Vector3.left;
    private Vector3 segStartPos;
    private float segTraveled = 0f;

    // 转弯段 (协调转弯, 运动学)
    private Vector3 circleCenter;
    private float currentAngle = 0f;      // 圆上角度 rad
    private float totalTurnedAngle = 0f;
    private bool turningRight = true;
    private float currentBankAngle = 0f;

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

    // 直飞姿态: LookRotation(dir) * Euler(0,90,0)
    Quaternion BuildStraightRotation(Vector3 dir) => Quaternion.LookRotation(dir, Vector3.up) * ModelYawCorrection;
    // 转弯姿态: 在切线朝向上叠加 roll(-bank), 再模型修正 (同 SPattern ExecuteTurning)
    Quaternion BuildTurnRotation(Vector3 tangent, float bankDeg)
        => Quaternion.LookRotation(tangent, Vector3.up) * Quaternion.Euler(0, 0, -bankDeg) * ModelYawCorrection;

    void Start()
    {
        sessionId = DateTime.Now.ToString("yyyyMMdd_HHmmss");
        rb = GetComponent<Rigidbody>();
        if (rb == null) rb = gameObject.AddComponent<Rigidbody>();
        rb.useGravity = false; rb.drag = 0f; rb.angularDrag = 0f;
        rb.isKinematic = true;   // 运动学驱动, 直接设 position/rotation

        CalculateBeamAngles();

        // 起点: 让直飞-转弯-直飞大致居中
        transform.position = new Vector3(turnRadius + straightLen, targetHeight, -turnRadius);
        flightDirection = Vector3.left;
        transform.rotation = BuildStraightRotation(flightDirection);
        segStartPos = transform.position;

        if (qualityAssessor != null)
        {
            qualityAssessor.MaxScanAngleFromNadir = maxScanAngleFromNadir;
            qualityAssessor.CurrentPhase = "Straight";
        }

        outputDirCached = Path.Combine(Application.dataPath, "..", "PointClouds", sessionId + "_" + outputPrefix);
        if (!Directory.Exists(outputDirCached)) Directory.CreateDirectory(outputDirCached);
        if (savePointCloud)
            pcWriter = new StreamWriter(Path.Combine(outputDirCached, $"{outputPrefix}_PointCloud_{sessionId}.txt"));
        trajWriter = new StreamWriter(Path.Combine(outputDirCached, $"{outputPrefix}_Trajectory_{sessionId}.txt"));
        if (qualityAssessor != null) qualityAssessor.OpenMetricsStream(outputDirCached);
        trajectoryData.AppendLine("# t px py pz qx qy qz qw  (Banking run)");
        isScanning = true;
        Debug.Log($"[Banking] start v={cruiseSpeed} R={turnRadius} maxBank={maxBankAngle}");
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
        float dt = Time.fixedDeltaTime;

        switch (phase)
        {
            case Phase.StraightIn:
            case Phase.StraightOut:
            {
                Vector3 step = flightDirection * cruiseSpeed * dt;
                transform.position += step;
                transform.rotation = BuildStraightRotation(flightDirection);
                segTraveled += step.magnitude;
                if (qualityAssessor != null) qualityAssessor.CurrentPhase = "Straight";

                if (segTraveled >= straightLen)
                {
                    if (phase == Phase.StraightIn) StartTurn();
                    else { CompleteFlight(); return; }
                }
                break;
            }
            case Phase.Turning:
            {
                float angVel = cruiseSpeed / turnRadius;      // 角速度
                float dAng = angVel * dt;
                // proper U-turn (|..n): angle DECREASES, half-circle bulges forward, exit anti-parallel
                currentAngle -= dAng;
                totalTurnedAngle += dAng;

                // 圆上位置
                Vector3 pos = circleCenter + new Vector3(
                    Mathf.Cos(currentAngle) * turnRadius, 0f, Mathf.Sin(currentAngle) * turnRadius);
                pos.y = targetHeight;
                transform.position = pos;

                // 切线 = 速度方向
                Vector3 tangent = new Vector3(Mathf.Sin(currentAngle), 0, -Mathf.Cos(currentAngle));
                tangent.Normalize();

                // 协调转弯 bank (同 SPattern): atan2(v², R·g), clamp, 平滑过渡
                float targetBank = Mathf.Atan2(cruiseSpeed * cruiseSpeed, turnRadius * Physics.gravity.magnitude) * Mathf.Rad2Deg;
                targetBank = Mathf.Clamp(targetBank, 0, maxBankAngle);
                if (!turningRight) targetBank = -targetBank;
                currentBankAngle = Mathf.Lerp(currentBankAngle, targetBank, dt * bankSmoothSpeed);

                transform.rotation = BuildTurnRotation(tangent, currentBankAngle);
                if (qualityAssessor != null) qualityAssessor.CurrentPhase = "Turning";

                if (totalTurnedAngle >= Mathf.PI - 0.02f)   // 180°
                {
                    flightDirection = tangent;   // 退出转弯沿当前切线
                    phase = Phase.StraightOut;
                    segStartPos = transform.position;
                    segTraveled = 0f;
                    currentBankAngle = 0f;
                }
                break;
            }
        }

        PerformScanSubstepped();
        if (++flushCounter >= 50) { flushCounter = 0; pcWriter?.Flush(); trajWriter?.Flush(); }
        RecordTrajectorySample();

        prevScanPosition = transform.position;
        prevScanRotation = transform.rotation;
        hasPrevPose = true;
    }

    void StartTurn()
    {
        phase = Phase.Turning;
        totalTurnedAngle = 0f;
        currentBankAngle = 0f;
        // 圆心在当前位置的右侧 (turningRight) 垂直于航向方向
        Vector3 right = Vector3.Cross(Vector3.up, flightDirection).normalized; // 航向右侧
        if (!turningRight) right = -right;
        circleCenter = transform.position + right * turnRadius;
        circleCenter.y = targetHeight;
        // 当前在圆上的起始角
        Vector3 toCraft = transform.position - circleCenter;
        currentAngle = Mathf.Atan2(toCraft.z, toCraft.x);
        Debug.Log($"[Banking] enter turn, center={circleCenter}");
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
        phase = Phase.Done;
        Debug.Log($"[Banking] done. pts={totalPointsCollected} sweeps={scanCount}");
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
        Debug.Log($"[Banking] saved to {outputDir}");
#if UNITY_EDITOR
        UnityEditor.AssetDatabase.Refresh();
#endif
    }
}
