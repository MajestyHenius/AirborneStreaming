using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Text;
using System;

/// <summary>
/// 3.2.2 用：匀速直线飞行 + VLP-16 旋转扫描。
/// 不含 banking / acceleration / vibration / multi-strip。
/// 用于和 MUN-FRL Bell412-1 真实直线段做 1:1 几何对比。
/// 
/// 朝向约定与 SPatternScanFlight 完全一致：
///   transform.rotation = LookRotation(flightDir, up) * Euler(0, 90, 0)
/// 这样本地 X = 沿航向, 本地 -Y = nadir, 本地 Z = 横航向。
/// </summary>
public class LineFlightScanner : MonoBehaviour
{
    [Header("===== LiDAR 配置 (VLP-16 等效) =====")]
    [SerializeField] private int lines = 16;
    [SerializeField] private float verticalFOV = 30f;
    [SerializeField] private float maxDetectionDistance = 100f;

    [Header("===== 旋转扫描参数 =====")]
    [SerializeField] private float rotationFrequency = 10f;
    [SerializeField] private float maxScanAngleFromNadir = 75f;
    [SerializeField] private float initialRotationPhase = 0f;
    [SerializeField] private int firingsPerFixedUpdate = 360;

    [Header("===== 平台运动 =====")]
    // 默认值对齐 MUN-FRL Bell412-1 直线段
    [SerializeField] private float cruiseSpeed = 8.5f;          // m/s
    [SerializeField] private float targetHeight = 25f;          // AGL, m
    [SerializeField] private Vector3 startPosition = new Vector3(0, 25, 0);
    [SerializeField] private Vector3 endPosition = new Vector3(83.5f, 25, 0);

    [Header("===== 输出 =====")]
    [SerializeField] private string outputPrefix = "LineFlight";
    [SerializeField] private StreamingQualityAssessment qualityAssessor;

    // ---- internal state ----
    private bool isScanning = false;
    private bool isCompleted = false;
    private Vector3 flightDirection = Vector3.right;
    private float totalDistance = 0f;
    private float traveledDistance = 0f;

    private Vector3 prevScanPosition;
    private Quaternion prevScanRotation;
    private bool hasPrevPose = false;
    private int totalPointsCollected = 0;
    private int scanCount = 0;

    // Beam 角度预计算
    private float[] beamSinTheta;
    private float[] beamCosTheta;

    // 缓存（OnFiring 用）
    private Vector3[] lastScanOrigins;
    private Vector3[] lastScanHitPoints;
    private bool[] lastScanHasHit;

    // 数据存储
    private List<List<float>> rangeMatrix = new List<List<float>>();
    private StringBuilder pointCloudData = new StringBuilder();
    private StringBuilder trajectoryData = new StringBuilder();
    private string sessionId;
    private Rigidbody rb;

    // ===== 关键:模型修正,与 SPatternScanFlight 完全一致 =====
    // SPattern 在 ExecuteStraightFlight 里:
    //   targetRotation = LookRotation(currentFlightDirection, up) * Euler(0,90,0); // 模型修正
    // 这里我们采用同样的方案,保证 LiDAR 本地坐标系语义一致。
    private static readonly Quaternion ModelYawCorrection = Quaternion.Euler(0f, 90f, 0f);

    Quaternion BuildPlatformRotation(Vector3 dir)
    {
        return Quaternion.LookRotation(dir, Vector3.up) * ModelYawCorrection;
    }

    void Start()
    {
        sessionId = DateTime.Now.ToString("yyyyMMdd_HHmmss");

        // Rigidbody (无重力,匀速直线)
        rb = GetComponent<Rigidbody>();
        if (rb == null) rb = gameObject.AddComponent<Rigidbody>();
        rb.useGravity = false;
        rb.drag = 0f;
        rb.angularDrag = 0f;

        CalculateBeamAngles();

        // 初始位置和姿态
        startPosition = new Vector3(startPosition.x, targetHeight, startPosition.z);
        endPosition = new Vector3(endPosition.x, targetHeight, endPosition.z);

        transform.position = startPosition;

        // 飞行方向 = end - start, 归一化
        Vector3 dirVec = endPosition - startPosition;
        totalDistance = dirVec.magnitude;
        flightDirection = dirVec.normalized;

        // 与 SPattern 一致的模型朝向修正:
        //         否则 LiDAR 本地 X/Z 轴被调换,扫描方向会跟航向重合 ===
        transform.rotation = BuildPlatformRotation(flightDirection);

        // 设置匀速飞行 velocity
        rb.velocity = flightDirection * cruiseSpeed;

        // 轨迹文件 header
        trajectoryData.AppendLine($"# LineFlight Trajectory");
        trajectoryData.AppendLine($"# Start: ({startPosition.x:F3}, {startPosition.y:F3}, {startPosition.z:F3})");
        trajectoryData.AppendLine($"# End:   ({endPosition.x:F3}, {endPosition.y:F3}, {endPosition.z:F3})");
        trajectoryData.AppendLine($"# Speed: {cruiseSpeed:F3} m/s");
        trajectoryData.AppendLine($"# Length: {totalDistance:F3} m");
        trajectoryData.AppendLine($"# Sampling: per FixedUpdate (dt = {Time.fixedDeltaTime:F4} s)");
        trajectoryData.AppendLine("# t  px  py  pz  qx  qy  qz  qw");

        isScanning = true;
        Debug.Log($"LineFlight 开始: 从 {startPosition} 飞到 {endPosition}, " +
                  $"速度 {cruiseSpeed} m/s, 总长度 {totalDistance:F2} m, " +
                  $"机体姿态 {transform.rotation.eulerAngles}");
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
        float raw = 2f * Mathf.PI * rotationFrequency * t
                    + initialRotationPhase * Mathf.Deg2Rad;
        return Mathf.Repeat(raw + Mathf.PI, 2f * Mathf.PI) - Mathf.PI;
    }

    Vector3 GetLocalBeamDirection(int i, float cosOmega, float sinOmega)
    {
        // 与 SPattern 一致:本地 X = along-track, 本地 -Y = nadir, 本地 Z = cross-track
        return new Vector3(
            beamSinTheta[i],
            -beamCosTheta[i] * cosOmega,
            -beamCosTheta[i] * sinOmega
        );
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

        // 匀速直线推进,姿态恒定
        rb.velocity = flightDirection * cruiseSpeed;

        // 保持姿态稳定并采用与 SPattern 一致的模型朝向修正。
        transform.rotation = BuildPlatformRotation(flightDirection);

        // 检查是否到达终点
        traveledDistance = (transform.position - startPosition).magnitude;
        if (traveledDistance >= totalDistance)
        {
            CompleteFlight();
            return;
        }

        PerformScanSubstepped();
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
                    bool hasHit = Physics.Raycast(
                        scanPosition, worldDirection, out hit, maxDetectionDistance);

                    float range = hasHit ? hit.distance : -1f;
                    currentScanRanges.Add(range);

                    if (k == N - 1)
                    {
                        lastScanOrigins[i] = scanPosition;
                        lastScanHasHit[i] = hasHit;
                        lastScanHitPoints[i] = hasHit
                            ? hit.point
                            : scanPosition + worldDirection * maxDetectionDistance;
                    }

                    if (hasHit)
                    {
                        Vector3 hitPoint = hit.point;
                        float scanAngleDeg = omega * Mathf.Rad2Deg;
                        float intensity = CalculateIntensity(hit, range);

                        pointCloudData.AppendLine(
                            $"{hitPoint.x:F3} {hitPoint.y:F3} {hitPoint.z:F3} " +
                            $"{range:F3} {intensity:F2} {scanAngleDeg:F2}");

                        totalPointsCollected++;
                    }
                }

                rangeMatrix.Add(currentScanRanges);
            }

            if (qualityAssessor != null)
            {
                qualityAssessor.OnFiring(currentScanRanges, scanPosition, omega);
            }
        }
    }

    void RecordTrajectorySample()
    {
        float t = Time.fixedTime;
        Vector3 p = transform.position;
        Quaternion q = transform.rotation;
        trajectoryData.AppendLine(
            $"{t:F4} {p.x:F4} {p.y:F4} {p.z:F4} " +
            $"{q.x:F6} {q.y:F6} {q.z:F6} {q.w:F6}");
    }

    void CompleteFlight()
    {
        isCompleted = true;
        rb.velocity = Vector3.zero;
        Debug.Log($"LineFlight 完成: 总点数 {totalPointsCollected}, sweep 数 {scanCount}");
        SaveScanData();
    }

    void OnApplicationQuit()
    {
        if (totalPointsCollected > 0 && !isCompleted)
        {
            Debug.Log("提前退出,紧急落盘点云...");
            SaveScanData();
        }
    }

    void SaveScanData()
    {
        string outputDir = Path.Combine(
            Application.dataPath, "..", "PointClouds", sessionId + "_" + outputPrefix);

        if (!Directory.Exists(outputDir))
            Directory.CreateDirectory(outputDir);

        string pointCloudFile = Path.Combine(
            outputDir, $"{outputPrefix}_PointCloud_{sessionId}.txt");
        File.WriteAllText(pointCloudFile, pointCloudData.ToString());

        string matrixFile = Path.Combine(
            outputDir, $"{outputPrefix}_RangeMatrix_{sessionId}.txt");
        SaveRangeMatrix(matrixFile);

        string trajectoryFile = Path.Combine(
            outputDir, $"{outputPrefix}_Trajectory_{sessionId}.txt");
        File.WriteAllText(trajectoryFile, trajectoryData.ToString());

        if (qualityAssessor != null)
            qualityAssessor.SaveMetrics(outputDir);

        Debug.Log($"数据已保存到: {outputDir}");

#if UNITY_EDITOR
        UnityEditor.AssetDatabase.Refresh();
#endif
    }

    void SaveRangeMatrix(string filepath)
    {
        using (StreamWriter writer = new StreamWriter(filepath))
        {
            writer.WriteLine($"# LineFlight Range Matrix");
            writer.WriteLine($"# Scans: {rangeMatrix.Count}");
            writer.WriteLine($"# Lines per scan: {lines}");
            foreach (var scan in rangeMatrix)
            {
                writer.WriteLine(string.Join(" ",
                    scan.ConvertAll(r => r.ToString("F3"))));
            }
        }
    }

    void OnGUI()
    {
        GUI.Label(new Rect(10, 10, 400, 20),
            $"State: {(isCompleted ? "Completed" : "Flying")}");
        GUI.Label(new Rect(10, 30, 400, 20),
            $"Progress: {traveledDistance:F1} / {totalDistance:F1} m");
        GUI.Label(new Rect(10, 50, 400, 20),
            $"Speed: {rb.velocity.magnitude:F2} m/s");
        GUI.Label(new Rect(10, 70, 400, 20),
            $"Altitude: {transform.position.y:F2} m");
        GUI.Label(new Rect(10, 90, 400, 20),
            $"Points: {totalPointsCollected}");
    }
}