using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Text;
using System;
public class SPatternScanFlightPhasedLoop : MonoBehaviour
{
    [Header("===== LiDAR配置 (VLP-16 等效) =====")]
    // Idealised VLP-16 configuration: 16 channels, 30-degree fan, 10 Hz, 28,800 nominal firings/revolution, 100 m range.
    [SerializeField] private int lines = 16;                        // n: 通道数
    [SerializeField] private float verticalFOV = 30f;               // Θ: 沿航向扇面 (deg), VLP-16 = ±15°
    [SerializeField] private float maxDetectionDistance = 100f;     // VLP-16 max range
    [Header("===== 等效旋转扫描参数 =====")]
    [SerializeField] private float rotationFrequency = 10f;          // 真机值 10 Hz
    [SerializeField] private float maxScanAngleFromNadir = 89f;
    [SerializeField] private float initialRotationPhase = 0f;
    // 每个 FixedUpdate 内 firing 的次数
    // 50 Hz × 360 = 18 kHz，对齐 VLP-16 真机重频
    // ω/f = 2π·10 / 18000 = 0.2°/firing 严格等效
    [SerializeField] private int firingsPerFixedUpdate = 360;
    [SerializeField] private StreamingQualityAssessment qualityAssessor;

    [Header("===== 平台运动 =====")]
    [SerializeField] private float targetHeight = 50f;              // AGL altitude, m
    [SerializeField] private float cruiseSpeed = 5f;                // m/s, VLP-16 UAV typical

    [Header("===== 平台运动扰动 (论文 Eq.14) =====")]
    [SerializeField] private float cruiseAcceleration = 0f;     // a, m/s²，沿航向恒定加速度（0 = 关闭）
    [SerializeField] private float targetCruiseSpeedAfterAccel = 5f;  // 加/减速达到此速度后保持
    [SerializeField] private float vibrationSigma = 0f;         // σ_v, m/s，ε_y/ε_z 的标准差（0 = 关闭）
    [SerializeField] private bool resetAccelPerSwath = true;    // 是否每条航线起点重置 t



    [Header("===== S型扫描参数 =====")]
    [SerializeField] private float swathWidth = 20f; // 相邻航线间距
    [SerializeField] private int numberOfSwaths = 5; // 航线数量
    [SerializeField] private float swathLength = 400f; // 每条航线长度

    [Header("===== 转弯参数 =====")]
    [SerializeField] private float turnRadius = 30f; // 转弯半径
    [SerializeField] private float liftMultiplier = 1.05f; // 升力倍数
    [SerializeField] private float maxBankAngle = 30f; // 最大倾斜角
    [SerializeField] private float bankSmoothSpeed = 2f; // 倾斜过渡速度

    [Header("===== 运行时状态 =====")]
    [SerializeField] private bool isScanning = false;
    [SerializeField] private int currentSwathIndex = 0;

    [Header("===== 长时程性能测试 ====="  )]
    [SerializeField] private bool  loopSurvey = false;        // true: 到端点折返, 无限往复
    [SerializeField] private float maxMissionSeconds = 0f;    // >0: 按仿真时长结束; 0: 按条带数结束
    private int swathStep = +1;                               // +1 / -1
    [SerializeField] private FlightState currentState = FlightState.StraightFlight;
    private bool isAccelerating = false;  // true while still ramping toward target cruise speed (for Phase tagging)
    [SerializeField] private int totalPointsCollected = 0;
    [SerializeField] private float distanceInCurrentSwath = 0f;


    [Header("===== 点云实时可视化 =====")]
    [SerializeField] private bool showInWorld = true;  // 在世界中显示
    [SerializeField] private bool showInUI = true;     // 在UI面板中显示
    [SerializeField] private bool enableRealtimeVisualization = true;
    [SerializeField] private int maxDisplayPoints = 70000;  // 最大显示点数
    [SerializeField] private float pointDisplaySize = 0.2f;  // 点的显示大小
    [SerializeField] private float minHeightColor = 0f;  // 颜色映射的最低高度
    [SerializeField] private float maxHeightColor = 150f;  // 颜色映射的最高高度
    [SerializeField] private Gradient heightGradient;// 高度颜色梯度
    [SerializeField] private Gradient intensityGradient; // 强度颜色梯度

    [SerializeField] private float minDistance = 44f;      // 距离颜色映射的最小值
    [SerializeField] private float maxDistance = 140f;    // 距离颜色映射的最大值
    [SerializeField] private Gradient distanceGradient;  // 距离专用颜色梯度
    [SerializeField] private ColorMode colorMode = ColorMode.Height;
    [SerializeField] private float minIntensity = 0f;
    [SerializeField] private float maxIntensity = 100f;

    [SerializeField] private float actualPointSize = 0.5f;  // 实际点大小（世界空间）
    [SerializeField] private float uiPointSize = 2f;      // UI中的点大小
    [SerializeField] private bool uiAutoRotate = false;  // UI中点云自动旋转
    [SerializeField] private float uiRotateSpeed = 10f; // 旋转速度
    [SerializeField] private Vector2 uiPanelSize = new Vector2(400, 400); // UI面板大小
    [SerializeField] private Vector2 uiPanelPosition = new Vector2(20, 800); // UI面板位置
    [SerializeField] private float uiPointScale = 1f;  // UI中点的缩放比例
    [SerializeField] private float uiViewDistance = 200f; // UI视图距离
    [SerializeField] private float uiCameraFOV = 90f;      // 视野角度


    private Vector3 prevScanPosition;
    private Quaternion prevScanRotation;
    private bool hasPrevPose = false;
    private enum FlightState
    {
        StraightFlight,
        Turning,
        Completed
    }
    private enum ColorMode
    {
        Height,
        Intensity,
        Distance
    }
    private struct PointData
    {
        public Vector3 position;
        public float intensity;
        public float distance;
    }
    // 转弯状态
    private float currentAngle = 0f; // 当前在圆上的角度 (rad)
    private float currentBankAngle = 0f; // 当前实际倾斜角度
    private Vector3 circleCenter; // 当前转弯圆心
    private bool turningRight = false; // 转弯方向
    private float turnStartAngle = 0f; // 转弯开始时的角度
    private float totalTurnedAngle = 0f; // 已转过的总角度

    // 飞行控制
    private Vector3 currentFlightDirection = Vector3.left; // 当前飞行方向（世界坐标）
    private bool isFirstSwath = true;
    private float swathTime = 0f;   // 论文 Eq.(14) 中的 t，仅在直飞段累计
    // LiDAR相关
    private LineRenderer[] lineRendArray;
    private Vector3[] laserDirections;
    private Rigidbody rb;

    // 数据存储
    private List<List<float>> rangeMatrix = new List<List<float>>();
    private StringBuilder pointCloudData = new StringBuilder();
    // 论文 2.2 节 s_t 的位姿部分：每 FixedUpdate 一行
    private StringBuilder trajectoryData = new StringBuilder();
    private int fixedStepCounter = 0;

    private string sessionId;
    // --- Performance instrumentation ---
    [SerializeField] private bool savePointCloud = false;  // Disable to avoid retaining point-cloud text for export.
    [SerializeField] private bool saveRangeMatrix = false;
    [SerializeField] private bool saveTrajectory = false;
    [SerializeField] private bool enablePointDisplay = false;  // off for perf/timeseries -> no allPointsData growth
    private StreamWriter perfWriter;
    private int perfStepCounter = 0;
    private float lastScanTime;
    private System.Diagnostics.Stopwatch wallClock = new System.Diagnostics.Stopwatch();
    private double liveMB = 0.0;        // GC.GetTotalMemory(true), refreshed every LIVE_EVERY steps
    private long   cumPoints = 0;      // running snapshot of totalPointsCollected
    private int    lateSteps = 0;
    private const int LIVE_EVERY = 1000;  // forcing a collection is expensive; sample sparsely
    private int scanCount = 0;

    // 缓存射线信息
    private Vector3[] lastScanOrigins;
    private Vector3[] lastScanHitPoints;
    private bool[] lastScanHasHit;
    private bool hasNewScanData = false;

    // 点云显示组件
    private ParticleSystem pointCloudDisplay;
    private ParticleSystem.Particle[] displayParticles;
    private int currentDisplayPointCount = 0;
    private ParticleSystemRenderer particleRenderer;
    private RenderTexture uiRenderTexture;
    private Camera uiCamera;
    private GameObject uiCameraObject;
    private ParticleSystem uiPointCloudDisplay;
    private ParticleSystem.Particle[] uiDisplayParticles;
    private int currentUIDisplayPointCount = 0;
    private Material uiPointMaterial;
    private List<PointData> allPointsData = new List<PointData>();

    void Start()
    {
        InitializeSystem();
        if (showInWorld) InitializePointCloudVisualization();
        if (showInUI)
        {
            InitializeUIPointCloudDisplay();
            //AdjustUIViewToFitScan();
        }

    }
    void OnApplicationQuit()
    {
        if (totalPointsCollected > 0 && currentState != FlightState.Completed)
        {
            Debug.Log("提前退出，紧急落盘点云...");
            SaveScanData();
        }
    }


    void InitializeSystem()
    {
        sessionId = DateTime.Now.ToString("yyyyMMdd_HHmmss");

        // 刚体设置
        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            rb = gameObject.AddComponent<Rigidbody>();
        }
        rb.useGravity = true;
        rb.drag = 0.1f;
        rb.angularDrag = 1f;

        // 设置初始位置: X 起点在 swathLength/2 (沿 -X 飞), Z 起点自动居中,
        // 使 numberOfSwaths 条带在 +Z 方向展开后整体中心落在原点。
        // Z 跨度 = (numberOfSwaths-1)*swathWidth, 起点 Z = -跨度/2。
        float zSpan = (numberOfSwaths - 1) * swathWidth;
        transform.position = new Vector3(swathLength / 2, targetHeight, -zSpan / 2f);

        // 机头朝向（-X是前方）
        transform.rotation = Quaternion.Euler(0, 90, 0);

        // 设置初始速度（沿-X方向飞）
        rb.velocity = Vector3.left * cruiseSpeed;
        currentFlightDirection = Vector3.left;

        // 初始化激光方向
        //CalculateLaserDirections();
        CalculateBeamAngles();
        // 初始化可视化
        InitializeLineRenderers();


        if (qualityAssessor != null)
            qualityAssessor.MaxScanAngleFromNadir = maxScanAngleFromNadir;
        // open streaming metrics CSV (same path as SaveMetrics uses)
        if (qualityAssessor != null)
        {
            string mdir = Path.Combine(Application.dataPath, "..", "PointClouds", sessionId);
            if (!Directory.Exists(mdir)) Directory.CreateDirectory(mdir);
            qualityAssessor.OpenMetricsStream(mdir);
        }
        // open performance CSV (per FixedUpdate step) -- independent of the assessor,
        // so that a run without assessment can serve as a control
        {
            string pdir = Path.Combine(Application.dataPath, "..", "PointClouds", sessionId);
            if (!Directory.Exists(pdir)) Directory.CreateDirectory(pdir);
            perfWriter = new StreamWriter(Path.Combine(pdir, $"Performance_{sessionId}.csv"));
            perfWriter.WriteLine("Step,Timestamp,WallMs,ScanMs,MetricMs,MemMB,LiveMB,ReservedMB,NativeMB,GfxMB,PointsThisStep,CumPoints,LateSteps");
        }
        // 开始扫描
        StartScanning();
    }

    // 每束的固定俯仰角缓存（沿航向 along-track）
    private float[] beamSinTheta;
    private float[] beamCosTheta;

    void CalculateBeamAngles()
    {
        beamSinTheta = new float[lines];
        beamCosTheta = new float[lines];

        // 论文公式 (1-based): θ_i = -(2i - n - 1) / (2n) * Θ
        for (int i = 1; i <= lines; i++)
        {
            float thetaDeg = -((2f * i - lines - 1) / (2f * lines)) * verticalFOV;
            float thetaRad = thetaDeg * Mathf.Deg2Rad;
            beamSinTheta[i - 1] = Mathf.Sin(thetaRad);
            beamCosTheta[i - 1] = Mathf.Cos(thetaRad);
        }
    }
    // 计算时刻 t 的旋转角，归一化到 [-π, π]
    float ComputeOmegaAt(float t)
    {
        float raw = 2f * Mathf.PI * rotationFrequency * t
                    + initialRotationPhase * Mathf.Deg2Rad;
        return Mathf.Repeat(raw + Mathf.PI, 2f * Mathf.PI) - Mathf.PI;
    }

    // 在局部坐标系下根据 ω 算出第 i 束的射线方向
    Vector3 GetLocalBeamDirection(int i, float cosOmega, float sinOmega)
    {
        return new Vector3(
            beamSinTheta[i],
            -beamCosTheta[i] * cosOmega,
            -beamCosTheta[i] * sinOmega
        );
    }

    // 初始化射线渲染
    void InitializeLineRenderers()
    {
        if (lineRendArray != null)
        {
            foreach (var lr in lineRendArray)
            {
                if (lr != null && lr.gameObject != null)
                {
                    DestroyImmediate(lr.gameObject);
                }
            }
        }

        lineRendArray = new LineRenderer[lines];
        for (int i = 0; i < lines; i++)
        {
            GameObject lineObj = new GameObject($"LaserRay_{i}");
            lineObj.transform.parent = transform;

            LineRenderer lr = lineObj.AddComponent<LineRenderer>();
            lr.material = new Material(Shader.Find("Sprites/Default"));
            lr.startColor = Color.green;
            lr.endColor = Color.green;
            lr.startWidth = 0.05f;
            lr.endWidth = 0.05f;
            lr.positionCount = 2;

            lineRendArray[i] = lr;
        }
    }

    // 初始化点云显示_world
    void InitializePointCloudVisualization()
    {
        if (!showInWorld) return;
        // 创建点云显示对象
        GameObject pointCloudObj = new GameObject("PointCloud_World_Visualization");
        pointCloudObj.transform.parent = null;  // 不作为飞机的子对象，保持独立
        pointCloudDisplay = pointCloudObj.AddComponent<ParticleSystem>();

        // 配置ParticleSystem主模块
        var main = pointCloudDisplay.main;
        main.maxParticles = maxDisplayPoints;
        main.loop = false;
        main.playOnAwake = false;
        main.simulationSpace = ParticleSystemSimulationSpace.World;
        main.startLifetime = Mathf.Infinity;  // 点不消失
        main.startSpeed = 0;

        // 关闭不需要的模块
        var emission = pointCloudDisplay.emission;
        emission.enabled = false;
        var shape = pointCloudDisplay.shape;
        shape.enabled = false;

        var velocityOverLifetime = pointCloudDisplay.velocityOverLifetime;
        velocityOverLifetime.enabled = false;

        // 配置渲染器
        particleRenderer = pointCloudDisplay.GetComponent<ParticleSystemRenderer>();
        particleRenderer.material = new Material(Shader.Find("Sprites/Default"));
        particleRenderer.sortMode = ParticleSystemSortMode.Distance;
        //particleRenderer.minParticleSize = 0.01f;
        //particleRenderer.maxParticleSize = 1f;

        // 初始化粒子数组
        displayParticles = new ParticleSystem.Particle[maxDisplayPoints];

        // 初始化颜色梯度（如果没有在Inspector中设置）
        /*if (heightGradient == null || heightGradient.colorKeys.Length == 0)
        {
            heightGradient = new Gradient();
            GradientColorKey[] colorKeys = new GradientColorKey[5];
            colorKeys[0] = new GradientColorKey(Color.blue, 0.0f);
            colorKeys[1] = new GradientColorKey(Color.cyan, 0.25f);
            colorKeys[2] = new GradientColorKey(Color.green, 0.5f);
            colorKeys[3] = new GradientColorKey(Color.yellow, 0.75f);
            colorKeys[4] = new GradientColorKey(Color.red, 1.0f);

            GradientAlphaKey[] alphaKeys = new GradientAlphaKey[2];
            alphaKeys[0] = new GradientAlphaKey(1.0f, 0.0f);
            alphaKeys[1] = new GradientAlphaKey(1.0f, 1.0f);

            heightGradient.SetKeys(colorKeys, alphaKeys);
        }

        if (intensityGradient == null || intensityGradient.colorKeys.Length == 0)
        {
            intensityGradient = new Gradient();
            GradientColorKey[] colorKeys = new GradientColorKey[3];
            colorKeys[0] = new GradientColorKey(Color.black, 0.0f);
            colorKeys[1] = new GradientColorKey(Color.gray, 0.5f);
            colorKeys[2] = new GradientColorKey(Color.white, 1.0f);

            GradientAlphaKey[] alphaKeys = new GradientAlphaKey[2];
            alphaKeys[0] = new GradientAlphaKey(1.0f, 0.0f);
            alphaKeys[1] = new GradientAlphaKey(1.0f, 1.0f);

            intensityGradient.SetKeys(colorKeys, alphaKeys);
        }*/

        Debug.Log($"点云可视化系统初始化完成 - 最大点数: {maxDisplayPoints}");
    }

    // 初始化点云显示_UI
    void InitializeUIPointCloudDisplay()
    {
        // 创建渲染纹理
        uiRenderTexture = new RenderTexture(
            (int)uiPanelSize.x,
            (int)uiPanelSize.y,
            24,
            RenderTextureFormat.ARGB32
        );
        uiRenderTexture.antiAliasing = 2;

        // 创建UI摄像机
        uiCameraObject = new GameObject("UI_PointCloud_Camera");
        uiCameraObject.transform.position = new Vector3(1000, 1000, 1000); // 放在远离主场景的位置

        uiCamera = uiCameraObject.AddComponent<Camera>();
        uiCamera.targetTexture = uiRenderTexture;
        uiCamera.clearFlags = CameraClearFlags.SolidColor;
        uiCamera.backgroundColor = new Color(0.1f, 0.1f, 0.15f, 1f); // 深色背景
        uiCamera.cullingMask = 1 << LayerMask.NameToLayer("UI"); // 只渲染UI层
        uiCamera.orthographic = false;
        uiCamera.fieldOfView = uiCameraFOV;
        uiCamera.nearClipPlane = 0.1f;
        uiCamera.farClipPlane = 1000f;

        // 设置相机看向点云中心
        uiCamera.transform.LookAt(
            uiCameraObject.transform.position + Vector3.forward,
            Vector3.up
        );

        // 创建UI点云粒子系统
        GameObject uiPointCloudObj = new GameObject("UI_PointCloud_Display");
        uiPointCloudObj.transform.position = uiCameraObject.transform.position + Vector3.forward * uiViewDistance;
        uiPointCloudObj.layer = LayerMask.NameToLayer("UI");

        uiPointCloudDisplay = uiPointCloudObj.AddComponent<ParticleSystem>();

        // 配置UI粒子系统
        var main = uiPointCloudDisplay.main;
        main.maxParticles = maxDisplayPoints;
        main.loop = false;
        main.playOnAwake = false;
        main.simulationSpace = ParticleSystemSimulationSpace.World;
        main.startLifetime = Mathf.Infinity;
        main.startSpeed = 0;

        // 关闭不需要的模块
        var emission = uiPointCloudDisplay.emission;
        emission.enabled = false;
        var shape = uiPointCloudDisplay.shape;
        shape.enabled = false;

        // 配置UI渲染器
        var uiRenderer = uiPointCloudDisplay.GetComponent<ParticleSystemRenderer>();
        uiPointMaterial = new Material(Shader.Find("Sprites/Default"));
        uiRenderer.material = uiPointMaterial;
        uiRenderer.sortMode = ParticleSystemSortMode.Distance;

        // 初始化UI粒子数组
        uiDisplayParticles = new ParticleSystem.Particle[maxDisplayPoints];

        // 颜色梯度设置
        if (heightGradient == null || heightGradient.colorKeys.Length == 0)
        {
            heightGradient = CreateGradient(
                new Color[] {
                new Color(0, 0, 1, 1),    // 蓝色
                new Color(0, 1, 1, 1),    // 青色
                new Color(0, 1, 0, 1),    // 绿色
                new Color(1, 1, 0, 1),    // 黄色
                new Color(1, 0, 0, 1)     // 红色
                }
            );
        }

        if (intensityGradient == null || intensityGradient.colorKeys.Length == 0)
        {
            intensityGradient = CreateGradient(
                new Color[] {
                new Color(0, 0, 0.5f, 1), // 深蓝
                new Color(0, 0.5f, 1, 1),  // 蓝
                new Color(0, 1, 0, 1),     // 绿色
                new Color(1, 1, 0, 1),     // 黄色
                new Color(1, 0, 0, 1)      // 红色
                }
            );
        }

        if (distanceGradient == null || distanceGradient.colorKeys.Length == 0)
        {
            distanceGradient = CreateGradient(
                new Color[] {
            new Color(0, 0, 1, 1),      // 蓝色 - 近距离（44m）
            new Color(0, 0.5f, 1, 1),   // 浅蓝
            new Color(0, 1, 0, 1),       // 绿色 - 中等距离
            new Color(0.5f, 1, 0, 1),   // 黄绿
            new Color(1, 1, 0, 1),       // 黄色 - 远距离（140m）
                }
            );
        }


        Debug.Log("UI点云显示系统初始化完成");
    }
    // UI自适应
    void AdjustUIViewToFitScan()
    {
        if (uiCamera == null || uiPointCloudDisplay == null) return;

        // 计算需要显示的范围
        float scanWidth = swathWidth;  // 扫描宽度
        float scanLength = swathLength; // 扫描长度

        // 自动调整缩放比例，确保整个扫描区域都在视野内
        float maxDimension = Mathf.Max(scanWidth, scanLength);
        uiPointScale = 100f / maxDimension;  // 100是UI空间中的参考大小

        // 调整相机距离
        uiViewDistance = maxDimension * 2f;

        // 更新相机位置
        if (uiCameraObject != null)
        {
            uiCameraObject.transform.position = new Vector3(1000, 1000, 1000 - uiViewDistance);
            uiPointCloudDisplay.transform.position = new Vector3(1000, 1000, 1000);
        }
    }



    // 创建颜色渐变
    Gradient CreateGradient(Color[] colors)
    {
        Gradient gradient = new Gradient();
        GradientColorKey[] colorKeys = new GradientColorKey[colors.Length];
        for (int i = 0; i < colors.Length; i++)
        {
            colorKeys[i] = new GradientColorKey(colors[i], i / (float)(colors.Length - 1));
        }

        GradientAlphaKey[] alphaKeys = new GradientAlphaKey[2];
        alphaKeys[0] = new GradientAlphaKey(1.0f, 0.0f);
        alphaKeys[1] = new GradientAlphaKey(1.0f, 1.0f);

        gradient.SetKeys(colorKeys, alphaKeys);
        return gradient;
    }
    public void StartScanning()
    {
        isScanning = true;
        currentSwathIndex = 0;
        currentState = FlightState.StraightFlight;
        scanCount = 0;
        totalPointsCollected = 0;
        distanceInCurrentSwath = 0f;
        turningRight = false; // 第一次转弯将是右转
        isFirstSwath = true;

        pointCloudData.AppendLine($"# LiDAR S-Pattern Scan Point Cloud");
        pointCloudData.AppendLine($"# Session: {sessionId}");
        pointCloudData.AppendLine($"# Date: {DateTime.Now}");
        pointCloudData.AppendLine($"# Format: x y z range intensity scan_angle");

        trajectoryData.Clear();
        trajectoryData.AppendLine($"# LiDAR S-Pattern Platform Trajectory");
        trajectoryData.AppendLine($"# Session: {sessionId}");
        trajectoryData.AppendLine($"# Date: {DateTime.Now}");
        trajectoryData.AppendLine($"# Sampling: per FixedUpdate (dt = {Time.fixedDeltaTime:F4} s)");
        trajectoryData.AppendLine($"# Format: step  t  x  y  z  qx  qy  qz  qw  vx  vy  vz  swath_id  state");
        fixedStepCounter = 0;


        Debug.Log($"开始S型扫描 - 航线数:{numberOfSwaths}, 间距:{swathWidth}m, 长度:{swathLength}m");
        swathTime = 0f;
    }


    // 论文 Eq. 中 s_t 的 (t, p(t), R(t)) 部分；点云文件 + 本文件 = 完整 s_t 序列
    void RecordTrajectorySample()
    {
        Vector3 p = transform.position;
        Quaternion q = transform.rotation;
        Vector3 v = rb != null ? rb.velocity : Vector3.zero;
        float t = Time.fixedTime;

        if (saveTrajectory)
        trajectoryData.AppendLine(
            $"{fixedStepCounter} " +
            $"{t:F4} " +
            $"{p.x:F4} {p.y:F4} {p.z:F4} " +
            $"{q.x:F6} {q.y:F6} {q.z:F6} {q.w:F6} " +
            $"{v.x:F4} {v.y:F4} {v.z:F4} " +
            $"{currentSwathIndex} " +
            $"{(int)currentState}");

        fixedStepCounter++;
    }



    void FixedUpdate()
    {
        if (loopSurvey && maxMissionSeconds > 0f && Time.fixedTime >= maxMissionSeconds)
        { if (isScanning) CompleteScan(); return; }
        if (!isScanning || currentState == FlightState.Completed) return;

        switch (currentState)
        {
            case FlightState.StraightFlight: ExecuteStraightFlight(); break;
            case FlightState.Turning: ExecuteTurning(); break;
        }

        if (qualityAssessor != null)
            qualityAssessor.CurrentPhase =
                (currentState == FlightState.Turning) ? "Turning"
                : (isAccelerating ? "Accel" : "Cruise");

        long _ps0 = System.Diagnostics.Stopwatch.GetTimestamp();
        PerformScanSubstepped();
        long _ps1 = System.Diagnostics.Stopwatch.GetTimestamp();
        if (perfWriter != null)
        {
            if (!wallClock.IsRunning) wallClock.Start();

            double scanMs   = (_ps1 - _ps0) * 1000.0 / System.Diagnostics.Stopwatch.Frequency;
            double metricMs = (qualityAssessor != null) ? qualityAssessor.LastSweepMetricMs : 0.0;
            double rayMs    = scanMs - metricMs; if (rayMs < 0) rayMs = 0;
            double memMB    = System.GC.GetTotalMemory(false) / (1024.0 * 1024.0);

            // live set: forcing a collection is expensive, so sample it sparsely
            if (perfStepCounter % LIVE_EVERY == 0)
                liveMB = System.GC.GetTotalMemory(true) / (1024.0 * 1024.0);

            double reservedMB = UnityEngine.Profiling.Profiler.GetTotalReservedMemoryLong()  / (1024.0 * 1024.0);
            double nativeMB   = UnityEngine.Profiling.Profiler.GetTotalAllocatedMemoryLong() / (1024.0 * 1024.0);
            double gfxMB      = UnityEngine.Profiling.Profiler.GetAllocatedMemoryForGraphicsDriver() / (1024.0 * 1024.0);

            // throughput: points produced in this physics step
            long ptsThisStep = totalPointsCollected - cumPoints;
            cumPoints = totalPointsCollected;

            // deadline miss: the step took longer than the fixed timestep it represents
            if (scanMs > Time.fixedDeltaTime * 1000.0) lateSteps++;

            double wallMs = wallClock.Elapsed.TotalMilliseconds;

            perfWriter.WriteLine(
                $"{perfStepCounter},{Time.fixedTime:F4},{wallMs:F1},{rayMs:F4},{metricMs:F4}," +
                $"{memMB:F3},{liveMB:F3},{reservedMB:F3},{nativeMB:F3},{gfxMB:F3}," +
                $"{ptsThisStep},{cumPoints},{lateSteps}");
            if ((++perfStepCounter & 63) == 0) perfWriter.Flush();
        }
        RecordTrajectorySample();   // 论文 2.2: 记录 s_t 的位姿部分

        prevScanPosition = transform.position;
        prevScanRotation = transform.rotation;
        hasPrevPose = true;
    }
    void Update()
    {
        if (!isScanning || currentState == FlightState.Completed) return;

        // 平滑更新LineRenderer
        if (lastScanOrigins != null && lastScanOrigins.Length == lines)
        {
            float omegaNow = ComputeOmegaAt(Time.time);
            float cosO = Mathf.Cos(omegaNow);
            float sinO = Mathf.Sin(omegaNow);
            bool groundFacing = Mathf.Abs(omegaNow) <= maxScanAngleFromNadir * Mathf.Deg2Rad;

            for (int i = 0; i < lines; i++)
            {
                if (lineRendArray[i] == null) continue;

                Vector3 currentOrigin = transform.position;
                Vector3 currentEndPoint;

                if (hasNewScanData && i < lastScanHitPoints.Length)
                {
                    // 沿用上次扫描的击中点，避免高频闪烁
                    currentEndPoint = lastScanHitPoints[i];
                }
                else
                {
                    Vector3 localDir = GetLocalBeamDirection(i, cosO, sinO);
                    Vector3 worldDir = transform.TransformDirection(localDir);
                    currentEndPoint = currentOrigin + worldDir * maxDetectionDistance;
                }

                lineRendArray[i].SetPosition(0, currentOrigin);
                lineRendArray[i].SetPosition(1, currentEndPoint);

                Color rayColor;
                if (!groundFacing)
                    rayColor = new Color(0.3f, 0.3f, 0.3f, 0.3f); // 朝上：灰色虚示
                else if (hasNewScanData && i < lastScanHasHit.Length)
                    rayColor = lastScanHasHit[i] ? Color.green : Color.red;
                else
                    rayColor = Color.yellow;

                lineRendArray[i].startColor = rayColor;
                lineRendArray[i].endColor = rayColor;
            }
        }

        // UI点云自动旋转
        if (showInUI && uiAutoRotate && uiPointCloudDisplay != null)
        {
            uiPointCloudDisplay.transform.Rotate(Vector3.up, uiRotateSpeed * Time.deltaTime);
        }

    }

    /*void ExecuteStraightFlight()
    {
        // 保持当前飞行方向的速度
        Vector3 targetVelocity = currentFlightDirection * cruiseSpeed;
        targetVelocity.y = rb.velocity.y;

        Vector3 velocityError = targetVelocity - rb.velocity;
        velocityError.y = 0;
        rb.AddForce(velocityError * rb.mass * 10f);

        // 高度控制
        float heightError = targetHeight - transform.position.y;
        float gravityForce = Physics.gravity.magnitude * rb.mass;
        rb.AddForce(transform.up * gravityForce * liftMultiplier);

        if (Mathf.Abs(heightError) > 1f)
        {
            rb.AddForce(Vector3.up * heightError * rb.mass * 0.5f);
        }

        // 更新朝向（机头对准飞行方向）
        if (currentFlightDirection.magnitude > 0.1f)
        {
            Quaternion targetRotation = Quaternion.LookRotation(currentFlightDirection, Vector3.up);
            targetRotation = targetRotation * Quaternion.Euler(0, 90, 0); // 模型修正
            rb.MoveRotation(Quaternion.Slerp(transform.rotation, targetRotation, Time.fixedDeltaTime * 2f));
        }

        // 累计飞行距离
        distanceInCurrentSwath += rb.velocity.magnitude * Time.fixedDeltaTime;

        // 检查是否到达航线末端
        if (distanceInCurrentSwath >= swathLength)
        {
            if (currentSwathIndex + swathStep >= 0 && currentSwathIndex + swathStep < numberOfSwaths)
            {
                InitiateTurn();
            }
            else
            {
                if (loopSurvey)
                {
                    swathStep = -swathStep;          // fold back at the end strip
                    turningRight = !turningRight;    // flip turn side so the arc stays inside the scene
                    InitiateTurn();
                }
                else CompleteScan();
            }
        }
    }*/

    void ExecuteStraightFlight()
    {
        // === 论文 Eq.(14) 沿航向: v(t) = v_0 + a t ===
        //swathTime += Time.fixedDeltaTime;
        //float currentSpeed = Mathf.Max(0.1f, cruiseSpeed + cruiseAcceleration * swathTime);
        swathTime += Time.fixedDeltaTime;

        // 加速段：从 cruiseSpeed 加/减速到 targetCruiseSpeedAfterAccel，达到后保持
        float currentSpeed;
        if (cruiseAcceleration != 0f)
        {
            float raw = cruiseSpeed + cruiseAcceleration * swathTime;
            if (cruiseAcceleration > 0f)
                currentSpeed = Mathf.Min(raw, targetCruiseSpeedAfterAccel);
            else
                currentSpeed = Mathf.Max(raw, targetCruiseSpeedAfterAccel);
        }
        else
        {
            currentSpeed = cruiseSpeed;
        }
        isAccelerating = (cruiseAcceleration != 0f) && (Mathf.Abs(currentSpeed - targetCruiseSpeedAfterAccel) > 0.01f);
        currentSpeed = Mathf.Max(0.1f, currentSpeed);



        Vector3 targetVelocity = currentFlightDirection * currentSpeed;
        targetVelocity.y = rb.velocity.y;

        Vector3 velocityError = targetVelocity - rb.velocity;
        velocityError.y = 0;
        rb.AddForce(velocityError * rb.mass * 10f);

        // 高度控制（不变）
        float heightError = targetHeight - transform.position.y;
        float gravityForce = Physics.gravity.magnitude * rb.mass;
        rb.AddForce(transform.up * gravityForce * liftMultiplier);

        if (Mathf.Abs(heightError) > 1f)
        {
            rb.AddForce(Vector3.up * heightError * rb.mass * 0.5f);
        }

        // === 论文 Eq.(14) 横向/垂向: ε_y, ε_z ~ N(0, σ_v²)，独立同分布 ===
        if (vibrationSigma > 1e-6f)
        {
            Vector3 crossTrack = Vector3.Cross(currentFlightDirection, Vector3.up).normalized;
            float epsY = SampleGaussian() * vibrationSigma;   // 横向（cross-track）
            float epsZ = SampleGaussian() * vibrationSigma;   // 垂向
            rb.velocity += epsY * crossTrack + epsZ * Vector3.up;
        }

        // 更新朝向（用 currentFlightDirection 而非 velocity，避免振动让机头抖）
        if (currentFlightDirection.magnitude > 0.1f)
        {
            Quaternion targetRotation = Quaternion.LookRotation(currentFlightDirection, Vector3.up);
            targetRotation = targetRotation * Quaternion.Euler(0, 90, 0); // 模型修正
            rb.MoveRotation(Quaternion.Slerp(transform.rotation, targetRotation, Time.fixedDeltaTime * 2f));
        }

        // 累计飞行距离
        distanceInCurrentSwath += rb.velocity.magnitude * Time.fixedDeltaTime;

        // 检查是否到达航线末端
        if (distanceInCurrentSwath >= swathLength)
        {
            if (currentSwathIndex + swathStep >= 0 && currentSwathIndex + swathStep < numberOfSwaths)
                InitiateTurn();
            else
                if (loopSurvey)
                {
                    swathStep = -swathStep;          // fold back at the end strip
                    turningRight = !turningRight;    // flip turn side so the arc stays inside the scene
                    InitiateTurn();
                }
                else CompleteScan();
        }
    }


    void InitiateTurn()
    {
        currentState = FlightState.Turning;
        totalTurnedAngle = 0f;

        // S型扫描：交替转弯方向
        // 第一次右转，第二次左转，第三次右转...

        // 计算圆心位置（完全按照盘旋代码）
        Vector3 toRight = Vector3.Cross(currentFlightDirection, Vector3.up).normalized;

        if (turningRight)
        {
            // 右转：圆心在飞机右侧
            circleCenter = transform.position + toRight * turnRadius;
        }
        else
        {
            // 左转：圆心在飞机左侧
            circleCenter = transform.position - toRight * turnRadius;
        }

        // 计算初始角度
        Vector3 toPlane = transform.position - circleCenter;
        toPlane.y = 0;
        currentAngle = Mathf.Atan2(toPlane.z, toPlane.x);
        turnStartAngle = currentAngle;

        Debug.Log($"开始{(turningRight ? "右" : "左")}转：从航线{currentSwathIndex}到{currentSwathIndex + 1}");
    }

    void ExecuteTurning()
    {
        // 完全使用盘旋代码的物理模型
        float angularVelocity = cruiseSpeed / turnRadius;

        // 更新角度
        float deltaAngle = angularVelocity * Time.fixedDeltaTime;
        if (turningRight)
        {
            currentAngle += deltaAngle;
        }
        else
        {
            currentAngle -= deltaAngle;
        }
        totalTurnedAngle += deltaAngle;

        // 计算目标位置（在圆上）
        Vector3 positionOnCircle = circleCenter + new Vector3(
            Mathf.Cos(currentAngle) * turnRadius,
            0,
            Mathf.Sin(currentAngle) * turnRadius
        );
        positionOnCircle.y = transform.position.y;

        // 位置误差修正
        Vector3 positionError = positionOnCircle - transform.position;
        positionError.y = 0;

        // 计算切线方向（速度方向）
        Vector3 tangent;
        if (turningRight)
        {
            tangent = new Vector3(-Mathf.Sin(currentAngle), 0, Mathf.Cos(currentAngle));
        }
        else
        {
            tangent = new Vector3(Mathf.Sin(currentAngle), 0, -Mathf.Cos(currentAngle));
        }

        // 速度控制
        Vector3 targetVelocity = tangent * cruiseSpeed;
        targetVelocity.y = rb.velocity.y;

        Vector3 velocityError = targetVelocity - rb.velocity;
        velocityError.y = 0;
        rb.AddForce(velocityError * rb.mass * 10f);

        // 向心力
        if (positionError.magnitude > 0.1f)
        {
            rb.AddForce(positionError.normalized * rb.mass * positionError.magnitude * 5f);
        }

        // 计算并应用倾斜角
        float targetBankAngle = Mathf.Atan2(cruiseSpeed * cruiseSpeed, turnRadius * Physics.gravity.magnitude) * Mathf.Rad2Deg;
        targetBankAngle = Mathf.Clamp(targetBankAngle, 0, maxBankAngle);
        if (!turningRight) targetBankAngle = -targetBankAngle;

        currentBankAngle = Mathf.Lerp(currentBankAngle, targetBankAngle, Time.fixedDeltaTime * bankSmoothSpeed);

        // 升力
        float gravityForce = Physics.gravity.magnitude * rb.mass;
        float heightError = targetHeight - transform.position.y;
        float heightCorrection = Mathf.Clamp(heightError * 0.01f, -0.1f, 0.1f);
        float liftBoost = 1f / Mathf.Max(Mathf.Cos(currentBankAngle * Mathf.Deg2Rad), 0.5f);
        rb.AddForce(transform.up * gravityForce * (liftMultiplier + heightCorrection) * liftBoost);

        // 更新朝向和横滚
        if (tangent.magnitude > 0.1f)
        {
            Quaternion targetRotation = Quaternion.LookRotation(tangent, Vector3.up);
            targetRotation = targetRotation * Quaternion.Euler(0, 0, -currentBankAngle);
            targetRotation = targetRotation * Quaternion.Euler(0, 90, 0); // 模型修正
            rb.MoveRotation(Quaternion.Slerp(transform.rotation, targetRotation, Time.fixedDeltaTime * 2f));
        }

        // 检查是否完成180度转弯
        if (totalTurnedAngle >= Mathf.PI - 0.05f) // 接近180度
        {
            CompleteTurn();
        }
    }

    void CompleteTurn()
    {
        currentState = FlightState.StraightFlight;
        currentSwathIndex += swathStep;
        distanceInCurrentSwath = 0f;
        currentBankAngle = 0f;

        // 反转飞行方向
        currentFlightDirection = -currentFlightDirection;

        // S型：交替转弯方向
        turningRight = !turningRight;

        Debug.Log($"完成转弯，进入航线 {currentSwathIndex}，下次{(turningRight ? "右转" : "左转")}");
        if (resetAccelPerSwath) swathTime = 0f;
        AutoSaveProgress();


    }
    // 论文 Eq.(14) 的 ε_y, ε_z 采样：标准正态，独立
    float SampleGaussian()
    {
        float u1 = Mathf.Max(1e-7f, UnityEngine.Random.value);
        float u2 = UnityEngine.Random.value;
        return Mathf.Sqrt(-2f * Mathf.Log(u1)) * Mathf.Cos(2f * Mathf.PI * u2);
    }


    /*void AutoSaveProgress()
    {
        string outputDir = Path.Combine(Application.dataPath, "..", "PointClouds", sessionId);
        if (!Directory.Exists(outputDir)) Directory.CreateDirectory(outputDir);
        string file = Path.Combine(outputDir, $"AUTO_swath{currentSwathIndex}_{sessionId}.txt");
        File.WriteAllText(file, pointCloudData.ToString());
        Debug.Log($"自动保存：{file} ({totalPointsCollected} pts)");
    }*/

    void AutoSaveProgress()
    {
        string outputDir = Path.Combine(Application.dataPath, "..", "PointClouds", sessionId);
        if (!Directory.Exists(outputDir)) Directory.CreateDirectory(outputDir);

        string pcFile = Path.Combine(outputDir, $"AUTO_swath{currentSwathIndex}_{sessionId}.txt");
        File.WriteAllText(pcFile, pointCloudData.ToString());

        string trajFile = Path.Combine(outputDir, $"AUTO_Trajectory_swath{currentSwathIndex}_{sessionId}.txt");
        File.WriteAllText(trajFile, trajectoryData.ToString());

        Debug.Log($"自动保存：{pcFile} ({totalPointsCollected} pts) + 轨迹");
    }

    void PerformScanSubstepped()
    {
        int N = Mathf.Max(1, firingsPerFixedUpdate);
        float dt = Time.fixedDeltaTime;
        float t0 = Time.fixedTime - dt;   // 本物理步起点

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
        int totalAddedThisFixedUpdate = 0;

        for (int k = 0; k < N; k++)
        {
            float u = (k + 1f) / N;            // 子步时间分数 (0,1]
            float tSub = t0 + u * dt;
            // Entry into this simulated firing, before geometry/raycast work.
            long firingWallTick = (qualityAssessor != null && qualityAssessor.enableSweepTiming)
                ? System.Diagnostics.Stopwatch.GetTimestamp() : 0;

            float omega = ComputeOmegaAt(tSub);
            bool isGroundFacing = Mathf.Abs(omega) <= omegaMax;

            // 沿物理步线性插值平台位姿

            //if (Mathf.Abs(omega) > omegaMax) continue;   // 朝上半圈丢弃

            // 沿物理步线性插值平台位姿（dt 内位移 < 0.2 m，线性足够）
            Vector3 scanPosition = Vector3.Lerp(startPos, endPos, u);
            Quaternion scanRotation = Quaternion.Slerp(startRot, endRot, u);

            //
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

                    // 只在最后一个子步更新可视化缓存，避免射线显示被高频覆盖
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

                        if (savePointCloud)
                        pointCloudData.AppendLine(
                            $"{hitPoint.x:F3} {hitPoint.y:F3} {hitPoint.z:F3} " +
                            $"{range:F3} {intensity:F2} {scanAngleDeg:F2}");

                        totalPointsCollected++;
                        AddPointToDisplay(hitPoint, intensity, range);
                        totalAddedThisFixedUpdate++;
                    }
                }

                if (saveRangeMatrix) rangeMatrix.Add(currentScanRanges);



            }





            if (qualityAssessor != null)
            {
                qualityAssessor.OnFiring(currentScanRanges, scanPosition, omega, tSub, firingWallTick);
            }

        }

        if (totalAddedThisFixedUpdate > 0)
        {
            if (showInWorld && pointCloudDisplay != null)
                pointCloudDisplay.SetParticles(displayParticles, currentDisplayPointCount);
            if (showInUI && uiPointCloudDisplay != null)
                uiPointCloudDisplay.SetParticles(uiDisplayParticles, currentUIDisplayPointCount);
        }

        hasNewScanData = true;
    }
    void PerformScan()
    {
        // 当前旋转角
        float omega = ComputeOmegaAt(Time.fixedTime);

        // 朝上的半圈直接丢弃（保持 lastScanData 缓存，避免可视化抖动）
        float omegaMax = maxScanAngleFromNadir * Mathf.Deg2Rad;
        if (Mathf.Abs(omega) > omegaMax)
        {
            // 不发射，但也不更新 hasNewScanData，保持上一帧的射线显示
            return;
        }

        scanCount++;
        List<float> currentScanRanges = new List<float>();
        Vector3 scanPosition = transform.position;
        int pointsAddedThisScan = 0;

        if (lastScanOrigins == null || lastScanOrigins.Length != lines)
        {
            lastScanOrigins = new Vector3[lines];
            lastScanHitPoints = new Vector3[lines];
            lastScanHasHit = new bool[lines];
        }

        float cosOmega = Mathf.Cos(omega);
        float sinOmega = Mathf.Sin(omega);

        for (int i = 0; i < lines; i++)
        {
            Vector3 localDir = GetLocalBeamDirection(i, cosOmega, sinOmega);
            Vector3 worldDirection = transform.TransformDirection(localDir);

            RaycastHit hit;
            bool hasHit = Physics.Raycast(scanPosition, worldDirection, out hit, maxDetectionDistance);

            float range = hasHit ? hit.distance : -1f;
            currentScanRanges.Add(range);

            lastScanOrigins[i] = scanPosition;
            lastScanHasHit[i] = hasHit;
            lastScanHitPoints[i] = hasHit
                ? hit.point
                : scanPosition + worldDirection * maxDetectionDistance;

            if (hasHit)
            {
                Vector3 hitPoint = hit.point;
                // 用 ω 作为 cross-track 扫描角，记录到点云元数据
                float scanAngleDeg = omega * Mathf.Rad2Deg;
                float intensity = CalculateIntensity(hit, range);

                if (savePointCloud)
                pointCloudData.AppendLine(
                    $"{hitPoint.x:F3} {hitPoint.y:F3} {hitPoint.z:F3} " +
                    $"{range:F3} {intensity:F2} {scanAngleDeg:F2}");

                totalPointsCollected++;
                AddPointToDisplay(hitPoint, intensity, range);
                pointsAddedThisScan++;
            }
        }

        if (pointsAddedThisScan > 0)
        {
            if (showInWorld && pointCloudDisplay != null)
                pointCloudDisplay.SetParticles(displayParticles, currentDisplayPointCount);
            if (showInUI && uiPointCloudDisplay != null)
                uiPointCloudDisplay.SetParticles(uiDisplayParticles, currentUIDisplayPointCount);
        }

        hasNewScanData = true;
        if (saveRangeMatrix) rangeMatrix.Add(currentScanRanges);
    }

    float CalculateIntensity(RaycastHit hit, float range)
    {
        float baseIntensity = 100f;
        float distanceAttenuation = Mathf.Exp(-range * 0.01f);
        float incidentAngle = Vector3.Angle(hit.normal, -hit.point.normalized);
        float angleEffect = Mathf.Cos(incidentAngle * Mathf.Deg2Rad);
        return baseIntensity * distanceAttenuation * angleEffect;
    }

    // 更新射线渲染
    void UpdateVisualization(int lineIndex, Vector3 origin, Vector3 hitPoint, bool hasHit)
    {
        if (lineRendArray[lineIndex] != null)
        {
            lineRendArray[lineIndex].SetPosition(0, origin);
            lineRendArray[lineIndex].SetPosition(1, hasHit ? hitPoint : origin + laserDirections[lineIndex] * 10f);

            Color rayColor = hasHit ? Color.green : Color.red;
            lineRendArray[lineIndex].startColor = rayColor;
            lineRendArray[lineIndex].endColor = rayColor;
        }
    }


    void AddPointToDisplay(Vector3 point, float intensity, float distance)
    {
        if (!enablePointDisplay) return;   // off for perf/timeseries runs -> no point accumulation
        // 保存点数据
        PointData pointData = new PointData
        {
            position = point,
            intensity = intensity,
            distance = distance
        };
        allPointsData.Add(pointData);

        // 计算颜色
        Color pointColor = CalculatePointColor(point, intensity, distance);
        // 添加到世界显示（确保使用独立的计数器）
        if (showInWorld && pointCloudDisplay != null && currentDisplayPointCount < maxDisplayPoints)
        {
            displayParticles[currentDisplayPointCount].position = point;
            displayParticles[currentDisplayPointCount].startSize = actualPointSize;
            displayParticles[currentDisplayPointCount].startColor = pointColor;
            displayParticles[currentDisplayPointCount].remainingLifetime = Mathf.Infinity;
            currentDisplayPointCount++;  // 世界显示的计数器
        }


        // 添加到UI显示
        if (showInUI && uiPointCloudDisplay != null && currentUIDisplayPointCount < maxDisplayPoints)
        {
            // 转换到UI坐标系
            // 假设飞行是沿X轴，我们要让它在UI中向上（Y轴）增长
            Vector3 relativePos = point - new Vector3(0, targetHeight, 0); // 不再偏移swathLength/2

            // 重新映射坐标：
            // 世界X（飞行方向） -> UI的Y（向上）
            // 世界Z（扫描宽度） -> UI的X（左右）
            // 世界Y（高度） -> UI的Z（深度）
            Vector3 uiMappedPos = new Vector3(
                relativePos.z * uiPointScale,      // 扫描宽度映射到左右
                -relativePos.x * uiPointScale,      // 飞行方向映射到上下
                relativePos.y * uiPointScale       // 高度映射到深度
            );

            // 偏移到UI空间中心
            Vector3 uiPos = uiPointCloudDisplay.transform.position + uiMappedPos;

            uiDisplayParticles[currentUIDisplayPointCount].position = uiPos;
            uiDisplayParticles[currentUIDisplayPointCount].startSize = uiPointSize;
            uiDisplayParticles[currentUIDisplayPointCount].startColor = pointColor;
            uiDisplayParticles[currentUIDisplayPointCount].remainingLifetime = Mathf.Infinity;
            currentUIDisplayPointCount++;
        }
    }


    // 计算点的颜色
    Color CalculatePointColor(Vector3 point, float intensity, float distance)
    {
        Color pointColor = Color.white;
        float normalizedValue = 0;

        switch (colorMode)
        {
            case ColorMode.Height:
                normalizedValue = Mathf.InverseLerp(minHeightColor, maxHeightColor, point.y);
                pointColor = heightGradient.Evaluate(Mathf.Clamp01(normalizedValue));
                break;

            case ColorMode.Intensity:
                normalizedValue = Mathf.InverseLerp(minIntensity, maxIntensity, intensity);
                pointColor = intensityGradient.Evaluate(Mathf.Clamp01(normalizedValue));
                break;

            case ColorMode.Distance:
                // 不再反转，直接映射：近（44m）->0（蓝），远（140m）->1（黄）
                normalizedValue = Mathf.InverseLerp(minDistance, maxDistance, distance);
                pointColor = distanceGradient.Evaluate(Mathf.Clamp01(normalizedValue));

                // 调试输出，帮助验证
                if (UnityEngine.Random.Range(0, 100) < 1) // 随机输出1%的点用于调试
                {
                    //Debug.Log($"Distance: {distance:F1}m, Normalized: {normalizedValue:F2}, Color: {pointColor}");
                }
                break;
        }

        return pointColor;
    }


    // 批量更新显示（提高性能）
    void UpdatePointCloudDisplay()
    {
        if (!enableRealtimeVisualization || pointCloudDisplay == null) return;

        pointCloudDisplay.SetParticles(displayParticles, currentDisplayPointCount);
    }



    // 清除点云显示的方法
    public void ClearPointCloudDisplay()
    {
        if (pointCloudDisplay != null)
        {
            currentDisplayPointCount = 0;
            pointCloudDisplay.Clear();
            Debug.Log("点云显示已清除");
        }
    }

    // 重置UI视角
    void ResetUIView()
    {
        if (uiPointCloudDisplay != null)
        {
            uiPointCloudDisplay.transform.rotation = Quaternion.identity;
        }
    }


    // 点云颜色刷新方法
    void RefreshAllColors()
    {
        for (int i = 0; i < allPointsData.Count && i < maxDisplayPoints; i++)
        {
            Color newColor = CalculatePointColor(
                allPointsData[i].position,
                allPointsData[i].intensity,
                allPointsData[i].distance
            );

            if (showInWorld && i < currentDisplayPointCount)
            {
                displayParticles[i].startColor = newColor;
            }

            if (showInUI && i < currentUIDisplayPointCount)
            {
                uiDisplayParticles[i].startColor = newColor;
            }
        }

        if (showInWorld && pointCloudDisplay != null)
        {
            pointCloudDisplay.SetParticles(displayParticles, currentDisplayPointCount);
        }

        if (showInUI && uiPointCloudDisplay != null)
        {
            uiPointCloudDisplay.SetParticles(uiDisplayParticles, currentUIDisplayPointCount);
        }
    }


    // 清理资源
    void OnDestroy()
    {
        // 原有清理逻辑保留
        if (pointCloudDisplay != null) DestroyImmediate(pointCloudDisplay.gameObject);
        if (uiCameraObject != null) DestroyImmediate(uiCameraObject);
        if (uiRenderTexture != null) uiRenderTexture.Release();

        // 兜底保存
        if (totalPointsCollected > 0 && pointCloudData != null && pointCloudData.Length > 200)
        {
            try { SaveScanData(); } catch { /* Editor 退 Play 时可能 IO 失败，忽略 */ }
        }
    }



    void CompleteScan()
    {
        currentState = FlightState.Completed;
        isScanning = false;
        rb.velocity = Vector3.zero;

        SaveScanData();
        Debug.Log($"S型扫描完成！总点数: {totalPointsCollected}, 扫描次数: {scanCount}");
//#if UNITY_EDITOR
//    UnityEditor.EditorApplication.isPlaying = false;
//#else
//    Application.Quit();
//#endif

    }

    void SaveScanData()
    {
        string outputDir = Path.Combine(Application.dataPath, "..", "PointClouds", sessionId);

        if (!Directory.Exists(outputDir))
        {
            Directory.CreateDirectory(outputDir);
        }

        string pointCloudFile = Path.Combine(outputDir, $"SPattern_PointCloud_{sessionId}.txt");
        File.WriteAllText(pointCloudFile, pointCloudData.ToString());

        string matrixFile = Path.Combine(outputDir, $"SPattern_RangeMatrix_{sessionId}.txt");
        SaveRangeMatrix(matrixFile);

        string trajectoryFile = Path.Combine(outputDir, $"SPattern_Trajectory_{sessionId}.txt");
        File.WriteAllText(trajectoryFile, trajectoryData.ToString());

        // 保存质量指标
        if (qualityAssessor != null)
        {
            qualityAssessor.SaveMetrics(outputDir);
        }
        // close performance CSV
        if (perfWriter != null)
        {
            perfWriter.Flush(); perfWriter.Close(); perfWriter = null;
        }




        Debug.Log($"数据已保存到: {outputDir}");

#if UNITY_EDITOR
        UnityEditor.AssetDatabase.Refresh();
#endif
    }

    void SaveRangeMatrix(string filepath)
    {
        using (StreamWriter writer = new StreamWriter(filepath))
        {
            writer.WriteLine($"# S-Pattern Range Matrix");
            writer.WriteLine($"# Scans: {rangeMatrix.Count}");
            writer.WriteLine($"# Lines per scan: {lines}");

            foreach (var scan in rangeMatrix)
            {
                writer.WriteLine(string.Join(" ", scan.ConvertAll(r => r.ToString("F3"))));
            }
        }
    }

    void OnGUI()
    {
        GUI.Label(new Rect(10, 10, 400, 20), $"状态: {currentState}");
        GUI.Label(new Rect(10, 30, 400, 20), $"航线: {currentSwathIndex + 1}/{numberOfSwaths}");
        GUI.Label(new Rect(10, 50, 400, 20), $"速度: {rb.velocity.magnitude:F1} m/s");
        GUI.Label(new Rect(10, 70, 400, 20), $"高度: {transform.position.y:F1} m");
        GUI.Label(new Rect(10, 90, 400, 20), $"点云数量: {totalPointsCollected}");
        GUI.Label(new Rect(10, 110, 400, 20), $"飞行方向: {currentFlightDirection}");

        if (currentState == FlightState.StraightFlight)
        {
            GUI.Label(new Rect(10, 130, 400, 20), $"航线进度: {distanceInCurrentSwath:F1}/{swathLength:F1} m");
        }
        else if (currentState == FlightState.Turning)
        {
            GUI.Label(new Rect(10, 130, 400, 20), $"转弯进度: {(totalTurnedAngle * Mathf.Rad2Deg):F1}°/180°");
            GUI.Label(new Rect(10, 150, 400, 20), $"倾斜角: {currentBankAngle:F1}°");
            GUI.Label(new Rect(10, 170, 400, 20), $"{(turningRight ? "右转" : "左转")}");
        }

        // UI点云显示窗口
        if (showInUI && uiRenderTexture != null)
        {
            // 绘制背景框
            GUI.Box(new Rect(uiPanelPosition.x - 5, uiPanelPosition.y - 25,
                            uiPanelSize.x + 10, uiPanelSize.y + 30), "点云雷达显示");

            // 绘制渲染纹理
            GUI.DrawTexture(new Rect(uiPanelPosition.x, uiPanelPosition.y,
                                     uiPanelSize.x, uiPanelSize.y), uiRenderTexture);

            // 显示信息
            float infoY = uiPanelPosition.y + uiPanelSize.y + 5;
            GUI.Label(new Rect(uiPanelPosition.x, infoY, 200, 20),
                     $"显示点数: {currentUIDisplayPointCount}");
            GUI.Label(new Rect(uiPanelPosition.x, infoY + 20, 200, 20),
                     $"着色: {colorMode}");

            // 控制按钮
            if (GUI.Button(new Rect(uiPanelPosition.x, infoY + 45, 80, 25), "切换着色"))
            {
                colorMode = (ColorMode)(((int)colorMode + 1) % 3);
                RefreshAllColors();
            }

            if (GUI.Button(new Rect(uiPanelPosition.x + 85, infoY + 45, 80, 25),
                          uiAutoRotate ? "停止旋转" : "开始旋转"))
            {
                uiAutoRotate = !uiAutoRotate;
            }

            if (GUI.Button(new Rect(uiPanelPosition.x + 170, infoY + 45, 80, 25), "重置视角"))
            {
                ResetUIView();
            }
        }

        // 显示模式切换
        if (GUI.Button(new Rect(Screen.width - 110, 10, 100, 30),
                       showInWorld ? "隐藏世界" : "显示世界"))
        {
            showInWorld = !showInWorld;
            if (pointCloudDisplay != null)
                pointCloudDisplay.gameObject.SetActive(showInWorld);
        }

        if (GUI.Button(new Rect(Screen.width - 110, 45, 100, 30),
                       showInUI ? "隐藏UI" : "显示UI"))
        {
            showInUI = !showInUI;
            if (uiCameraObject != null)
                uiCameraObject.SetActive(showInUI);
        }
    }

    void OnDrawGizmos()
    {
        if (Application.isPlaying)
        {
            // 速度方向
            if (rb != null)
            {
                Gizmos.color = Color.blue;
                Gizmos.DrawRay(transform.position, rb.velocity.normalized * 5f);
            }

            // 机头方向（-X）
            Gizmos.color = Color.red;
            Gizmos.DrawRay(transform.position, -transform.right * 5f);

            // 升力方向
            Gizmos.color = Color.green;
            Gizmos.DrawRay(transform.position, transform.up * 8f);

            // 转弯时显示圆心和路径
            if (currentState == FlightState.Turning)
            {
                Gizmos.color = turningRight ? Color.yellow : Color.cyan;
                Gizmos.DrawWireSphere(circleCenter, turnRadius);
                Gizmos.DrawLine(transform.position, circleCenter);
            }
        }
    }

    public void StopScanning()
    {
        isScanning = false;
        currentState = FlightState.Completed;
        Debug.Log($"扫描停止。总计收集 {totalPointsCollected} 个点");
    }
}
