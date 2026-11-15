using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.IO.Compression;
using System.Reflection;
using System.Security.Cryptography;
using UnityEngine;

// An opt-in experiment harness; uses the original terrain algorithm and flight/scanner.
[DefaultExecutionOrder(-1000)]
public class PerlinTerrainExperiment : MonoBehaviour
{
    public bool flatControl;
    public int seed = 42;
    public int resolution = 513;
    public int size = 420;
    public int relief = 20;
    public float noiseScale = 4f;
    public int octaves = 3;
    public float persistence = 0.5f;
    public float lacunarity = 2f;
    public string status = "READY";
    public string outputDirectory = "";
    private SPatternScanFlightPhased scanner;
    private StreamingQualityAssessment qa;
    private Terrain terrain;
    private BinaryWriter points;
    private StreamWriter sweepFile;
    private StreamWriter poses;
    private Manifest manifest;
    private bool inside, finished;
    private int sweepIndex = -1;
    private int nReturns, nRays, fixedSteps;
    private long totalReturns, totalFirings, groundFirings;
    private float sweepStart, sweepEnd;
    private Vector3 firstPosition, lastPosition;
    private double rangeSum, slopeSum, incidenceSum;
    private float minRange, maxRange, minX, maxX, minZ, maxZ, minY, maxY;
    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;
    private static readonly BindingFlags Fields = BindingFlags.Instance | BindingFlags.NonPublic;

    [Serializable] private class Manifest
    {
        public string status, caseName, unityVersion, outputDirectory, originalScannerOutput;
        public string scannerHash, terrainGeneratorHash, assessorHash;
        public int seed, heightmapResolution, terrainWidth, terrainLength, octaves, treeCount;
        public float noiseScale, persistence, lacunarity, requestedRelief;
        public float baseElevation, elevationMin, elevationMax, elevationMean, elevationSD;
        public float slopeMeanDeg, slopeRmsDeg, slopeP95Deg, slopeMaxDeg;
        public float fixedDeltaTime = 0.02f, targetReferenceHeight = 50, cruiseSpeed = 5;
        public float acceleration = 2, acceleratedSpeed = 15, stripLength = 300, stripSpacing = 60, turnRadius = 30;
        public int strips = 5, channels = 16, firingsPerFixedStep = 360, windowSize = 50, bins = 4;
        public float verticalFanDegrees = 30, rotationHz = 10, rangeLimit = 100, angularGateDeg = 89;
        public string channelGeometry = "existing sector-centre convention; unchanged";
        public string flightModel = "original SPatternScanFlightPhased rigid-body dynamics, not a prescribed-pose replacement";
        public string terrainAlgorithm = "original TerrainGenerator: seeded octave offsets, Mathf.PerlinNoise, min-max normalization; mean elevation translated to zero";
        public string pointFormat = "gzip-compressed little endian records: float32 x,y,z; int32 sweepIndex; Unity coordinates, y up";
        public string cciMode = "independent expected counts; nominal-ray return completeness";
        public bool captureDeltaTimeForOfflineExecution = true;
        public float simulationEndTime, wallSeconds;
        public long returnCount, firingCount, groundFacingFiringCount;
        public int recordedSweeps, physicsSteps;
    }

    private float wallStart;
    private void Awake()
    {
        try
        {
            string[] args = Environment.GetCommandLineArgs();
            for (int i = 0; i + 1 < args.Length; i++)
            {
                if (args[i] == "-terrainCase") flatControl = args[i + 1] == "flat";
                if (args[i] == "-terrainOutput") outputDirectory = args[i + 1];
            }
            if (string.IsNullOrEmpty(outputDirectory))
                outputDirectory = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "PointClouds",
                    "Terrain_" + DateTime.Now.ToString("yyyyMMdd_HHmmss_fff"), flatControl ? "flat" : "perlin"));
            Directory.CreateDirectory(outputDirectory);
            Time.fixedDeltaTime = 0.02f;
            Time.captureDeltaTime = 0.02f;
            Application.runInBackground = true;
            QualitySettings.vSyncCount = 0;
            Application.targetFrameRate = -1;
            manifest = new Manifest {
                status = "running", caseName = flatControl ? "flat" : "perlin",
                unityVersion = Application.unityVersion, outputDirectory = outputDirectory,
                seed = seed, heightmapResolution = resolution, terrainWidth = size, terrainLength = size,
                octaves = octaves, noiseScale = noiseScale, persistence = persistence, lacunarity = lacunarity,
                requestedRelief = flatControl ? 0 : relief, treeCount = 0,
                scannerHash = Hash("SPatternScanFlightPhased.cs"),
                terrainGeneratorHash = Hash("TerrainGenerator.cs"), assessorHash = Hash("StreamingQualityAssessment.cs")
            };
            BuildTerrain();
            ConfigureScanner();
            points = new BinaryWriter(new GZipStream(new FileStream(Path.Combine(outputDirectory, "points_xyzs.bin.gz"),
                FileMode.Create, FileAccess.Write, FileShare.Read, 1024 * 1024), System.IO.Compression.CompressionLevel.Fastest));
            sweepFile = new StreamWriter(Path.Combine(outputDirectory, "sweeps.csv"));
            sweepFile.WriteLine("Sweep,StartTime,EndTime,Phase,MeanX,MeanY,MeanZ,NadirGroundY,Clearance,CastRays,ValidReturns,MeanRange,MinRange,MaxRange,MeanHitSlopeDeg,MeanIncidenceDeg,MinX,MaxX,MinZ,MaxZ,MinY,MaxY,Complete");
            poses = new StreamWriter(Path.Combine(outputDirectory, "trajectory.csv"));
            poses.WriteLine("Step,Time,X,Y,Z,Qx,Qy,Qz,Qw,Vx,Vy,Vz,GroundY,Clearance,Phase");
            wallStart = Time.realtimeSinceStartup;
            SaveManifest();
            status = "RUNNING: " + manifest.caseName;
        }
        catch (Exception e) { Fail(e); }
    }

    private void BuildTerrain()
    {
        if (Terrain.activeTerrains.Length != 0)
            throw new InvalidOperationException("Run in the generated empty experiment scene, not over an existing terrain.");
        var generator = new GameObject("Original Perlin TerrainGenerator").AddComponent<TerrainGenerator>();
        generator.generateOnStart = false;
        generator.terrainWidth = size; generator.terrainLength = size;
        generator.terrainHeight = relief; generator.heightmapResolution = resolution;
        generator.baseHeight = 0; generator.seed = seed; generator.noiseScale = noiseScale;
        generator.octaves = octaves; generator.persistence = persistence; generator.lacunarity = lacunarity;
        generator.treeCount = 0;
        // Call the existing two geometric stages, without invoking tree creation.
        typeof(TerrainGenerator).GetMethod("CreateTerrainObject", Fields).Invoke(generator, null);
        typeof(TerrainGenerator).GetMethod("GenerateHeightmap", Fields).Invoke(generator, null);
        terrain = Terrain.activeTerrain;
        if (terrain == null) throw new InvalidOperationException("TerrainGenerator created no active terrain.");
        TerrainData td = terrain.terrainData;
        if (flatControl) td.SetHeights(0, 0, new float[resolution, resolution]);
        float[,] h = td.GetHeights(0, 0, resolution, resolution);
        double sum = 0, sum2 = 0; float low = float.MaxValue, high = float.MinValue;
        for (int z = 0; z < resolution; z++)
            for (int x = 0; x < resolution; x++)
            {
                float value = h[z,x] * td.size.y;
                sum += value; sum2 += value * value; low = Mathf.Min(low,value); high = Mathf.Max(high,value);
            }
        int count = resolution * resolution;
        float mean = (float)(sum / count);
        terrain.transform.position = new Vector3(-size / 2f, -mean, -size / 2f);
        terrain.name = flatControl ? "Flat_Control_420m" : "Bare_Perlin_420m";
        manifest.baseElevation = -mean;
        manifest.elevationMin = low - mean; manifest.elevationMax = high - mean;
        manifest.elevationMean = 0; manifest.elevationSD = (float)Math.Sqrt(Math.Max(0, sum2 / count - mean * mean));
        var slopes = new float[count]; double ss = 0, ss2 = 0;
        using (var f = new StreamWriter(Path.Combine(outputDirectory, "heightmap.csv")))
        {
            for (int z = 0; z < resolution; z++)
            {
                for (int x = 0; x < resolution; x++)
                {
                    if (x > 0) f.Write(",");
                    f.Write((h[z,x] * td.size.y - mean).ToString("R", Inv));
                    float slope = td.GetSteepness(x / (float)(resolution - 1), z / (float)(resolution - 1));
                    slopes[z * resolution + x] = slope; ss += slope; ss2 += slope * slope;
                }
                f.WriteLine();
            }
        }
        Array.Sort(slopes);
        manifest.slopeMeanDeg = (float)(ss / count);
        manifest.slopeRmsDeg = (float)Math.Sqrt(ss2 / count);
        manifest.slopeP95Deg = slopes[(int)(0.95 * (count - 1))];
        manifest.slopeMaxDeg = slopes[count-1];
        if (td.treeInstanceCount != 0) throw new InvalidOperationException("Unexpected trees in bare terrain.");
        Physics.SyncTransforms();
    }

    private void ConfigureScanner()
    {
        var craft = new GameObject("Original SPattern VLP16 Scanner");
        qa = craft.AddComponent<StreamingQualityAssessment>();
        qa.WindowSize = 50; qa.CCIBinCount = 4; qa.CCIMinBinFirings = 2;
        qa.useAbsoluteExpectation = true; qa.MaxDetectionDistance = 100; qa.VerticalFOVDeg = 30;
        qa.MaxScanAngleFromNadir = 89; qa.PDVRThreshold = float.PositiveInfinity;
        qa.ISCThreshold = float.PositiveInfinity; qa.CCIThreshold = -1;
        typeof(StreamingQualityAssessment).GetField("logAlertsToConsole", Fields).SetValue(qa, false);
        scanner = craft.AddComponent<SPatternScanFlightPhased>();
        Set("lines",16); Set("verticalFOV",30f); Set("maxDetectionDistance",100f);
        Set("rotationFrequency",10f); Set("maxScanAngleFromNadir",89f); Set("initialRotationPhase",0f);
        Set("firingsPerFixedUpdate",360); Set("qualityAssessor",qa);
        Set("targetHeight",50f); Set("cruiseSpeed",5f); Set("cruiseAcceleration",2f);
        Set("targetCruiseSpeedAfterAccel",15f); Set("vibrationSigma",0f); Set("randomSeed",1);
        Set("resetAccelPerSwath",true); Set("swathWidth",60f); Set("numberOfSwaths",5); Set("swathLength",300f);
        Set("turnRadius",30f); Set("liftMultiplier",1.05f); Set("maxBankAngle",30f); Set("bankSmoothSpeed",2f);
        Set("showInWorld",false); Set("showInUI",false); Set("enableRealtimeVisualization",false);
        Set("enablePointDisplay",false); Set("savePointCloud",false); Set("saveRangeMatrix",false); Set("saveTrajectory",true);
        scanner.TerrainFiringObserved += OnFiring;
        scanner.TerrainReturnObserved += OnReturn;
    }
    private void Set(string name, object value)
    {
        FieldInfo field = typeof(SPatternScanFlightPhased).GetField(name, Fields);
        if (field == null) throw new MissingFieldException(name);
        field.SetValue(scanner,value);
    }
    private object Get(string name) { return typeof(SPatternScanFlightPhased).GetField(name, Fields).GetValue(scanner); }

    private void OnFiring(float time, Vector3 position, Quaternion rotation, float omega, bool facing)
    {
        totalFirings++;
        if (facing)
        {
            groundFirings++;
            if (!inside)
            {
                sweepIndex++; nReturns = 0; nRays = 0; rangeSum = slopeSum = incidenceSum = 0;
                minRange = minX = minY = minZ = float.MaxValue;
                maxRange = maxX = maxY = maxZ = float.MinValue;
                sweepStart = time; firstPosition = position;
            }
            nRays += 16; sweepEnd = time; lastPosition = position;
        }
        else if (inside) WriteSweep(sweepIndex > 0);
        inside = facing;
    }
    private void OnReturn(Vector3 origin, Vector3 direction, RaycastHit hit)
    {
        Vector3 p = hit.point;
        points.Write(p.x); points.Write(p.y); points.Write(p.z); points.Write(sweepIndex);
        totalReturns++; nReturns++;
        rangeSum += hit.distance; minRange = Mathf.Min(minRange,hit.distance); maxRange = Mathf.Max(maxRange,hit.distance);
        slopeSum += Vector3.Angle(hit.normal,Vector3.up);
        incidenceSum += Vector3.Angle(hit.normal,-direction);
        minX = Mathf.Min(minX,p.x); maxX = Mathf.Max(maxX,p.x);
        minY = Mathf.Min(minY,p.y); maxY = Mathf.Max(maxY,p.y);
        minZ = Mathf.Min(minZ,p.z); maxZ = Mathf.Max(maxZ,p.z);
    }
    private float Ground(Vector3 p) { return terrain.SampleHeight(p) + terrain.transform.position.y; }
    private void WriteSweep(bool complete)
    {
        Vector3 p = (firstPosition + lastPosition) * 0.5f;
        float ground = Ground(p);
        var values = new object[] {sweepIndex,sweepStart,sweepEnd,qa.CurrentPhase,p.x,p.y,p.z,ground,p.y-ground,
            nRays,nReturns,nReturns>0?rangeSum/nReturns:0,minRange,maxRange,
            nReturns>0?slopeSum/nReturns:0,nReturns>0?incidenceSum/nReturns:0,minX,maxX,minZ,maxZ,minY,maxY,complete?1:0};
        sweepFile.WriteLine(string.Join(",",Array.ConvertAll(values,x=>Convert.ToString(x,Inv))));
        if (sweepIndex % 100 == 0)
        {
            sweepFile.Flush(); points.Flush(); poses.Flush();
            Debug.Log("[Terrain] " + manifest.caseName + " sweep=" + sweepIndex + " points=" + totalReturns);
        }
    }
    // Executes before the scanner's next physics update and observes the current pose.
    private void FixedUpdate()
    {
        if (finished || scanner == null) return;
        if (Get("currentState").ToString() == "Completed") return;
        Vector3 p = scanner.transform.position; Quaternion q = scanner.transform.rotation;
        Rigidbody rb = scanner.GetComponent<Rigidbody>(); Vector3 v = rb.velocity;
        float ground = Ground(p);
        var values = new object[] {fixedSteps++,Time.fixedTime,p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.x,v.y,v.z,ground,p.y-ground,qa.CurrentPhase};
        poses.WriteLine(string.Join(",",Array.ConvertAll(values,x=>Convert.ToString(x,Inv))));
    }
    private void Update()
    {
        if (finished || scanner == null) return;
        if (Get("currentState").ToString() == "Completed") Finish();
        else if (Time.fixedTime > 600) Fail(new TimeoutException("Mission exceeds 600 simulated seconds."));
    }
    private void Finish()
    {
        if (totalReturns != Convert.ToInt64(Get("totalPointsCollected")))
        {
            Fail(new InvalidOperationException("Observer and scanner point counts disagree."));
            return;
        }
        if (inside) { WriteSweep(false); inside = false; }
        finished = true;
        manifest.returnCount = totalReturns; manifest.firingCount = totalFirings;
        manifest.groundFacingFiringCount = groundFirings;
        manifest.recordedSweeps = sweepIndex + 1; manifest.physicsSteps = fixedSteps;
        manifest.simulationEndTime = Time.fixedTime; manifest.wallSeconds = Time.realtimeSinceStartup - wallStart;
        string session = (string)Get("sessionId");
        string source = Path.GetFullPath(Path.Combine(Application.dataPath,"..","PointClouds",session));
        manifest.originalScannerOutput = source;
        if (Directory.Exists(source))
            foreach (string f in Directory.GetFiles(source))
                if (!Path.GetFileName(f).Contains("PointCloud") && !Path.GetFileName(f).Contains("RangeMatrix"))
                    File.Copy(f,Path.Combine(outputDirectory,Path.GetFileName(f)),true);
        manifest.status = "complete"; Close(); SaveManifest();
        status = "COMPLETE: " + outputDirectory;
        Debug.Log("[Terrain] " + status);
    }
    private static string Hash(string file)
    {
        using (var sha = SHA256.Create())
            return BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(Path.Combine(Application.dataPath,"Scripts",file)))).Replace("-","").ToLowerInvariant();
    }
    private void SaveManifest() { File.WriteAllText(Path.Combine(outputDirectory,"manifest.json"),JsonUtility.ToJson(manifest,true)); }
    private void Fail(Exception e)
    {
        finished = true; status = "FAILED: " + e;
        if (scanner != null) scanner.StopScanning();
        Close();
        if (manifest != null) {manifest.status = status; SaveManifest();}
        Debug.LogError(status);
    }
    private void Close()
    {
        points?.Dispose(); points = null; sweepFile?.Dispose(); sweepFile = null; poses?.Dispose(); poses = null;
    }
    private void OnDisable()
    {
        if (scanner != null) { scanner.TerrainFiringObserved -= OnFiring; scanner.TerrainReturnObserved -= OnReturn; }
        Close();
        if (!finished && manifest != null) {manifest.status = "incomplete"; SaveManifest();}
        Time.captureDeltaTime = 0;
    }
}
