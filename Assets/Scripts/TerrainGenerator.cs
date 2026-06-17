using System.Collections.Generic;
using UnityEngine;

public class TerrainGenerator : MonoBehaviour
{
    [Header("地形尺寸")]
    public int terrainWidth = 256;
    public int terrainLength = 256;
    public int terrainHeight = 50;
    public float baseHeight = 0f;       // 地形基础海拔（米）
    public int heightmapResolution = 257; // 必须是2^n+1

    [Header("Perlin噪声参数")]
    public float noiseScale = 20f;
    public int octaves = 4;
    public float persistence = 0.5f;
    public float lacunarity = 2f;
    public int seed = 42;

    [Header("树木参数")]
    public GameObject[] treePrefabs; // 在Inspector中拖入树木预制体
    public int treeCount = 500;
    public float minTreeHeight = 0.8f;
    public float maxTreeHeight = 1.2f;
    public float minTreeWidth = 0.8f;
    public float maxTreeWidth = 1.2f;
    [Range(0f, 1f)]
    public float minSpawnHeight = 0.1f; // 最低生成高度比例
    [Range(0f, 1f)]
    public float maxSpawnHeight = 0.7f; // 最高生成高度比例

    public float treeMinAltitude = 5f;   // 树木最低生成海拔（米）
    public float treeMaxAltitude = 35f;  // 树木最高生成海拔（米）
    public float treeMinSlope = 0f;      // 最小坡度（度）
    public float treeMaxSlope = 30f;     // 最大坡度（度），陡坡不长树


    [Header("生成控制")]
    public bool generateOnStart = true;
    //public Vector3 terrainPosition = new Vector3(-128, 0, -30);

    private Terrain terrain;
    private TerrainData terrainData;

    void Start()
    {
        if (generateOnStart)
        {
            GenerateTerrain();
        }
    }

    [ContextMenu("生成地形")]
    public void GenerateTerrain()
    {
        CreateTerrainObject();
        GenerateHeightmap();
        PlaceTrees();
        Debug.Log("地形和树木生成完成");
    }

    void CreateTerrainObject()
    {
        // 创建TerrainData
        terrainData = new TerrainData();
        terrainData.heightmapResolution = heightmapResolution;
        terrainData.size = new Vector3(terrainWidth, terrainHeight, terrainLength);

        // 创建Terrain GameObject
        GameObject terrainObj = Terrain.CreateTerrainGameObject(terrainData);
        terrainObj.name = "GeneratedTerrain";

        // 修改这一行：让地形中心在原点
        // Terrain的position是左下角，所以X和Z都要偏移负的一半
        terrainObj.transform.position = new Vector3(
            -terrainWidth / 2f,      // X方向偏移
            baseHeight,               // Y方向使用基础海拔
            -terrainLength / 2f       // Z方向偏移
        );

        terrain = terrainObj.GetComponent<Terrain>();

        // 添加Collider以支持Raycast
        TerrainCollider collider = terrainObj.GetComponent<TerrainCollider>();
        if (collider == null)
        {
            collider = terrainObj.AddComponent<TerrainCollider>();
        }
        collider.terrainData = terrainData;
    }

    void GenerateHeightmap()
    {
        float[,] heights = new float[heightmapResolution, heightmapResolution];
        System.Random prng = new System.Random(seed);

        // 每个octave的随机偏移
        Vector2[] octaveOffsets = new Vector2[octaves];
        for (int i = 0; i < octaves; i++)
        {
            float offsetX = prng.Next(-10000, 10000);
            float offsetY = prng.Next(-10000, 10000);
            octaveOffsets[i] = new Vector2(offsetX, offsetY);
        }

        float maxNoiseHeight = float.MinValue;
        float minNoiseHeight = float.MaxValue;

        // 第一遍：生成噪声值
        float[,] noiseMap = new float[heightmapResolution, heightmapResolution];

        for (int y = 0; y < heightmapResolution; y++)
        {
            for (int x = 0; x < heightmapResolution; x++)
            {
                float amplitude = 1f;
                float frequency = 1f;
                float noiseHeight = 0f;

                for (int i = 0; i < octaves; i++)
                {
                    float sampleX = (x / (float)heightmapResolution * noiseScale + octaveOffsets[i].x) * frequency;
                    float sampleY = (y / (float)heightmapResolution * noiseScale + octaveOffsets[i].y) * frequency;

                    float perlinValue = Mathf.PerlinNoise(sampleX, sampleY) * 2 - 1;
                    noiseHeight += perlinValue * amplitude;

                    amplitude *= persistence;
                    frequency *= lacunarity;
                }

                noiseMap[x, y] = noiseHeight;
                maxNoiseHeight = Mathf.Max(maxNoiseHeight, noiseHeight);
                minNoiseHeight = Mathf.Min(minNoiseHeight, noiseHeight);
            }
        }

        // 第二遍：归一化到[0,1]
        for (int y = 0; y < heightmapResolution; y++)
        {
            for (int x = 0; x < heightmapResolution; x++)
            {
                heights[y, x] = Mathf.InverseLerp(minNoiseHeight, maxNoiseHeight, noiseMap[x, y]);
            }
        }

        terrainData.SetHeights(0, 0, heights);
    }

    void PlaceTrees()
    {
        if (treePrefabs == null || treePrefabs.Length == 0)
        {
            Debug.LogWarning("未指定树木预制体，使用程序生成的简单树木");
            CreateProceduralTreePrototype();
        }
        else
        {
            TreePrototype[] treePrototypes = new TreePrototype[treePrefabs.Length];
            for (int i = 0; i < treePrefabs.Length; i++)
            {
                treePrototypes[i] = new TreePrototype();
                treePrototypes[i].prefab = treePrefabs[i];
            }
            terrainData.treePrototypes = treePrototypes;
        }

        List<TreeInstance> treeList = new List<TreeInstance>();
        System.Random prng = new System.Random(seed + 1);

        int attempts = 0;
        int maxAttempts = treeCount * 5;

        while (treeList.Count < treeCount && attempts < maxAttempts)
        {
            attempts++;

            float x = (float)prng.NextDouble();
            float z = (float)prng.NextDouble();

            // 获取该位置的实际海拔高度
            float normalizedHeight = terrainData.GetInterpolatedHeight(x, z) / terrainHeight;
            float actualAltitude = baseHeight + normalizedHeight * terrainHeight;

            // 检查海拔是否在允许范围内
            if (actualAltitude < treeMinAltitude || actualAltitude > treeMaxAltitude)
                continue;

            // 检查坡度（可选）
            float slope = terrainData.GetSteepness(x, z);
            if (slope < treeMinSlope || slope > treeMaxSlope)
                continue;

            TreeInstance tree = new TreeInstance();
            tree.position = new Vector3(x, normalizedHeight, z);
            tree.prototypeIndex = prng.Next(0, terrainData.treePrototypes.Length);
            tree.widthScale = Mathf.Lerp(minTreeWidth, maxTreeWidth, (float)prng.NextDouble());
            tree.heightScale = Mathf.Lerp(minTreeHeight, maxTreeHeight, (float)prng.NextDouble());
            tree.color = Color.white;
            tree.lightmapColor = Color.white;

            treeList.Add(tree);
        }

        terrainData.treeInstances = treeList.ToArray();
        Debug.Log($"放置了 {treeList.Count} 棵树木（尝试 {attempts} 次）");
    }

    void CreateProceduralTreePrototype()
    {
        // 创建简单的程序化树木
        GameObject treeObj = new GameObject("ProceduralTree");

        // 树干
        GameObject trunk = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        trunk.transform.parent = treeObj.transform;
        trunk.transform.localScale = new Vector3(0.3f, 2f, 0.3f);
        trunk.transform.localPosition = new Vector3(0, 2f, 0);
        trunk.GetComponent<Renderer>().material.color = new Color(0.4f, 0.25f, 0.1f);

        // 树冠
        GameObject canopy = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        canopy.transform.parent = treeObj.transform;
        canopy.transform.localScale = new Vector3(3f, 4f, 3f);
        canopy.transform.localPosition = new Vector3(0, 5f, 0);
        canopy.GetComponent<Renderer>().material.color = new Color(0.1f, 0.5f, 0.1f);

        // 设置为树木原型
        TreePrototype[] prototypes = new TreePrototype[1];
        prototypes[0] = new TreePrototype();
        prototypes[0].prefab = treeObj;
        terrainData.treePrototypes = prototypes;

        // 隐藏原始对象（Terrain会实例化它）
        treeObj.SetActive(false);
        DontDestroyOnLoad(treeObj);
    }

    [ContextMenu("清除地形")]
    public void ClearTerrain()
    {
        if (terrain != null)
        {
            DestroyImmediate(terrain.gameObject);
            terrain = null;
            terrainData = null;
        }
    }
}