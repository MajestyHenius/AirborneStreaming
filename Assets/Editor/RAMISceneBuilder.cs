using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEditor;
using UnityEngine;

public class RAMISceneBuilder : EditorWindow
{
    private string dataFolder = "RAMI/HET09_JBS_SUM/Parameters";
    private bool invertRotation = false;
    private bool fixXAxisUp = false;
    private bool enableGPUInstancing = true;
    private bool groupByType = true;
    private bool autoAlignGround = true;   // 新增：生成后自动贴地
    private string rootName = "RAMI_Scene";

    [MenuItem("RAMI/Build Scene from LESS")]
    public static void ShowWindow()
    {
        GetWindow<RAMISceneBuilder>("RAMI Scene Builder");
    }

    private void OnGUI()
    {
        GUILayout.Label("LESS / RAMI 场景重建工具", EditorStyles.boldLabel);
        EditorGUILayout.Space();

        dataFolder = EditorGUILayout.TextField("数据文件夹 (Assets/...)", dataFolder);
        rootName = EditorGUILayout.TextField("根物体名称", rootName);

        EditorGUILayout.Space();
        invertRotation = EditorGUILayout.Toggle("旋转取反 (-angle)", invertRotation);
        fixXAxisUp = EditorGUILayout.Toggle("修正 X 轴朝上 (-90°)", fixXAxisUp);
        enableGPUInstancing = EditorGUILayout.Toggle("启用 GPU Instancing", enableGPUInstancing);
        groupByType = EditorGUILayout.Toggle("按类型分组", groupByType);
        autoAlignGround = EditorGUILayout.Toggle("生成后自动贴地 (y=0)", autoAlignGround);

        EditorGUILayout.Space();
        if (GUILayout.Button("生成场景", GUILayout.Height(40)))
        {
            BuildScene();
        }

        EditorGUILayout.Space();
        if (GUILayout.Button("对齐到地面 (y=0)"))
        {
            AlignToGround();
        }

        if (GUILayout.Button("显示场景边界信息"))
        {
            PrintSceneBounds();
        }

        if (GUILayout.Button("清除已生成的场景"))
        {
            ClearScene();
        }

        EditorGUILayout.Space();
        if (GUILayout.Button("Convert to Colliders (remove renderers, add MeshColliders)", GUILayout.Height(34)))
        {
            ConvertToColliders();
        }

        EditorGUILayout.HelpBox(
            "坐标转换：LESS (X,Y,Z, Z-up) → Unity (X, Z, Y, Y-up)\n" +
            "贴地原理：扫描所有 Mesh 的世界包围盒，把根节点抬高 -min.y。",
            MessageType.Info);
    }

    private void BuildScene()
    {
        string fullFolder = Path.Combine(Application.dataPath, dataFolder);
        string objectsFile = Path.Combine(fullFolder, "objects.txt");
        string instancesFile = Path.Combine(fullFolder, "instances.txt");

        if (!File.Exists(objectsFile))
        {
            EditorUtility.DisplayDialog("错误", $"找不到 objects.txt: {objectsFile}", "OK");
            return;
        }
        if (!File.Exists(instancesFile))
        {
            EditorUtility.DisplayDialog("错误", $"找不到 instances.txt: {instancesFile}", "OK");
            return;
        }

        // 1. 解析 objects.txt
        Dictionary<string, string> nameToObjFile = new Dictionary<string, string>();
        foreach (string rawLine in File.ReadAllLines(objectsFile))
        {
            string line = rawLine.Trim();
            if (string.IsNullOrEmpty(line) || line.StartsWith("#")) continue;

            string[] parts = line.Split(new[] { ' ', '\t' }, System.StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length < 2) continue;

            nameToObjFile[parts[0]] = parts[1];
        }

        // 2. 加载 OBJ
        Dictionary<string, GameObject> nameToPrefab = new Dictionary<string, GameObject>();
        foreach (var kv in nameToObjFile)
        {
            string assetPath = "Assets/" + dataFolder.Replace("\\", "/") + "/" + kv.Value;
            GameObject prefab = AssetDatabase.LoadAssetAtPath<GameObject>(assetPath);
            if (prefab == null)
            {
                Debug.LogWarning($"找不到 OBJ 资源: {assetPath}");
                continue;
            }
            nameToPrefab[kv.Key] = prefab;

            if (enableGPUInstancing)
            {
                EnableInstancingOnPrefab(prefab);
            }
        }

        Debug.Log($"已加载 {nameToPrefab.Count} 种对象类型");

        // 3. 创建根节点
        GameObject root = GameObject.Find(rootName);
        if (root != null) DestroyImmediate(root);
        root = new GameObject(rootName);

        Dictionary<string, Transform> groupTransforms = new Dictionary<string, Transform>();

        int total = 0, missing = 0;
        var lines = File.ReadAllLines(instancesFile);
        var ci = CultureInfo.InvariantCulture;

        try
        {
            for (int i = 0; i < lines.Length; i++)
            {
                if (i % 200 == 0)
                {
                    EditorUtility.DisplayProgressBar("正在生成场景",
                        $"{i}/{lines.Length}", (float)i / lines.Length);
                }

                string line = lines[i].Trim();
                if (string.IsNullOrEmpty(line) || line.StartsWith("#")) continue;

                string[] parts = line.Split(new[] { ' ', '\t' },
                    System.StringSplitOptions.RemoveEmptyEntries);
                if (parts.Length < 5) continue;

                string typeName = parts[0];
                if (!nameToPrefab.TryGetValue(typeName, out GameObject prefab))
                {
                    missing++;
                    continue;
                }

                float lx = float.Parse(parts[1], ci);
                float ly = float.Parse(parts[2], ci);
                float lz = float.Parse(parts[3], ci);
                float angle = float.Parse(parts[4], ci);

                Vector3 pos = new Vector3(lx, lz, ly);
                float yRot = invertRotation ? -angle : angle;

                Transform parent = root.transform;
                if (groupByType)
                {
                    if (!groupTransforms.TryGetValue(typeName, out parent))
                    {
                        GameObject groupGo = new GameObject(typeName);
                        groupGo.transform.SetParent(root.transform, false);
                        parent = groupGo.transform;
                        groupTransforms[typeName] = parent;
                    }
                }

                GameObject inst = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
                inst.transform.SetParent(parent, false);
                inst.transform.localPosition = pos;

                Quaternion rot = Quaternion.Euler(0f, yRot, 0f);
                if (fixXAxisUp)
                {
                    rot = rot * Quaternion.Euler(-90f, 0f, 0f);
                }
                inst.transform.localRotation = rot;

                inst.name = $"{typeName}_{total}";
                total++;
            }
        }
        finally
        {
            EditorUtility.ClearProgressBar();
        }

        Debug.Log($"场景生成完成：共 {total} 个实例，跳过 {missing} 个未找到类型的实例。");

        if (autoAlignGround)
        {
            AlignToGround();
        }

        Selection.activeGameObject = root;
        SceneView.lastActiveSceneView?.FrameSelected();
    }

    /// <summary>
    /// 计算根节点下所有 Renderer 的整体包围盒
    /// </summary>
    private bool TryGetSceneBounds(GameObject root, out Bounds bounds)
    {
        bounds = new Bounds();
        var renderers = root.GetComponentsInChildren<Renderer>();
        if (renderers.Length == 0) return false;

        bool initialized = false;
        foreach (var r in renderers)
        {
            if (!initialized)
            {
                bounds = r.bounds;
                initialized = true;
            }
            else
            {
                bounds.Encapsulate(r.bounds);
            }
        }
        return initialized;
    }

    /// <summary>
    /// 把根节点整体抬高 -min.y，使最低点贴 y=0
    /// </summary>
    private void AlignToGround()
    {
        GameObject root = GameObject.Find(rootName);
        if (root == null)
        {
            Debug.LogWarning("找不到场景根节点，请先生成场景。");
            return;
        }

        if (!TryGetSceneBounds(root, out Bounds b))
        {
            Debug.LogWarning("场景中没有可计算包围盒的 Renderer。");
            return;
        }

        float hLow = b.min.y;
        Vector3 p = root.transform.position;
        // 抬高使最低点为 0
        root.transform.position = new Vector3(p.x, p.y - hLow, p.z);

        Debug.Log($"已对齐到地面：原最低点 y = {hLow:F4}，根节点上移 {-hLow:F4}。" +
                  $" 现场景 Y 范围：[0, {(b.size.y):F2}]");
    }

    private void PrintSceneBounds()
    {
        GameObject root = GameObject.Find(rootName);
        if (root == null)
        {
            Debug.LogWarning("找不到场景根节点。");
            return;
        }
        if (!TryGetSceneBounds(root, out Bounds b))
        {
            Debug.LogWarning("场景中无 Renderer。");
            return;
        }

        Debug.Log($"场景包围盒：\n" +
                  $"  min = {b.min}\n" +
                  $"  max = {b.max}\n" +
                  $"  size = {b.size}\n" +
                  $"  center = {b.center}\n" +
                  $"  最低点 y (h_low) = {b.min.y:F4}");
    }

    private void ClearScene()
    {
        GameObject root = GameObject.Find(rootName);
        if (root != null)
        {
            DestroyImmediate(root);
            Debug.Log("已清除场景。");
        }
    }

    /// <summary>
    /// For raycast-only use: remove all MeshRenderers (rendering is the cost on
    /// huge TLS meshes / many instances) and add a MeshCollider on every mesh so
    /// Physics.Raycast can still hit the geometry. Renderer affects display only;
    /// the scan needs colliders, not renderers.
    /// </summary>
    private void ConvertToColliders()
    {
        GameObject root = GameObject.Find(rootName);
        if (root == null) { Debug.LogWarning("No scene root; build the scene first."); return; }

        var filters = root.GetComponentsInChildren<MeshFilter>(true);
        int addedCol = 0, removedRend = 0;
        foreach (var mf in filters)
        {
            if (mf.sharedMesh == null) continue;

            // add MeshCollider if absent
            var mc = mf.GetComponent<MeshCollider>();
            if (mc == null)
            {
                mc = mf.gameObject.AddComponent<MeshCollider>();
                // >2^21 (2,097,152) triangle meshes break PhysX Fast Midphase -> raycast crash.
                // Disable Fast Midphase (keep other default cook opts) for stable collision.
                mc.cookingOptions = MeshColliderCookingOptions.EnableMeshCleaning
                                  | MeshColliderCookingOptions.WeldColocatedVertices
                                  | MeshColliderCookingOptions.CookForFasterSimulation;
                // (UseFastMidphase intentionally omitted)
                mc.sharedMesh = mf.sharedMesh;
                addedCol++;
            }

            // remove MeshRenderer (display only; not needed for raycast)
            var mr = mf.GetComponent<MeshRenderer>();
            if (mr != null) { DestroyImmediate(mr); removedRend++; }
        }

        Debug.Log($"[RAMI] ConvertToColliders done: added {addedCol} MeshColliders, removed {removedRend} MeshRenderers. Scene is now raycast-only (invisible).");
#if UNITY_EDITOR
        UnityEditor.SceneManagement.EditorSceneManager.MarkSceneDirty(
            UnityEditor.SceneManagement.EditorSceneManager.GetActiveScene());
#endif
    }

    private void EnableInstancingOnPrefab(GameObject prefab)
    {
        var renderers = prefab.GetComponentsInChildren<MeshRenderer>(true);
        foreach (var r in renderers)
        {
            foreach (var mat in r.sharedMaterials)
            {
                if (mat != null && !mat.enableInstancing)
                {
                    mat.enableInstancing = true;
                    EditorUtility.SetDirty(mat);
                }
            }
        }
    }
}