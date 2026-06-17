using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

/// <summary>
/// 在 Editor 中复用 RAMI 重建森林 prefab，自动检测 XZ patch 尺寸并平铺成大场景。
/// </summary>
public class RamiSceneTiler : MonoBehaviour
{
    [Header("===== 源 =====")]
    [Tooltip("把上一步生成的 RAMI_Scene 保存为 prefab 后拖到这里")]
    public GameObject scenePrefab;

    [Header("===== 自动检测 patch 尺寸 =====")]
    [Tooltip("instances.txt 在 Assets 下的相对路径")]
    public string instancesAssetPath = "RAMI/HET09_JBS_SUM/Parameters/instances.txt";
    [Tooltip("objects_boundingbox.txt path under Assets")]
    public string bboxAssetPath = "RAMI/HET09_JBS_SUM/Parameters/objects_boundingbox.txt";

    [Tooltip("自动检测时给 patch 尺寸加的安全余量（米），通常用 0 或 1")]
    public float autoDetectPadding = 0f;

    [Header("===== Patch 尺寸（XZ 可不同） =====")]
    public float patchSizeX = 100f;
    public float patchSizeZ = 100f;

    [Tooltip("拼接偏移：每个 tile 内部要平移多少才能让源 prefab 的 (minX,minZ) 对齐到 tile 原点。" +
             "通常 = (-minX, -minZ)，可以用 Auto Detect 自动算出。")]
    public Vector2 sourceOriginOffset = Vector2.zero;

    [Header("===== 平铺数量 =====")]
    public int tilesAlongX = 5;
    public int tilesAlongZ = 5;
    public bool centerAtOrigin = true;

    [Header("===== Collider =====")]
    public bool ensureMeshColliders = false;

    public const string TileRootName = "RAMI_Tiles";

    /// <summary>
    /// 从 instances.txt 解析出 LESS 实例的 XY 范围，并写入 patchSizeX/Z 和 sourceOriginOffset。
    /// LESS (X,Y) → Unity (X,Z)
    /// </summary>
    /// <summary>
    /// Unified entry: try objects_boundingbox.txt first (mesh-style scenes carry
    /// their full extent in mesh geometry); if the bbox file is missing or its
    /// extent is degenerate, fall back to instances.txt (instance-style scenes
    /// carry extent in the spread of instance positions). One button for all scenes.
    /// </summary>
    public void AutoDetectExtent()
    {
        if (TryExtentFromPrefab(out float fx, out float fz, out Vector2 foff))
        {
            patchSizeX = fx + autoDetectPadding;
            patchSizeZ = fz + autoDetectPadding;
            sourceOriginOffset = foff;
            Debug.Log($"[RamiTiler] extent from PREFAB: patchX={patchSizeX:F2} patchZ={patchSizeZ:F2} offset={sourceOriginOffset}");
        }
        else if (TryExtentFromBoundingBox(out float bx, out float bz, out Vector2 boff))
        {
            patchSizeX = bx + autoDetectPadding;
            patchSizeZ = bz + autoDetectPadding;
            sourceOriginOffset = boff;
            Debug.Log($"[RamiTiler] extent from BOUNDINGBOX: patchX={patchSizeX:F2} patchZ={patchSizeZ:F2} offset={sourceOriginOffset}");
        }
        else if (TryExtentFromInstances(out float ix, out float iz, out Vector2 ioff))
        {
            patchSizeX = ix + autoDetectPadding;
            patchSizeZ = iz + autoDetectPadding;
            sourceOriginOffset = ioff;
            Debug.Log($"[RamiTiler] extent from INSTANCES (bbox unavailable): patchX={patchSizeX:F2} patchZ={patchSizeZ:F2} offset={sourceOriginOffset}");
        }
        else
        {
            Debug.LogError("[RamiTiler] extent detection failed: neither bbox nor instances usable.");
            return;
        }
#if UNITY_EDITOR
        EditorUtility.SetDirty(this);
#endif
    }

    /// Try parse objects_boundingbox.txt. Returns false if missing/empty/degenerate.
    /// Most robust: read the prefab's actual Unity-space Renderer bounds (after mesh
    /// import + RAMISceneBuilder coord conversion). No guessing which file column maps
    /// to which axis -- uses the real geometry as it sits in Unity.
    private bool TryExtentFromPrefab(out float patchX, out float patchZ, out Vector2 offset)
    {
        patchX = patchZ = 0f; offset = Vector2.zero;
        if (scenePrefab == null) return false;
        var rends = scenePrefab.GetComponentsInChildren<Renderer>(true);
        if (rends.Length == 0) return false;
        Bounds b = rends[0].bounds;
        foreach (var r in rends) b.Encapsulate(r.bounds);
        patchX = b.size.x;
        patchZ = b.size.z;
        // offset so the prefab's (minX,minZ) aligns to the tile origin
        offset = new Vector2(-b.min.x, -b.min.z);
        Debug.Log($"[RamiTiler] extent from PREFAB bounds: size=({patchX:F1},{patchZ:F1}) min=({b.min.x:F1},{b.min.z:F1}) offset={offset}");
        return patchX > 0.5f && patchZ > 0.5f;
    }


    private bool TryExtentFromBoundingBox(out float patchX, out float patchZ, out Vector2 offset)
    {
        patchX = patchZ = 0f; offset = Vector2.zero;
        string fullPath = Path.Combine(Application.dataPath, bboxAssetPath);
        if (!File.Exists(fullPath)) return false;

        var ci = CultureInfo.InvariantCulture;
        float[] mn = { float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity };
        float[] mx = { float.NegativeInfinity, float.NegativeInfinity, float.NegativeInfinity };
        int n = 0;
        foreach (string raw in File.ReadAllLines(fullPath))
        {
            string line = raw.Trim();
            if (string.IsNullOrEmpty(line) || !line.Contains(":")) continue;
            string vals = line.Substring(line.IndexOf(':') + 1);
            string[] p = vals.Split(new[] { ' ', '\t' }, System.StringSplitOptions.RemoveEmptyEntries);
            if (p.Length < 6) continue;
            float[] v = new float[6]; bool ok = true;
            for (int k = 0; k < 6; k++)
                if (!float.TryParse(p[k], NumberStyles.Float, ci, out v[k])) { ok = false; break; }
            if (!ok) continue;
            for (int axis = 0; axis < 3; axis++)
            {
                mn[axis] = Mathf.Min(mn[axis], Mathf.Min(v[axis], v[axis + 3]));
                mx[axis] = Mathf.Max(mx[axis], Mathf.Max(v[axis], v[axis + 3]));
            }
            n++;
        }
        if (n == 0) return false;

        float[] range = { mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2] };
        // height axis = smallest range; other two are horizontal
        int hAxis = 0;
        if (range[1] < range[hAxis]) hAxis = 1;
        if (range[2] < range[hAxis]) hAxis = 2;
        int ax0 = -1, ax1 = -1;
        for (int k = 0; k < 3; k++) { if (k == hAxis) continue; if (ax0 < 0) ax0 = k; else ax1 = k; }
        // degenerate guard: horizontal extent must be non-trivial
        if (range[ax0] < 0.5f || range[ax1] < 0.5f) return false;

        patchX = range[ax0];
        patchZ = range[ax1];
        offset = new Vector2(-mn[ax0], -mn[ax1]);
        return true;
    }


    private bool TryExtentFromInstances(out float patchX, out float patchZ, out Vector2 offset)
    {
        patchX = patchZ = 0f; offset = Vector2.zero;
        string fullPath = Path.Combine(Application.dataPath, instancesAssetPath);
        if (!File.Exists(fullPath))
        {
            Debug.LogError($"[RamiTiler] 找不到 instances.txt: {fullPath}");
            return false;
        }

        float minX = float.PositiveInfinity, maxX = float.NegativeInfinity;
        float minY = float.PositiveInfinity, maxY = float.NegativeInfinity;
        var ci = CultureInfo.InvariantCulture;
        int n = 0;

        foreach (string raw in File.ReadAllLines(fullPath))
        {
            string line = raw.Trim();
            if (string.IsNullOrEmpty(line) || line.StartsWith("#")) continue;
            string[] p = line.Split(new[] { ' ', '\t' },
                System.StringSplitOptions.RemoveEmptyEntries);
            if (p.Length < 5) continue;

            if (!float.TryParse(p[1], NumberStyles.Float, ci, out float x)) continue;
            if (!float.TryParse(p[2], NumberStyles.Float, ci, out float y)) continue;

            if (x < minX) minX = x;
            if (x > maxX) maxX = x;
            if (y < minY) minY = y;
            if (y > maxY) maxY = y;
            n++;
        }

        if (n == 0)
        {
            Debug.LogError("[RamiTiler] instances.txt 没解析到任何实例。");
            return false;
        }

        float rangeX = maxX - minX;
        float rangeZ = maxY - minY;   // LESS Y → Unity Z

        patchX = rangeX;
        patchZ = rangeZ;

        // 让原 prefab 的 (minX, minY) 对齐到 tile 原点 (0,0)
        // 注意 LESS Y → Unity Z，所以这里第二维用 minY
        offset = new Vector2(-minX, -minY);

        Debug.Log($"[RamiTiler] 自动检测完成（共 {n} 个实例）：\n" +
                  $"  X 范围 [{minX:F2}, {maxX:F2}]，宽度 {rangeX:F2} m\n" +
                  $"  Y 范围 [{minY:F2}, {maxY:F2}]，宽度 {rangeZ:F2} m\n" +
                  $"  → patchSizeX = {patchSizeX:F2}, patchSizeZ = {patchSizeZ:F2}\n" +
                  $"  → sourceOriginOffset = {sourceOriginOffset}");

#if UNITY_EDITOR
        EditorUtility.SetDirty(this);
#endif
        return true;
    }

    public void BuildTiles()
    {
        if (scenePrefab == null)
        {
            Debug.LogError("[RamiTiler] scenePrefab 没有指定。");
            return;
        }

        ClearTiles();

        Transform root = new GameObject(TileRootName).transform;
        root.SetParent(transform, false);

        // 整张大地图的左下角（在 root 局部空间下）
        float xStart = centerAtOrigin ? -(tilesAlongX * patchSizeX) * 0.5f : 0f;
        float zStart = centerAtOrigin ? -(tilesAlongZ * patchSizeZ) * 0.5f : 0f;

        int count = 0;
        for (int ix = 0; ix < tilesAlongX; ix++)
        {
            for (int iz = 0; iz < tilesAlongZ; iz++)
            {
                // 每个 tile 的左下角
                float tileX = xStart + ix * patchSizeX;
                float tileZ = zStart + iz * patchSizeZ;

                // 加上 source 内部的对齐偏移，让源 prefab 的 (minX,minZ) 落到 tile 左下角
                float px = tileX + sourceOriginOffset.x;
                float pz = tileZ + sourceOriginOffset.y;

                GameObject inst;
#if UNITY_EDITOR
                inst = (GameObject)PrefabUtility.InstantiatePrefab(scenePrefab, root);
#else
                inst = Instantiate(scenePrefab, root);
#endif
                inst.name = $"tile_{ix}_{iz}";
                inst.transform.localPosition = new Vector3(px, 0f, pz);
                inst.transform.localRotation = Quaternion.identity;

                if (ensureMeshColliders)
                {
                    foreach (var mf in inst.GetComponentsInChildren<MeshFilter>())
                    {
                        if (mf.sharedMesh != null && mf.GetComponent<MeshCollider>() == null)
                        {
                            var mc = mf.gameObject.AddComponent<MeshCollider>();
                            mc.sharedMesh = mf.sharedMesh;
                        }
                    }
                }
                count++;
            }
        }

        Debug.Log($"[RamiTiler] 已生成 {count} 个 tile，覆盖 " +
                  $"{tilesAlongX * patchSizeX:F0} x {tilesAlongZ * patchSizeZ:F0} m");

#if UNITY_EDITOR
        UnityEditor.SceneManagement.EditorSceneManager.MarkSceneDirty(
            UnityEditor.SceneManagement.EditorSceneManager.GetActiveScene());
#endif
    }

    public void ClearTiles()
    {
        Transform existing = transform.Find(TileRootName);
        if (existing != null)
        {
#if UNITY_EDITOR
            DestroyImmediate(existing.gameObject);
#else
            Destroy(existing.gameObject);
#endif
        }
    }
}

#if UNITY_EDITOR
[CustomEditor(typeof(RamiSceneTiler))]
public class RamiSceneTilerEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();
        var tiler = (RamiSceneTiler)target;

        GUILayout.Space(10);

        if (GUILayout.Button("Auto Detect Extent (bbox, fallback instances)",
                             GUILayout.Height(28)))
        {
            tiler.AutoDetectExtent();
        }

        GUILayout.Space(6);
        if (GUILayout.Button("Build Tiles", GUILayout.Height(34)))
        {
            tiler.BuildTiles();
        }
        if (GUILayout.Button("Clear Tiles"))
        {
            tiler.ClearTiles();
        }
    }
}
#endif