using System.Diagnostics;
using UnityEditor;
using UnityEngine;
using Debug = UnityEngine.Debug;

public class ColliderCookProbe
{
    [MenuItem("RAMI/Probe Cook Cost (bake all MeshColliders now)")]
    public static void ProbeCook()
    {
        string rootName = "RAMI_Scene";
        var root = GameObject.Find(rootName);
        if (root == null) { Debug.LogWarning("找不到场景根节点 " + rootName); return; }

        var cols = root.GetComponentsInChildren<MeshCollider>(true);
        Debug.Log($"[CookProbe] 共 {cols.Length} 个 MeshCollider，开始逐个 cook 计时…");

        long totalTris = 0;
        var swAll = Stopwatch.StartNew();
        int worstIdx = -1; double worstMs = 0; string worstName = "";

        for (int i = 0; i < cols.Length; i++)
        {
            var mc = cols[i];
            if (mc.sharedMesh == null) continue;
            long tris = mc.sharedMesh.triangles.Length / 3;
            totalTris += tris;

            // 强制重新 cook：把 sharedMesh 重新赋一遍，PhysX 会重建 BVH
            var mesh = mc.sharedMesh;
            var sw = Stopwatch.StartNew();
            mc.sharedMesh = null;
            mc.sharedMesh = mesh;     // ← 这一行触发 cook
            sw.Stop();

            double ms = sw.Elapsed.TotalMilliseconds;
            if (ms > worstMs) { worstMs = ms; worstIdx = i; worstName = mc.name; }
            if (ms > 50.0)
                Debug.LogWarning($"[CookProbe] 慢: {mc.name}  tris={tris}  cook={ms:F1} ms");
        }
        swAll.Stop();

        Debug.Log($"[CookProbe] 完成。总三角面={totalTris:N0}  " +
                  $"总耗时={swAll.Elapsed.TotalMilliseconds:F0} ms  " +
                  $"最慢: {worstName} ({worstMs:F1} ms)");
    }
}