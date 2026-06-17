using UnityEngine;
using UnityEditor;
public class TerrainTreeToGameObject : MonoBehaviour
{
    [ContextMenu("Convert Terrain Trees to GameObjects")]
    public void ConvertTreesToGameObjects()
    {
        Terrain terrain = GetComponent<Terrain>();
        if (terrain == null) return;
        TerrainData terrainData = terrain.terrainData;
        TreeInstance[] trees = terrainData.treeInstances;
        TreePrototype[] prototypes = terrainData.treePrototypes;
        // 创建父对象
        GameObject treeParent = new GameObject("Converted_Trees");
        treeParent.transform.parent = terrain.transform;
        // 转换每棵树
        for (int i = 0; i < trees.Length; i++)
        {
            TreeInstance tree = trees[i];
            TreePrototype prototype = prototypes[tree.prototypeIndex];
            if (prototype.prefab != null)
            {
                // 计算世界坐标
                Vector3 worldPos = Vector3.Scale(tree.position, terrainData.size) + terrain.transform.position;
                // 实例化树
                GameObject treeObj = Instantiate(prototype.prefab, worldPos,
                    Quaternion.Euler(0, tree.rotation * Mathf.Rad2Deg, 0), treeParent.transform);
                // 缩放
                treeObj.transform.localScale = new Vector3(tree.widthScale, tree.heightScale, tree.widthScale);
                // 添加Mesh Collider到所有包含MeshRenderer的子对象
                MeshRenderer[] renderers = treeObj.GetComponentsInChildren<MeshRenderer>();
                foreach (MeshRenderer renderer in renderers)
                {
                    MeshFilter meshFilter = renderer.GetComponent<MeshFilter>();
                    if (meshFilter != null && meshFilter.sharedMesh != null)
                    {
                        // 如果没有Collider，添加MeshCollider
                        Collider existingCollider = renderer.GetComponent<Collider>();
                        if (existingCollider != null)
                        {
                            DestroyImmediate(existingCollider);
                        }
                        MeshCollider meshCollider = renderer.gameObject.AddComponent<MeshCollider>();
                        meshCollider.sharedMesh = meshFilter.sharedMesh;
                        // 如果树木网格复杂，可以考虑用convex
                        // meshCollider.convex = true;
                    }
                }
                // 添加层级标记（可选）
                treeObj.layer = LayerMask.NameToLayer("Default"); // 或创建专门的Tree层
            }
        }
        // 清除Terrain上的树（可选）
        // terrainData.treeInstances = new TreeInstance[0];
        Debug.Log($"转换了 {trees.Length} 棵树");
    }
}
