using System.Collections.Generic;
using UnityEngine;

public class MetricLiveDisplay : MonoBehaviour
{
    public StreamingQualityAssessment assessor;
    public int maxPoints = 200;          // 曲线显示多少个历史点
    public float panelWidth = 400f;
    public float panelHeight = 90f;      // 每条曲线高度
    public float panelX = 20f;
    public float panelY = 20f;

    private Queue<float> pdvrBuf = new Queue<float>();
    private Queue<float> iscBuf = new Queue<float>();
    private Queue<float> cciBuf = new Queue<float>();

    private Material lineMat;

    void Start()
    {
        // 用于 GL 画线的简单材质
        Shader shader = Shader.Find("Hidden/Internal-Colored");
        lineMat = new Material(shader);
        lineMat.hideFlags = HideFlags.HideAndDontSave;
    }

    void Update()
    {
        if (assessor == null) return;
        var m = assessor.GetLatestMetrics();
        Push(pdvrBuf, m.pdvr);
        Push(iscBuf, m.isc);
        Push(cciBuf, m.cci);
    }

    void Push(Queue<float> buf, float v)
    {
        buf.Enqueue(v);
        while (buf.Count > maxPoints) buf.Dequeue();
    }

    void OnGUI()
    {
        DrawCurve(pdvrBuf, panelY, "PDVR", 0.2f, Color.yellow);
        DrawCurve(iscBuf, panelY + panelHeight + 10, "ISC", 0.3f, Color.cyan);
        DrawCurve(cciBuf, panelY + 2 * (panelHeight + 10), "CCI", 1.0f, Color.green);
    }

    void DrawCurve(Queue<float> buf, float y, string label, float yMax, Color col)
    {
        // 背景框
        GUI.Box(new Rect(panelX, y, panelWidth, panelHeight), label);

        if (buf.Count < 2) return;

        float[] vals = buf.ToArray();
        float x0 = panelX + 5;
        float w = panelWidth - 10;
        float h = panelHeight - 25;
        float yBase = y + panelHeight - 5;

        // 用 GUI 画线段（OnGUI 里没有原生画线，用许多小矩形近似，或用 GL）
        for (int i = 1; i < vals.Length; i++)
        {
            float xa = x0 + w * (i - 1) / (maxPoints - 1);
            float xb = x0 + w * i / (maxPoints - 1);
            float ya = yBase - h * Mathf.Clamp01(vals[i - 1] / yMax);
            float yb = yBase - h * Mathf.Clamp01(vals[i] / yMax);
            DrawLineSeg(new Vector2(xa, ya), new Vector2(xb, yb), col);
        }
    }

    // OnGUI 里画线段：用一个旋转的细长 GUI 贴图
    private static Texture2D _lineTex;
    void DrawLineSeg(Vector2 a, Vector2 b, Color col)
    {
        if (_lineTex == null)
        {
            _lineTex = new Texture2D(1, 1);
            _lineTex.SetPixel(0, 0, Color.white);
            _lineTex.Apply();
        }
        Matrix4x4 saved = GUI.matrix;
        float angle = Mathf.Atan2(b.y - a.y, b.x - a.x) * Mathf.Rad2Deg;
        float len = Vector2.Distance(a, b);
        GUIUtility.RotateAroundPivot(angle, a);
        Color old = GUI.color;
        GUI.color = col;
        GUI.DrawTexture(new Rect(a.x, a.y - 1, len, 2), _lineTex);
        GUI.color = old;
        GUI.matrix = saved;
    }
}