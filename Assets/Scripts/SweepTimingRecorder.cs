using System;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using UnityEngine;

// Opt-in diagnostic, not an alternative metric implementation.
// All wall durations use the same monotonic Stopwatch clock.
public sealed class SweepTimingRecorder : IDisposable
{
    private StreamWriter writer;
    private long firstTick, assessmentTicks, commitTick, readyTick;
    private double firstSim, commitSim;
    private bool active, pending;
    private int recordId;

    public SweepTimingRecorder()
    {
        try
        {
            string dir = Path.Combine(Application.dataPath, "..", "PointClouds",
                "TimingDiagnostics", DateTime.Now.ToString("yyyyMMdd_HHmmss_fff")
                + "_" + Guid.NewGuid().ToString("N").Substring(0, 8));
            Directory.CreateDirectory(dir);
            writer = new StreamWriter(Path.Combine(dir, "SweepTiming.csv"));
            writer.WriteLine("RecordId,SweepIndex,Window,Phase,FirstFiringSimS,CommitFiringSimS,CollectionSimMs,FirstFiringToComputedWallMs,CommitToComputedMs,GroundFiringAssessmentMs,AssessmentPlusCommitMs");
        }
        catch (Exception ex) when (ex is IOException || ex is UnauthorizedAccessException)
        {
            UnityEngine.Debug.LogWarning("Sweep timing output unavailable: " + ex.Message);
        }
    }

    public void Begin(double simulatedTime, long wallTick)
    {
        firstSim = simulatedTime;
        firstTick = wallTick;
        assessmentTicks = 0;
        pending = false;
        active = true;
    }

    public void AddAssessmentTicks(long ticks)
    {
        if (active) assessmentTicks += ticks;
    }

    public void StartCommit(double simulatedTime)
    {
        if (!active) return;
        commitSim = simulatedTime;
        commitTick = Stopwatch.GetTimestamp();
        pending = true;
    }

    public void ResultReady()
    {
        if (pending) readyTick = Stopwatch.GetTimestamp();
    }

    public void WriteCompleted(int sweep, int window, string phase)
    {
        if (!active || !pending) return;
        active = false;
        pending = false;
        if (writer == null) return;
        double scale = 1000.0 / Stopwatch.Frequency;
        double commitMs = (readyTick - commitTick) * scale;
        try
        {
            writer.WriteLine(string.Format(CultureInfo.InvariantCulture,
                "{0},{1},{2},\"{3}\",{4:R},{5:R},{6:R},{7:R},{8:R},{9:R},{10:R}",
                ++recordId, sweep, Math.Max(2, window), (phase ?? "").Replace("\"", "\"\""),
                firstSim, commitSim, (commitSim - firstSim) * 1000.0,
                (readyTick - firstTick) * scale, commitMs,
                assessmentTicks * scale, assessmentTicks * scale + commitMs));
            if ((recordId & 63) == 0) writer.Flush();
        }
        catch (Exception ex) when (ex is IOException || ex is UnauthorizedAccessException)
        {
            UnityEngine.Debug.LogWarning("Sweep timing output stopped: " + ex.Message);
            Dispose();
        }
    }

    public void Dispose()
    {
        if (writer == null) return;
        try { writer.Dispose(); }
        catch (IOException ex) { UnityEngine.Debug.LogWarning("Sweep timing close: " + ex.Message); }
        finally { writer = null; }
    }
}
