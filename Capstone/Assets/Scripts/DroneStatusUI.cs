using UnityEngine;
using TMPro;
using UnityEngine.UI;
using System.Linq;
using System.Text;

public class DroneStatusUI : MonoBehaviour
{
    [Header("Battery")]
    public Slider batterySlider;
    public TextMeshProUGUI batteryText;
    public Image batteryFill;

    [Header("Flight Info")]
    public TextMeshProUGUI heightText;
    public TextMeshProUGUI speedText;
    public TextMeshProUGUI distanceTravelledText;
    public TextMeshProUGUI flightTimeText;
    public TextMeshProUGUI statusText;

    [Header("Detections")]
    public TextMeshProUGUI detectionsText;

    [Header("References")]
    public DetectionReceiver detectionReceiver;

    [Header("Detection display")]
    public int maxDetectionLines = 3;
    public float detectionRefreshSeconds = 0.25f;
    private float detectionTimer = 0f;

    [Header("Hide during demo")]
    public GameObject[] hideDuringDemo;

    // ── DEMO MODE ────────────────────────────────────────
    [Header("Demo Mode (live-tweakable in Play mode)")]
    public bool demoMode = true;

    public enum DroneStatus { Idle, Flying, Searching, Returning }
    public DroneStatus status = DroneStatus.Searching;

    [Range(0f, 100f)] public float battery = 87f;
    [Range(0f, 1.7f)] public float height = 1.5f;
    [Range(0f, 2f)] public float speed = 0.4f;
    [Range(0f, 500f)] public float distanceTravelled = 0f;
    [Range(0f, 1800f)] public float flightTimeSeconds = 0f; 
    public void UpdateFromTelemetry(float posX, float posY, float posZ, bool flying)
    {
        if (demoMode) return; // ignore real telemetry while demoing

        status = flying ? DroneStatus.Flying : DroneStatus.Idle;
        height = Mathf.Min(posY, maxHeight);
    }

    [Header("Auto animate (untick to control by slider)")]
    public bool autoBattery = true;
    public bool autoHeight = true;
    public bool autoSpeed = true;
    public bool autoDistance = true;
    public bool autoTimer = true;

    [Header("Demo tuning")]
    public float maxHeight = 1.7f;
    public float baseHeight = 1.5f;
    public float heightWobble = 0.08f;
    public float batteryDrainPerSecond = 0.03f;
    public Vector2 speedRange = new Vector2(0.25f, 0.6f);
    // ─────────────────────────────────────────────────────

    void Update()
    {
        if (demoMode) AnimateDemo();

        RenderBattery();
        RenderFlightInfo();
        RenderStatus();
        UpdateDetections();
    }

    void AnimateDemo()
    {
        float t = Time.time;

        if (autoTimer)
            flightTimeSeconds += Time.deltaTime;

        if (autoBattery)
            battery = Mathf.Max(0f, battery - batteryDrainPerSecond * Time.deltaTime);

        if (autoHeight)
        {
            float wobble = (Mathf.PerlinNoise(t * 0.3f, 1f) - 0.5f) * 2f * heightWobble;
            height = baseHeight + wobble;
        }

        if (autoSpeed)
        {
            float n = Mathf.PerlinNoise(t * 0.2f, 7f);
            speed = Mathf.Lerp(speedRange.x, speedRange.y, n);
        }

        if (autoDistance)
            distanceTravelled += speed * Time.deltaTime;

        height = Mathf.Clamp(height, 0f, maxHeight);
    }
    void LateUpdate()
    {
        if (hideDuringDemo == null) return;
        foreach (var g in hideDuringDemo)
            if (g != null && g.activeSelf) g.SetActive(false);
    }
    void RenderBattery()
    {
        if (batterySlider != null)
            batterySlider.value = battery / 100f;

        if (batteryText != null)
            batteryText.text = $"{Mathf.RoundToInt(battery)}%";

        if (batteryFill != null)
        {
            if (battery > 50f) batteryFill.color = Color.green;
            else if (battery > 25f) batteryFill.color = Color.yellow;
            else batteryFill.color = Color.red;
        }
    }

    void RenderFlightInfo()
    {
        if (heightText != null)
            heightText.text = $"{height:F1}m";

        if (speedText != null)
            speedText.text = $"{speed:F1} m/s";

        if (distanceTravelledText != null)
            distanceTravelledText.text = $"{distanceTravelled:F1}m";

        if (flightTimeText != null)
        {
            int minutes = Mathf.FloorToInt(flightTimeSeconds / 60f);
            int seconds = Mathf.FloorToInt(flightTimeSeconds % 60f);
            flightTimeText.text = $"{minutes:00}:{seconds:00}";
        }
    }

    void RenderStatus()
    {
        if (statusText == null) return;

        switch (status)
        {
            case DroneStatus.Idle:
                statusText.text = "IDLE";
                statusText.color = Color.gray;
                break;
            case DroneStatus.Flying:
                statusText.text = "FLYING";
                statusText.color = Color.green;
                break;
            case DroneStatus.Searching:
                statusText.text = "SEARCHING";
                statusText.color = Color.cyan;
                break;
            case DroneStatus.Returning:
                statusText.text = "RETURNING";
                statusText.color = Color.yellow;
                break;
        }
    }

    void UpdateDetections()
    {
        if (detectionsText == null) return;
        if (detectionReceiver == null) return;
        if (!detectionReceiver.newDetectionsAvailable) return;

        detectionTimer += Time.deltaTime;
        if (detectionTimer < detectionRefreshSeconds) return;
        detectionTimer = 0f;

        var dets = detectionReceiver.latestDetections;
        if (dets.Count == 0)
        {
            detectionsText.text = "None";
            return;
        }

        var groups = dets
            .GroupBy(d => d.label)
            .Select(g => new { label = g.Key, count = g.Count(), conf = g.Max(d => d.confidence) })
            .OrderByDescending(g => g.conf)
            .ToList();

        var sb = new StringBuilder();
        int shown = Mathf.Min(maxDetectionLines, groups.Count);
        for (int i = 0; i < shown; i++)
        {
            string countText = groups[i].count > 1 ? $" x{groups[i].count}" : "";
            sb.AppendLine($"{groups[i].label}{countText} {Mathf.RoundToInt(groups[i].conf * 100f)}%");
        }
        if (groups.Count > shown)
            sb.Append($"+{groups.Count - shown} more");

        detectionsText.text = sb.ToString().TrimEnd();
    }

    // ── PUBLIC SETTERS (for later real hookup) ───────────
    public void SetStatus(DroneStatus s) { status = s; }
    public void SetBattery(float pct) { battery = pct; }
    public void SetHeight(float m) { height = Mathf.Min(m, maxHeight); }
    public void SetSpeed(float mps) { speed = mps; }
    public void AddDistance(float d) { distanceTravelled += d; }
}