using UnityEngine;
using TMPro;
using UnityEngine.UI;
using System.Collections.Generic;

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

    // ── MODE SWITCH ──────────────────────────────────────
    private bool USE_REAL_TELLO = false;
    // Set to true when testing with real drone
    // ─────────────────────────────────────────────────────

    private float flightTimer = 0f;
    private bool isFlying = false;

    // Simulation values
    private float fakeBattery = 100f;
    private Vector3 lastPosition = Vector3.zero;
    private float totalDistanceTravelled = 0f;
    private Vector3 currentVelocity = Vector3.zero;

    public enum DroneStatus { Idle, Flying, Searching, Returning }
    private DroneStatus currentStatus = DroneStatus.Idle;

    // Reference to drone transform for simulation
    public Transform droneTransform;

    void Start()
    {
        if (droneTransform != null)
            lastPosition = droneTransform.position;
    }

    void Update()
    {
        UpdateBattery();
        UpdateFlightMetrics();
        UpdateFlightTime();
        UpdateStatus();
        UpdateDetections();
    }

    void UpdateBattery()
    {
        if (USE_REAL_TELLO)
        {
            // ── REAL TELLO ────────────────────────────────
            // Call this from external script that has drone reference:
            // droneStatusUI.SetBattery(drone.get_battery());
            // ─────────────────────────────────────────────
        }
        else
        {
            // ── SIMULATION ───────────────────────────────
            fakeBattery -= Time.deltaTime * 0.1f;
            fakeBattery = Mathf.Clamp(fakeBattery, 0f, 100f);
            // ─────────────────────────────────────────────
        }

        if (batterySlider != null)
            batterySlider.value = fakeBattery / 100f;

        if (batteryText != null)
            batteryText.text = $"{Mathf.RoundToInt(fakeBattery)}%";

        if (batteryFill != null)
        {
            if (fakeBattery > 50f) batteryFill.color = Color.green;
            else if (fakeBattery > 25f) batteryFill.color = Color.yellow;
            else batteryFill.color = Color.red;
        }
    }

    public void UpdateFromTelemetry(float posX, float posY, float posZ, bool flying)
    {
        if (flying && !isFlying)
        {
            SetStatus(DroneStatus.Flying);
        }
        else if (!flying && isFlying)
        {
            SetStatus(DroneStatus.Idle);
        }

        if (heightText != null)
            heightText.text = $"{posY:F1}m";
    }

    void UpdateFlightTime()
    {
        if (!isFlying) return;
        flightTimer += Time.deltaTime;
        int minutes = Mathf.FloorToInt(flightTimer / 60f);
        int seconds = Mathf.FloorToInt(flightTimer % 60f);
        if (flightTimeText != null)
            flightTimeText.text = $"{minutes:00}:{seconds:00}";
    }

    void UpdateFlightMetrics()
    {
        if (droneTransform == null) return;

        if (USE_REAL_TELLO)
        {
            // ── REAL TELLO ────────────────────────────────────────
            // Height, speed come from Tello telemetry via djitellopy
            // Call these from your real Tello controller script:
            //
            // droneStatusUI.SetHeight(drone.get_height() / 100f);
            //
            // Tello reports speed in cm/s, convert to m/s:
            // float speedX = drone.get_speed_x() / 100f;
            // float speedY = drone.get_speed_y() / 100f;
            // float speedZ = drone.get_speed_z() / 100f;
            // float speed = Mathf.Sqrt(speedX*speedX + speedY*speedY + speedZ*speedZ);
            // droneStatusUI.SetSpeed(speed);
            //
            // Distance accumulated via AddDistance called each update:
            // droneStatusUI.AddDistance(distanceDelta);
            // ─────────────────────────────────────────────────────
        }
        else
        {
            // ── SIMULATION ────────────────────────────────────────
            // Height always visible even on ground
            if (heightText != null)
                heightText.text = $"{droneTransform.position.y:F1}m";

            if (!isFlying) return;

            // Speed from position delta
            Vector3 delta = droneTransform.position - lastPosition;
            float speed = delta.magnitude / Time.deltaTime;

            if (speedText != null)
                speedText.text = $"{speed:F1} m/s";

            // Accumulated distance travelled
            totalDistanceTravelled += delta.magnitude;
            if (distanceTravelledText != null)
                distanceTravelledText.text = $"{totalDistanceTravelled:F1}m";

            lastPosition = droneTransform.position;
            // ─────────────────────────────────────────────────────
        }
    }

    void UpdateStatus()
    {
        if (statusText == null) return;

        switch (currentStatus)
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

        if (detectionReceiver.latestDetections.Count == 0)
        {
            detectionsText.text = "None";
            return;
        }

        string output = "";
        foreach (var detection in detectionReceiver.latestDetections)
        {
            string conf = (detection.confidence * 100f).ToString("F0");
            output += $"{detection.label} {conf}%\n";
        }
        detectionsText.text = output.TrimEnd();
    }

    // ── PUBLIC SETTERS FOR REAL TELLO ────────────────────
    public void SetStatus(DroneStatus status)
    {
        currentStatus = status;
        isFlying = status != DroneStatus.Idle;
        if (status == DroneStatus.Idle)
        {
            flightTimer = 0f;
            totalDistanceTravelled = 0f;
        }
    }

    public void SetBattery(float percentage)
    {
        fakeBattery = percentage;
    }

    public void SetHeight(float metres)
    {
        if (heightText != null)
            heightText.text = $"{metres:F1}m";
    }

    public void SetSpeed(float metersPerSecond)
    {
        if (speedText != null)
            speedText.text = $"{metersPerSecond:F1} m/s";
    }

    public void AddDistance(float delta)
    {
        totalDistanceTravelled += delta;
        if (distanceTravelledText != null)
            distanceTravelledText.text = $"{totalDistanceTravelled:F1}m";
    }
    // ─────────────────────────────────────────────────────
}