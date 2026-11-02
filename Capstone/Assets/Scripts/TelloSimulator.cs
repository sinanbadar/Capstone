using UnityEngine;
using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Collections;
using System.Collections.Concurrent;

public class TelloSimulator : MonoBehaviour
{
    public float moveSpeed = 5f;
    public float rotateSpeed = 120f;
    public float verticalSpeed = 3f;
    private Vector4 rcVelocity = Vector4.zero;

    private UdpClient commandSocket;
    private Thread receiveThread;
    private ConcurrentQueue<string> commandQueue = new ConcurrentQueue<string>();
    private bool isFlying = false;
    private bool isMoving = false;
    private IPEndPoint lastSender;

    private UdpClient telemetrySocket;
    private IPEndPoint pythonTelemetryEndpoint;
    public int telemetryPort = 9998;
    public float telemetryRate = 0.1f;
    private float telemetryTimer = 0f;

    public DroneStatusUI droneStatusUI;

    [Header("RC movement (used by autonomous and controller)")]
    public float rcMoveSpeed = 0.03f;      // units per second for each rc unit (forward and sideways)
    public float rcVerticalSpeed = 0.03f;  // units per second for each rc unit (up and down)
    public float rcYawSpeed = 1.2f;        // degrees per second for each rc unit

    public LayerMask obstacleMask = ~0;   // set to the layers of walls and furniture, NOT the floor
    private int collisionCount = 0;
    private bool inContact = false;

    float ForwardDistance()
    {
        var hits = Physics.RaycastAll(transform.position, transform.forward, 40f, obstacleMask);
        float best = 40f;
        foreach (var h in hits)
        {
            if (h.transform.IsChildOf(transform)) continue;
            if (h.distance < best) best = h.distance;
        }
        return best;
    }

    void Start()
    {
        commandSocket = new UdpClient(8889);
        receiveThread = new Thread(ReceiveCommands);
        receiveThread.IsBackground = true;
        receiveThread.Start();
        Debug.Log("TelloSimulator listening on port 8889");

        telemetrySocket = new UdpClient();
        pythonTelemetryEndpoint = new IPEndPoint(IPAddress.Parse("127.0.0.1"), telemetryPort);
        Debug.Log("Telemetry sender ready on port " + telemetryPort);
    }

    void ReceiveCommands()
    {
        IPEndPoint sender = new IPEndPoint(IPAddress.Any, 0);
        while (true)
        {
            try
            {
                byte[] data = commandSocket.Receive(ref sender);
                lastSender = sender;
                string command = Encoding.UTF8.GetString(data).Trim();
                Debug.Log("Received: " + command);
                commandQueue.Enqueue(command);
                if (!command.StartsWith("rc"))
                {
                    Debug.Log("Received: " + command);
                    SendResponse(command, sender);
                }
            }
            catch (Exception e)
            {
                Debug.LogWarning("Receive error: " + e.Message);
            }
        }
    }

    void SendResponse(string response, IPEndPoint target)
    {
        try
        {
            byte[] okData = Encoding.UTF8.GetBytes("ok");
            commandSocket.Send(okData, okData.Length, target);
        }
        catch (Exception e)
        {
            Debug.LogWarning("Send error: " + e.Message);
        }
    }

    void Update()
    {
        while (commandQueue.TryDequeue(out string command))
        {
            ExecuteCommand(command);
        }

        if (isFlying && rcVelocity != Vector4.zero)
        {
            transform.Translate(
                rcVelocity.x * rcMoveSpeed * Time.deltaTime,       // left/right
                rcVelocity.z * rcVerticalSpeed * Time.deltaTime,   // up/down
                rcVelocity.y * rcMoveSpeed * Time.deltaTime,       // forward/back
                Space.Self);
            transform.Rotate(0, rcVelocity.w * rcYawSpeed * Time.deltaTime, 0);
        }

        telemetryTimer += Time.deltaTime;
        if (telemetryTimer >= telemetryRate)
        {
            telemetryTimer = 0f;
            SendTelemetry();
        }

        if (isFlying)
        {
            bool touching = false;
            foreach (var c in Physics.OverlapSphere(transform.position, 0.3f, obstacleMask))
                if (!c.transform.IsChildOf(transform)) { touching = true; break; }
            if (touching && !inContact) collisionCount++;
            inContact = touching;

            transform.position += Vector3.up *
                Mathf.Sin(Time.time * 2f) * 0.00002f;
        }
    }

    void SendTelemetry()
    {
        if (lastSender == null)
        {
            return;
        }
        try
        {
            TelemetryData telemetry = new TelemetryData
            {
                pos_x = transform.position.x,
                pos_y = transform.position.y,
                pos_z = transform.position.z,
                rot_x = transform.eulerAngles.x,
                rot_y = transform.eulerAngles.y,
                rot_z = transform.eulerAngles.z,
                is_flying = isFlying,
                timestamp = Time.time,
                // in SendTelemetry, inside the object initialiser
                fwd_dist = ForwardDistance(),
                collisions = collisionCount
            };

            string json = JsonUtility.ToJson(telemetry);
            byte[] data = Encoding.UTF8.GetBytes(json);
            telemetrySocket.Send(data, data.Length,
                new IPEndPoint(lastSender.Address, telemetryPort));
        }
        catch (Exception e)
        {
            Debug.LogWarning("Telemetry send error: " + e.Message);
        }
    }

    void ExecuteCommand(string command)
    {
        string[] parts = command.Split(' ');
        string cmd = parts[0].ToLower();

        switch (cmd)
        {
            case "command":
                Debug.Log("SDK mode enabled");
                break;

            case "rc":
                if (isFlying && parts.Length == 5)
                {
                    int.TryParse(parts[1], out int rcLR);
                    int.TryParse(parts[2], out int rcFB);
                    int.TryParse(parts[3], out int rcUD);
                    int.TryParse(parts[4], out int rcYaw);

                    rcVelocity = new Vector4(rcLR, rcFB, rcUD, rcYaw);
                }
                break;

            case "rc 0 0 0 0":
                rcVelocity = Vector4.zero;
                break;

            case "takeoff":
                isFlying = true;
                StartCoroutine(SmoothMove(Vector3.up, 1.0f));
                Debug.Log("Takeoff");
                if (droneStatusUI != null)
                    droneStatusUI.SetStatus(DroneStatusUI.DroneStatus.Flying);
                break;

            case "land":
                isFlying = false;
                StartCoroutine(SmoothMove(Vector3.down, 1.0f));
                Debug.Log("Land");
                if (droneStatusUI != null)
                    droneStatusUI.SetStatus(DroneStatusUI.DroneStatus.Idle);
                break;

            case "forward":
                if (isFlying && parts.Length > 1 && int.TryParse(parts[1], out int fDist))
                    StartCoroutine(SmoothMove(transform.TransformDirection(Vector3.forward), fDist * 0.05f));
                break;

            case "back":
                if (isFlying && parts.Length > 1 && int.TryParse(parts[1], out int bDist))
                    StartCoroutine(SmoothMove(transform.TransformDirection(Vector3.back), bDist * 0.05f));
                break;

            case "left":
                if (isFlying && parts.Length > 1 && int.TryParse(parts[1], out int lDist))
                    StartCoroutine(SmoothMove(transform.TransformDirection(Vector3.left), lDist * 0.05f));
                break;

            case "right":
                if (isFlying && parts.Length > 1 && int.TryParse(parts[1], out int rDist))
                    StartCoroutine(SmoothMove(transform.TransformDirection(Vector3.right), rDist * 0.05f));
                break;

            case "up":
                if (isFlying && parts.Length > 1 && int.TryParse(parts[1], out int uDist))
                    StartCoroutine(SmoothMove(Vector3.up, uDist * 0.05f));
                break;

            case "down":
                if (isFlying && parts.Length > 1 && int.TryParse(parts[1], out int dDist))
                    StartCoroutine(SmoothMove(Vector3.down, dDist * 0.05f));
                break;

            case "cw":
                if (isFlying && parts.Length > 1 && int.TryParse(parts[1], out int cwDeg))
                    StartCoroutine(SmoothRotate(cwDeg));
                break;

            case "ccw":
                if (isFlying && parts.Length > 1 && int.TryParse(parts[1], out int ccwDeg))
                    StartCoroutine(SmoothRotate(-ccwDeg));
                break;

            case "emergency":
                isFlying = false;
                StopAllCoroutines();
                Debug.Log("Emergency stop");
                break;

            default:
                Debug.Log("Unknown command: " + command);
                break;
        }
    }

    IEnumerator SmoothMove(Vector3 direction, float distance)
    {
        // Lock direction at start of movement
        Vector3 lockedDirection = direction.normalized;
        Vector3 startPos = transform.position;
        Vector3 targetPos = startPos + lockedDirection * distance;
        float duration = Mathf.Max(distance / moveSpeed, 0.5f);
        float elapsed = 0f;

        while (elapsed < duration)
        {
            elapsed += Time.deltaTime;
            float t = Mathf.SmoothStep(0f, 1f, elapsed / duration);
            transform.position = Vector3.Lerp(startPos, targetPos, t);
            yield return null;
        }

        transform.position = targetPos;
    }

    IEnumerator SmoothRotate(float degrees)
    {
        float startY = transform.eulerAngles.y;
        float targetY = startY + degrees;
        float duration = Mathf.Abs(degrees) / rotateSpeed;
        float elapsed = 0f;

        while (elapsed < duration)
        {
            elapsed += Time.deltaTime;
            float t = Mathf.SmoothStep(0f, 1f, elapsed / duration);
            float currentY = Mathf.LerpAngle(startY, targetY, t);
            transform.eulerAngles = new Vector3(
                transform.eulerAngles.x,
                currentY,
                transform.eulerAngles.z);
            yield return null;
        }

        transform.eulerAngles = new Vector3(
            transform.eulerAngles.x,
            targetY,
            transform.eulerAngles.z);
    }

    void OnDestroy()
    {
        receiveThread?.Abort();
        commandSocket?.Close();
        telemetrySocket?.Close();
    }
}

[System.Serializable]
public class TelemetryData
{
    public float pos_x;
    public float pos_y;
    public float pos_z;
    public float rot_x;
    public float rot_y;
    public float rot_z;
    public bool is_flying;
    public float timestamp;
    // in TelemetryData
    public float fwd_dist;
    public int collisions;

    
}