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
                SendResponse(command, sender);
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

        telemetryTimer += Time.deltaTime;
        if (telemetryTimer >= telemetryRate)
        {
            telemetryTimer = 0f;
            SendTelemetry();
        }

        if (isFlying)
        {
            transform.position += Vector3.up *
                Mathf.Sin(Time.time * 2f) * 0.002f;
        }
    }

    void SendTelemetry()
    {
        if (lastSender == null)
        {
            Debug.Log("SendTelemetry: lastSender is null, waiting for first command");
            return;
        }
        Debug.Log($"Sending telemetry to {lastSender.Address}:{telemetryPort} pos:{transform.position}");
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
                timestamp = Time.time
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

            case "takeoff":
                isFlying = true;
                StartCoroutine(SmoothMove(Vector3.up, 1.0f));
                Debug.Log("Takeoff");
                break;

            case "land":
                isFlying = false;
                StartCoroutine(SmoothMove(Vector3.down, 1.0f));
                Debug.Log("Land");
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
}