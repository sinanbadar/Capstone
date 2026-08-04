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

    void Start()
    {
        commandSocket = new UdpClient(8889);
        receiveThread = new Thread(ReceiveCommands);
        receiveThread.IsBackground = true;
        receiveThread.Start();
        Debug.Log("TelloSimulator listening on port 8889");
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
                SendResponse("ok", sender);
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
            byte[] data = Encoding.UTF8.GetBytes(response);
            commandSocket.Send(data, data.Length, target);
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
                    StartCoroutine(SmoothMove(transform.forward, fDist * 0.5f));
                break;

            case "back":
                if (isFlying && parts.Length > 1 && int.TryParse(parts[1], out int bDist))
                    StartCoroutine(SmoothMove(-transform.forward, bDist * 0.5f));
                break;

            case "left":
                if (isFlying && parts.Length > 1 && int.TryParse(parts[1], out int lDist))
                    StartCoroutine(SmoothMove(-transform.right, lDist * 0.5f));
                break;

            case "right":
                if (isFlying && parts.Length > 1 && int.TryParse(parts[1], out int rDist))
                    StartCoroutine(SmoothMove(transform.right, rDist * 0.5f));
                break;

            case "up":
                if (isFlying && parts.Length > 1 && int.TryParse(parts[1], out int uDist))
                    StartCoroutine(SmoothMove(Vector3.up, uDist * 0.5f));
                break;

            case "down":
                if (isFlying && parts.Length > 1 && int.TryParse(parts[1], out int dDist))
                    StartCoroutine(SmoothMove(Vector3.down, dDist * 0.5f));
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
        Vector3 startPos = transform.position;
        Vector3 targetPos = startPos + direction * distance;
        float duration = distance / moveSpeed;
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
    }
}