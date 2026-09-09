using UnityEngine;
using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Collections.Concurrent;

// Listens on UNITY_RESPONSE_PORT for JSON sent by bridge.py and
// pushes it into DroneStatusUI. Attach alongside DroneStatusUI
// and DetectionReceiver on the same GameObject (or wire the
// reference in the inspector).
public class TelloBridgeReceiver : MonoBehaviour
{
    public DroneStatusUI droneStatusUI;

    [Header("Network")]
    public int listenPort = 8891; // must match UNITY_RESPONSE_PORT in bridge.py

    private UdpClient udpClient;
    private Thread receiveThread;
    private readonly ConcurrentQueue<string> messageQueue = new ConcurrentQueue<string>();
    private volatile bool running = false;

    [Serializable]
    private class DroneStateMessage
    {
        public float pos_x;
        public float pos_y;
        public float pos_z;
        public bool is_flying;
        public int battery;
        public float height_m;
        public float speed_mps;
    }

    void Start()
    {
        udpClient = new UdpClient(listenPort);
        running = true;
        receiveThread = new Thread(ReceiveLoop);
        receiveThread.IsBackground = true;
        receiveThread.Start();
    }

    void ReceiveLoop()
    {
        IPEndPoint remoteEndPoint = new IPEndPoint(IPAddress.Any, 0);
        while (running)
        {
            try
            {
                byte[] data = udpClient.Receive(ref remoteEndPoint);
                string json = System.Text.Encoding.UTF8.GetString(data);
                messageQueue.Enqueue(json);
            }
            catch (SocketException)
            {
                // thrown on Close(), safe to ignore during shutdown
            }
        }
    }

    void Update()
    {
        while (messageQueue.TryDequeue(out string json))
        {
            ApplyState(json);
        }
    }

    void ApplyState(string json)
    {
        DroneStateMessage state;
        try
        {
            state = JsonUtility.FromJson<DroneStateMessage>(json);
        }
        catch (Exception e)
        {
            Debug.LogWarning($"Bad drone state JSON: {e.Message}");
            return;
        }

        if (droneStatusUI == null) return;

        droneStatusUI.SetBattery(state.battery);
        droneStatusUI.SetHeight(state.height_m);
        droneStatusUI.SetSpeed(state.speed_mps);
        droneStatusUI.UpdateFromTelemetry(state.pos_x, state.pos_y, state.pos_z, state.is_flying);

        var status = state.is_flying
            ? DroneStatusUI.DroneStatus.Flying
            : DroneStatusUI.DroneStatus.Idle;
        droneStatusUI.SetStatus(status);
    }

    void OnDestroy()
    {
        running = false;
        udpClient?.Close();
        receiveThread?.Join(200);
    }
}