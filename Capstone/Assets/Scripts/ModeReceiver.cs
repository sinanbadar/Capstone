using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Collections.Concurrent;
using Newtonsoft.Json;

public class ModeReceiver : MonoBehaviour
{
    public ControlModeUI controlModeUI;
    public int modePort = 9997;

    private UdpClient udpClient;
    private Thread receiveThread;
    private ConcurrentQueue<string> messageQueue = new ConcurrentQueue<string>();

    [System.Serializable]
    private class ModeMessage
    {
        public string mode;
    }

    void Start()
    {
        try
        {
            udpClient = new UdpClient(modePort);
            receiveThread = new Thread(ReceiveModes);
            receiveThread.IsBackground = true;
            receiveThread.Start();
            Debug.Log("Mode receiver listening on port " + modePort);
        }
        catch (System.Exception e)
        {
            Debug.LogWarning("Mode receiver error: " + e.Message);
        }
    }

    void ReceiveModes()
    {
        IPEndPoint sender = new IPEndPoint(IPAddress.Any, 0);
        while (true)
        {
            try
            {
                byte[] data = udpClient.Receive(ref sender);
                string json = Encoding.UTF8.GetString(data);
                messageQueue.Enqueue(json);
            }
            catch (System.Exception e)
            {
                Debug.LogWarning("Mode receive error: " + e.Message);
                break;
            }
        }
    }

    void Update()
    {
        while (messageQueue.TryDequeue(out string json))
        {
            try
            {
                ModeMessage msg = JsonConvert.DeserializeObject<ModeMessage>(json);
                if (msg != null && controlModeUI != null)
                {
                    UpdateMode(msg.mode);
                }
            }
            catch (System.Exception e)
            {
                Debug.LogWarning("Mode parse error: " + e.Message);
            }
        }
    }

    void UpdateMode(string mode)
    {
        switch (mode)
        {
            case "MANUAL":
                controlModeUI.SetMode(ControlModeUI.ControlMode.Manual);
                break;
            case "AUTONOMOUS":
                controlModeUI.SetMode(ControlModeUI.ControlMode.Autonomous);
                break;
            case "GAZE":
                controlModeUI.SetMode(ControlModeUI.ControlMode.Gaze);
                break;
        }
    }

    void OnDestroy()
    {
        udpClient?.Close();
    }
}