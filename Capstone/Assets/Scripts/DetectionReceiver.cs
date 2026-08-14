using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Collections.Generic;
using System.Collections.Concurrent;
using Newtonsoft.Json;

public class DetectionReceiver : MonoBehaviour
{
    public int detectionPort = 9999;
    private UdpClient udpClient;
    private Thread receiveThread;
    private ConcurrentQueue<string> messageQueue = new ConcurrentQueue<string>();

    [System.Serializable]
    public class BoundingBox
    {
        public float x1, y1, x2, y2;
    }

    [System.Serializable]
    public class Detection
    {
        public string label;
        public float confidence;
        public BoundingBox bbox;
    }

    public List<Detection> latestDetections = new List<Detection>();
    public bool newDetectionsAvailable = false;

    void Start()
    {
        udpClient = new UdpClient(detectionPort);
        receiveThread = new Thread(ReceiveDetections);
        receiveThread.IsBackground = true;
        receiveThread.Start();
        Debug.Log("Detection receiver listening on port " + detectionPort);
    }

    void ReceiveDetections()
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
                Debug.LogWarning("Detection receive error: " + e.Message);
            }
        }
    }

    void Update()
    {
        while (messageQueue.TryDequeue(out string json))
        {
            Debug.Log("RAW MESSAGE: " + json);
            latestDetections = JsonConvert.DeserializeObject<List<Detection>>(json);
            newDetectionsAvailable = true;
        }
    }

    void OnDestroy()
    {
        receiveThread?.Abort();
        udpClient?.Close();
    }
}