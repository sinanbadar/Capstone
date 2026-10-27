using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Collections.Concurrent;

public class AlertReceiver : MonoBehaviour
{
    public GameObject alertPanel;
    public int port = 9995;

    // Set by ModeReceiver when the mode changes
    public static bool manualActive = false;

    UdpClient client;
    Thread thread;
    ConcurrentQueue<bool> queue = new ConcurrentQueue<bool>();

    [System.Serializable]
    class AlertMessage { public bool active; }

    void Start()
    {
        if (alertPanel != null) alertPanel.SetActive(false);
        client = new UdpClient(port);
        thread = new Thread(Receive) { IsBackground = true };
        thread.Start();
    }

    void Receive()
    {
        IPEndPoint ep = new IPEndPoint(IPAddress.Any, 0);
        while (true)
        {
            try
            {
                string json = Encoding.UTF8.GetString(client.Receive(ref ep));
                AlertMessage m = JsonUtility.FromJson<AlertMessage>(json);
                if (m != null) queue.Enqueue(m.active);
            }
            catch { break; }
        }
    }

    void Update()
    {
        while (queue.TryDequeue(out bool active))
        {
            // Only show while not in manual, and ignore repeat packets after the operator took over
            if (active && !manualActive && alertPanel != null)
                alertPanel.SetActive(true);
        }
    }

    public void Hide()
    {
        if (alertPanel != null) alertPanel.SetActive(false);
    }

    void OnDestroy() { client?.Close(); }
}