using UnityEngine;
using UnityEngine.UI;
using System.Net;
using System.Net.Sockets;
using System.Collections.Generic;
using System.Threading;

public class VideoReceiver : MonoBehaviour
{
    public RawImage displayImage;
    public static bool isManualMode = true;

    private UdpClient udpClient;
    private Texture2D videoTexture;
    private byte[] pendingFrame;
    private bool frameReady = false;
    private Thread receiveThread;
    private Dictionary<int, byte[]> chunks = new();
    private int expectedTotal = 0;

    void Start()
    {
        videoTexture = new Texture2D(320, 240, TextureFormat.RGB24, false);
        displayImage.texture = videoTexture;
        displayImage.gameObject.SetActive(false);
        udpClient = new UdpClient(9093);
        receiveThread = new Thread(ReceiveLoop);
        receiveThread.IsBackground = true;
        receiveThread.Start();
    }

    void ReceiveLoop()
    {
        IPEndPoint ep = new IPEndPoint(IPAddress.Any, 9093);
        while (true)
        {
            try
            {
                byte[] data = udpClient.Receive(ref ep);

                int firstColon = System.Array.IndexOf(data, (byte)':');
                int secondColon = System.Array.IndexOf(data, (byte)':', firstColon + 1);

                int index = int.Parse(
                    System.Text.Encoding.ASCII.GetString(data, 0, firstColon));
                int total = int.Parse(
                    System.Text.Encoding.ASCII.GetString(
                        data, firstColon + 1, secondColon - firstColon - 1));

                byte[] payload = new byte[data.Length - secondColon - 1];
                System.Array.Copy(data, secondColon + 1, payload, 0, payload.Length);

                chunks[index] = payload;
                expectedTotal = total;

                if (chunks.Count == expectedTotal)
                {
                    System.IO.MemoryStream ms = new();
                    for (int i = 0; i < total; i++)
                        ms.Write(chunks[i], 0, chunks[i].Length);

                    pendingFrame = ms.ToArray();
                    frameReady = true;
                    chunks.Clear();
                }
            }
            catch { }
        }
    }

    void Update()
    {
        displayImage.gameObject.SetActive(isManualMode);

        if (isManualMode && frameReady && pendingFrame != null)
        {
            videoTexture.LoadImage(pendingFrame);
            videoTexture.Apply();
            frameReady = false;
        }
    }

    void OnDestroy()
    {
        receiveThread?.Abort();
        udpClient?.Close();
    }
}