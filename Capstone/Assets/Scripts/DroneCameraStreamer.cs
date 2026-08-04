using UnityEngine;
using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;

public class DroneCameraStreamer : MonoBehaviour
{
    public RenderTexture droneRenderTexture;
    public string pythonHost = "127.0.0.1";
    public int videoPort = 11111;
    public int frameRate = 15;

    private UdpClient videoSocket;
    private Texture2D frameTexture;
    private float frameInterval;
    private float timer = 0f;
    private IPEndPoint pythonEndpoint;

    void Start()
    {
        frameInterval = 1f / frameRate;
        frameTexture = new Texture2D(
            droneRenderTexture.width,
            droneRenderTexture.height,
            TextureFormat.RGB24, false);
        videoSocket = new UdpClient();
        pythonEndpoint = new IPEndPoint(
            IPAddress.Parse(pythonHost), videoPort);
        Debug.Log("Camera streamer ready");
    }

    void Update()
    {
        timer += Time.deltaTime;
        if (timer >= frameInterval)
        {
            timer = 0f;
            StreamFrame();
        }
    }

    void StreamFrame()
    {
        try
        {
            RenderTexture.active = droneRenderTexture;
            frameTexture.ReadPixels(
                new Rect(0, 0,
                droneRenderTexture.width,
                droneRenderTexture.height), 0, 0);
            frameTexture.Apply();
            RenderTexture.active = null;

            byte[] jpegBytes = frameTexture.EncodeToJPG(60);
            videoSocket.Send(
                jpegBytes, jpegBytes.Length, pythonEndpoint);
        }
        catch (Exception e)
        {
            Debug.LogWarning("Stream error: " + e.Message);
        }
    }

    void OnDestroy()
    {
        videoSocket?.Close();
    }
}