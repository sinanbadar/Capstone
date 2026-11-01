using UnityEngine;
using UnityEngine.UI;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Collections.Concurrent;

public class MapReceiver : MonoBehaviour
{
    [Header("Display")]
    public RawImage wallImage;          // transparent overlay, child of the map content object
    public RectTransform droneMarker;   // arrow or dot, same parent as wallImage, anchors centred
    public Text statsText;              // optional

    [Header("Map")]
    public int port = 9994;
    public float halfExtentM = 10f;     // map shows +/- this many metres from the start point
    public int texSize = 256;
    public Color wallColor = new Color(1f, 0.35f, 0.2f, 1f);

    [System.Serializable]
    class MapMessage
    {
        public float[] walls;
        public float drone_x, drone_z, heading;
        public float explored_m2;
        public int frontiers;
    }

    UdpClient client;
    Thread thread;
    ConcurrentQueue<string> queue = new ConcurrentQueue<string>();
    Texture2D tex;
    Color32[] pixels;

    void Start()
    {
        tex = new Texture2D(texSize, texSize, TextureFormat.RGBA32, false);
        tex.filterMode = FilterMode.Point;
        pixels = new Color32[texSize * texSize];
        wallImage.texture = tex;
        wallImage.raycastTarget = false;

        client = new UdpClient(port);
        thread = new Thread(Receive) { IsBackground = true };
        thread.Start();
        Debug.Log("MapReceiver listening on " + port);
    }

    void Receive()
    {
        IPEndPoint ep = new IPEndPoint(IPAddress.Any, 0);
        while (true)
        {
            try { queue.Enqueue(Encoding.UTF8.GetString(client.Receive(ref ep))); }
            catch { break; }
        }
    }

    void Update()
    {
        string latest = null;
        while (queue.TryDequeue(out string s)) latest = s;   // keep only the newest
        if (latest == null) return;

        MapMessage m = JsonUtility.FromJson<MapMessage>(latest);
        if (m == null || m.walls == null) return;
        Draw(m);
    }

    Vector2Int ToPixel(float x, float z)
    {
        float nx = x / halfExtentM * 0.5f + 0.5f;
        float nz = z / halfExtentM * 0.5f + 0.5f;
        return new Vector2Int(Mathf.RoundToInt(nx * (texSize - 1)),
                              Mathf.RoundToInt(nz * (texSize - 1)));
    }

    void Draw(MapMessage m)
    {
        for (int i = 0; i < pixels.Length; i++) pixels[i] = new Color32(0, 0, 0, 0);

        Color32 c = wallColor;
        for (int i = 0; i + 1 < m.walls.Length; i += 2)
        {
            Vector2Int p = ToPixel(m.walls[i], m.walls[i + 1]);
            for (int dx = -1; dx <= 1; dx++)            // 3x3 block, roughly one 0.25m cell
                for (int dy = -1; dy <= 1; dy++)
                {
                    int px = p.x + dx, py = p.y + dy;
                    if (px >= 0 && px < texSize && py >= 0 && py < texSize)
                        pixels[py * texSize + px] = c;
                }
        }
        tex.SetPixels32(pixels);
        tex.Apply();

        if (droneMarker != null)
        {
            Rect r = wallImage.rectTransform.rect;
            droneMarker.anchoredPosition = new Vector2(
                Mathf.Clamp(m.drone_x / halfExtentM, -1f, 1f) * r.width * 0.5f,
                Mathf.Clamp(m.drone_z / halfExtentM, -1f, 1f) * r.height * 0.5f);
            droneMarker.localEulerAngles = new Vector3(0, 0, -m.heading);   // heading is clockwise from "up"
        }

        if (statsText != null)
            statsText.text = $"Explored {m.explored_m2:0} m2\nOpen edges {m.frontiers}";
    }

    void OnDestroy() { client?.Close(); }
}