using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;

public class MinimapController : MonoBehaviour
{
    [Header("Minimap Display")]
    public RawImage minimapBackground;
    public RectTransform droneDot;
    public RectTransform operatorDot;

    [Header("Room Bounds")]
    public float roomMinX = -200f;
    public float roomMaxX = 200f;
    public float roomMinZ = -200f;
    public float roomMaxZ = 200f;

    [Header("References")]
    public Transform droneTransform;
    public Transform operatorTransform;

    [Header("Trail")]
    public int maxTrailPoints = 500;

    [Header("FOV Cone")]
    public float cameraFOV = 82.6f;
    public float cameraRange = 3.0f;

    [Header("Zoom and Pan")]
    public float zoomLevel = 1.0f;
    public float minZoom = 0.5f;
    public float maxZoom = 4.0f;
    private Vector2 panOffset = Vector2.zero;
    private bool isDragging = false;
    private Vector2 lastDragPos;

    [Header("Facing (assign your controller)")]
    public Transform facingSource;
    public bool rotateDotToFacing = true;

    [Header("Wall")]
    public float wallDistance = 1.5f;
    public float wallLength = 3f;
    [Range(1, 6)] public int wallThickness = 2;
    public Color wallColor = Color.white;

    [Header("Marker")]
    public Sprite markerSprite;
    public float markerDistance = 1.5f;
    public float markerSize = 24f;

    [Header("Tick to add (auto-unticks)")]
    public bool addWall;
    public bool addMarker;

    [Header("Demo Wander (moves droneTransform)")]
    public bool demoWander = true;
    public float demoSpeed = 0.5f;
    public float demoTurnRate = 90f;
    [Range(0.1f, 0.9f)] public float demoRadiusFraction = 0.4f;
    private float demoHeading = 0f;

    private List<GameObject> markers = new List<GameObject>();

    private List<Vector2> trailPoints = new List<Vector2>();
    private Texture2D minimapTexture;
    private int minimapSize = 256;

    private Vector3 lastDronePos;
    private List<ConeEntry> coneHistory = new List<ConeEntry>();
    private int maxConeHistory = 200;

    struct ConeEntry
    {
        public float worldX;
        public float worldZ;
        public float headingDeg;
    }

    void Start()
    {
        minimapTexture = new Texture2D(minimapSize, minimapSize);
        ClearTexture();
        minimapBackground.texture = minimapTexture;
        if (droneTransform != null)
            lastDronePos = droneTransform.position;
    }

    void Update()
    {

        if (addWall) { addWall = false; PlaceWall(); }
        if (addMarker) { addMarker = false; PlaceMarker(); }

        if (demoWander) DemoWander();
        UpdateFOVCone();
        UpdateDroneDot();
        UpdateOperatorDot();
        UpdateTrail();
        HandleZoomAndPan();
    }

    void DemoWander()
    {
        if (droneTransform == null) return;

        float dt = Time.deltaTime;
        float noise = Mathf.PerlinNoise(Time.time * 0.3f, 3f) - 0.5f;
        demoHeading += noise * 2f * demoTurnRate * dt;

        Vector3 center = operatorTransform != null ? operatorTransform.position : Vector3.zero;
        float radius = ((roomMaxX - roomMinX) / 2f) * demoRadiusFraction;

        Vector3 p = droneTransform.position;
        Vector3 toCenter = center - p;
        toCenter.y = 0f;
        if (toCenter.magnitude > radius)
        {
            float target = Mathf.Atan2(toCenter.x, toCenter.z) * Mathf.Rad2Deg;
            demoHeading = Mathf.MoveTowardsAngle(demoHeading, target, 180f * dt);
        }

        float r = demoHeading * Mathf.Deg2Rad;
        p += new Vector3(Mathf.Sin(r), 0f, Mathf.Cos(r)) * demoSpeed * dt;
        droneTransform.position = p;
    }

    Vector3 GetFacing()
    {
        float h;
        if (facingSource != null) h = facingSource.eulerAngles.y;
        else if (demoWander) h = demoHeading;
        else if (droneTransform != null) h = droneTransform.eulerAngles.y;
        else h = 0f;

        float r = h * Mathf.Deg2Rad;
        return new Vector3(Mathf.Sin(r), 0f, Mathf.Cos(r));
    }

    public void PlaceWall()
    {
        if (droneTransform == null || minimapTexture == null) return;

        Vector3 f = GetFacing();
        Vector3 right = new Vector3(f.z, 0f, -f.x);
        Vector3 c = droneTransform.position + f * wallDistance;
        Vector3 a = c - right * wallLength * 0.5f;
        Vector3 b = c + right * wallLength * 0.5f;

        Vector2 pa = WorldToPixelRelative(a.x, a.z);
        Vector2 pb = WorldToPixelRelative(b.x, b.z);

        DrawSolidLine(
            Mathf.RoundToInt(pa.x), Mathf.RoundToInt(pa.y),
            Mathf.RoundToInt(pb.x), Mathf.RoundToInt(pb.y),
            wallThickness, wallColor);
        minimapTexture.Apply();
    }

    public void PlaceMarker()
    {
        if (droneTransform == null) return;

        Vector3 w = droneTransform.position + GetFacing() * markerDistance;

        GameObject go = new GameObject("MapMarker", typeof(RectTransform), typeof(Image));
        RectTransform rt = go.GetComponent<RectTransform>();
        rt.SetParent(minimapBackground.rectTransform, false);
        rt.anchorMin = rt.anchorMax = new Vector2(0.5f, 0.5f);
        rt.pivot = new Vector2(0.5f, 0.5f);
        rt.sizeDelta = new Vector2(markerSize, markerSize);
        rt.anchoredPosition = WorldToMinimapRelative(w.x, w.z);

        Image img = go.GetComponent<Image>();
        img.sprite = markerSprite;
        img.raycastTarget = false;

        markers.Add(go);
    }

    public void ClearMarkers()
    {
        foreach (var m in markers)
            if (m != null) Destroy(m);
        markers.Clear();
    }

    void DrawSolidLine(int x0, int y0, int x1, int y1, int thickness, Color color)
    {
        int dx = Mathf.Abs(x1 - x0);
        int dy = Mathf.Abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;
        int half = thickness / 2;

        while (true)
        {
            for (int ox = -half; ox <= half; ox++)
                for (int oy = -half; oy <= half; oy++)
                {
                    int px = x0 + ox;
                    int py = y0 + oy;
                    if (px >= 0 && px < minimapSize && py >= 0 && py < minimapSize)
                        minimapTexture.SetPixel(px, py, color);
                }

            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x0 += sx; }
            if (e2 < dx) { err += dx; y0 += sy; }
        }
    }

    void UpdateFOVCone()
    {
        if (droneTransform == null) return;

        Vector3 currPos = droneTransform.position;
        float dx = currPos.x - lastDronePos.x;
        float dz = currPos.z - lastDronePos.z;

        if (Mathf.Sqrt(dx * dx + dz * dz) > 0.05f)
        {
            float heading = Mathf.Atan2(dx, dz) * Mathf.Rad2Deg;

            coneHistory.Add(new ConeEntry
            {
                worldX = currPos.x,
                worldZ = currPos.z,
                headingDeg = heading
            });

            if (coneHistory.Count > maxConeHistory)
                coneHistory.RemoveAt(0);

            PaintFOVCone(currPos.x, currPos.z, heading);
            lastDronePos = currPos;
        }
    }

    void PaintFOVCone(float worldX, float worldZ, float headingDeg)
    {
        float halfFOV = cameraFOV / 2f;
        int steps = 12;

        Vector2 apexPixel = WorldToPixelRelative(worldX, worldZ);

        for (int i = 0; i <= steps; i++)
        {
            float angle = headingDeg - halfFOV + (cameraFOV / steps) * i;
            float rad = angle * Mathf.Deg2Rad;

            float endX = worldX + cameraRange * Mathf.Sin(rad);
            float endZ = worldZ + cameraRange * Mathf.Cos(rad);

            Vector2 endPixel = WorldToPixelRelative(endX, endZ);

            DrawLine(
                Mathf.RoundToInt(apexPixel.x),
                Mathf.RoundToInt(apexPixel.y),
                Mathf.RoundToInt(endPixel.x),
                Mathf.RoundToInt(endPixel.y),
                new Color(0f, 1f, 0.4f, 0.08f));
        }
    }

    void DrawLine(int x0, int y0, int x1, int y1, Color color)
    {
        int dx = Mathf.Abs(x1 - x0);
        int dy = Mathf.Abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;

        while (true)
        {
            if (x0 >= 0 && x0 < minimapSize && y0 >= 0 && y0 < minimapSize)
            {
                Color existing = minimapTexture.GetPixel(x0, y0);
                minimapTexture.SetPixel(x0, y0,
                    new Color(
                        Mathf.Min(existing.r + color.r, 1f),
                        Mathf.Min(existing.g + color.g, 1f),
                        Mathf.Min(existing.b + color.b, 1f),
                        1f));
            }

            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x0 += sx; }
            if (e2 < dx) { err += dx; y0 += sy; }
        }
    }

    void UpdateDroneDot()
    {
        if (droneTransform == null || droneDot == null) return;

        RectTransform rt = minimapBackground.rectTransform;
        float halfW = rt.rect.width / 2f;
        float halfH = rt.rect.height / 2f;

        Vector2 pos = WorldToMinimapRelative(
            droneTransform.position.x,
            droneTransform.position.z);

        pos.x = Mathf.Clamp(pos.x, -halfW + 8f, halfW - 8f);
        pos.y = Mathf.Clamp(pos.y, -halfH + 8f, halfH - 8f);

        droneDot.anchoredPosition = pos;

        if (rotateDotToFacing)
        {
            Vector3 f = GetFacing();
            float h = Mathf.Atan2(f.x, f.z) * Mathf.Rad2Deg;
            droneDot.localRotation = Quaternion.Euler(0f, 0f, -h);
        }
    }

    void UpdateOperatorDot()
    {
        if (operatorDot == null) return;
        operatorDot.anchoredPosition = Vector2.zero;
    }

    void UpdateTrail()
    {
        if (droneTransform == null) return;

        Vector2 currentPixel = WorldToPixelRelative(
            droneTransform.position.x,
            droneTransform.position.z);

        int px = Mathf.RoundToInt(currentPixel.x);
        int py = Mathf.RoundToInt(currentPixel.y);
        PaintCircle(px, py, 2, new Color(0.2f, 0.6f, 1f, 0.5f));

        minimapTexture.Apply();
    }

    void HandleZoomAndPan()
    {
        float scroll = Input.GetAxis("Mouse ScrollWheel");
        if (scroll != 0f)
        {
            zoomLevel = Mathf.Clamp(zoomLevel + scroll * 2f, minZoom, maxZoom);
            ApplyZoom();
        }

        if (Input.GetMouseButtonDown(0))
        {
            RectTransform rt = minimapBackground.rectTransform;
            if (RectTransformUtility.RectangleContainsScreenPoint(rt, Input.mousePosition))
            {
                isDragging = true;
                lastDragPos = Input.mousePosition;
            }
        }

        if (isDragging && Input.GetMouseButton(0))
        {
            Vector2 delta = (Vector2)Input.mousePosition - lastDragPos;
            panOffset += delta * 0.5f;
            lastDragPos = Input.mousePosition;
            ApplyZoom();
        }

        if (Input.GetMouseButtonUp(0))
            isDragging = false;
    }

    void ApplyZoom()
    {
        RectTransform rt = minimapBackground.rectTransform;
        rt.localScale = new Vector3(zoomLevel, zoomLevel, 1f);
        rt.anchoredPosition = panOffset;
    }

    public void ZoomIn()
    {
        zoomLevel = Mathf.Min(zoomLevel + 0.25f, maxZoom);
        ApplyZoom();
    }

    public void ZoomOut()
    {
        zoomLevel = Mathf.Max(zoomLevel - 0.25f, minZoom);
        ApplyZoom();
    }

    public void ResetView()
    {
        zoomLevel = 1f;
        panOffset = Vector2.zero;
        ApplyZoom();
    }

    Vector2 WorldToPixelRelative(float worldX, float worldZ)
    {
        if (operatorTransform == null) return Vector2.zero;

        float relX = worldX - operatorTransform.position.x;
        float relZ = worldZ - operatorTransform.position.z;
        float range = (roomMaxX - roomMinX) / 2f;

        float normX = (relX / range) * 0.5f + 0.5f;
        float normZ = (relZ / range) * 0.5f + 0.5f;

        return new Vector2(
            Mathf.Clamp(normX * minimapSize, 0, minimapSize - 1),
            Mathf.Clamp(normZ * minimapSize, 0, minimapSize - 1));
    }

    Vector2 WorldToMinimapRelative(float worldX, float worldZ)
    {
        if (operatorTransform == null) return Vector2.zero;

        RectTransform rt = minimapBackground.rectTransform;
        float width = rt.rect.width;
        float height = rt.rect.height;

        float relX = worldX - operatorTransform.position.x;
        float relZ = worldZ - operatorTransform.position.z;
        float range = (roomMaxX - roomMinX) / 2f;

        float mapX = (relX / range) * (width / 2f);
        float mapY = (relZ / range) * (height / 2f);

        return new Vector2(mapX, mapY);
    }

    void PaintCircle(int cx, int cy, int radius, Color color)
    {
        for (int x = -radius; x <= radius; x++)
        {
            for (int y = -radius; y <= radius; y++)
            {
                if (x * x + y * y <= radius * radius)
                {
                    int px = cx + x;
                    int py = cy + y;
                    if (px >= 0 && px < minimapSize && py >= 0 && py < minimapSize)
                    {
                        Color existing = minimapTexture.GetPixel(px, py);
                        minimapTexture.SetPixel(px, py,
                            Color.Lerp(existing, color, 0.3f));
                    }
                }
            }
        }
    }

    void ClearTexture()
    {
        Color darkBg = new Color(0.05f, 0.08f, 0.12f, 0.9f);
        for (int x = 0; x < minimapSize; x++)
            for (int y = 0; y < minimapSize; y++)
                minimapTexture.SetPixel(x, y, darkBg);
        minimapTexture.Apply();
    }

    public void ClearTrail()
    {
        ClearTexture();
        coneHistory.Clear();
    }
}