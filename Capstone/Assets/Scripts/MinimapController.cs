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

    private List<Vector2> trailPoints = new List<Vector2>();
    private Texture2D minimapTexture;
    private int minimapSize = 256;

    // Store previous drone heading positions for cone drawing
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
        UpdateFOVCone();
        UpdateDroneDot();
        UpdateOperatorDot();
        UpdateTrail();
        HandleZoomAndPan();
    }

    // FOV CONE
    void UpdateFOVCone()
    {
        if (droneTransform == null) return;

        Vector3 currPos = droneTransform.position;
        float dx = currPos.x - lastDronePos.x;
        float dz = currPos.z - lastDronePos.z;

        // Only record cone when drone is moving
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

            // Paint cone onto texture
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

            // Draw triangle slice from apex to edge
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

    // DRONE DOT
    void UpdateDroneDot()
    {
        if (droneTransform == null || droneDot == null) return;

        RectTransform rt = minimapBackground.rectTransform;
        float halfW = rt.rect.width / 2f;
        float halfH = rt.rect.height / 2f;

        Vector2 pos = WorldToMinimapRelative(
            droneTransform.position.x,
            droneTransform.position.z);

        // Clamp dot inside minimap bounds
        pos.x = Mathf.Clamp(pos.x, -halfW + 8f, halfW - 8f);
        pos.y = Mathf.Clamp(pos.y, -halfH + 8f, halfH - 8f);

        droneDot.anchoredPosition = pos;
    }

    // OPERATOR DOT
    void UpdateOperatorDot()
    {
        if (operatorDot == null) return;
        operatorDot.anchoredPosition = Vector2.zero;
    }

    // TRAIL
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

    // ZOOM AND PAN
    void HandleZoomAndPan()
    {
        // Scroll wheel zoom
        float scroll = Input.GetAxis("Mouse ScrollWheel");
        if (scroll != 0f)
        {
            zoomLevel = Mathf.Clamp(zoomLevel + scroll * 2f, minZoom, maxZoom);
            ApplyZoom();
        }

        // Click and drag pan
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

    // HELPERS
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