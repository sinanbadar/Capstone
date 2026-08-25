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
    private List<Vector2> trailPoints = new List<Vector2>();
    private Texture2D minimapTexture;

    private int minimapSize = 256;

    void Start()
    {
        minimapTexture = new Texture2D(minimapSize, minimapSize);
        ClearTexture();
        minimapBackground.texture = minimapTexture;
    }

    void Update()
    {
        UpdateDroneDot();
        UpdateOperatorDot();
        UpdateTrail();
    }

    void UpdateDroneDot()
    {
        if (droneTransform == null || droneDot == null) return;
        Vector2 minimapPos = WorldToMinimapRelative(
            droneTransform.position.x,
            droneTransform.position.z);
        droneDot.anchoredPosition = minimapPos;
    }

    void UpdateOperatorDot()
    {
        if (operatorDot == null) return;
        operatorDot.anchoredPosition = Vector2.zero;
    }

    void UpdateTrail()
    {
        if (droneTransform == null || operatorTransform == null) return;

        Vector2 currentPixel = WorldToPixelRelative(
            droneTransform.position.x,
            droneTransform.position.z);

        int px = Mathf.RoundToInt(currentPixel.x);
        int py = Mathf.RoundToInt(currentPixel.y);
        PaintCircle(px, py, 4, new Color(0.2f, 0.6f, 1f, 0.5f));

        minimapTexture.Apply();
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

    Vector2 WorldToPixel(float worldX, float worldZ)
    {
        float normX = Mathf.InverseLerp(roomMinX, roomMaxX, worldX);
        float normZ = Mathf.InverseLerp(roomMinZ, roomMaxZ, worldZ);
        return new Vector2(normX * minimapSize, normZ * minimapSize);
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
    }
}