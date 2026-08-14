using UnityEngine;
using System.Collections.Generic;
using TMPro;

public class MarkerSpawner : MonoBehaviour
{
    public DetectionReceiver detectionReceiver;
    public float markerHeight = 1.5f;
    public float confidenceThreshold = 0.5f;

    private Dictionary<string, GameObject> activeMarkers = new Dictionary<string, GameObject>();

    void Update()
    {
        if (detectionReceiver.newDetectionsAvailable)
        {
            detectionReceiver.newDetectionsAvailable = false;
            ProcessDetections(detectionReceiver.latestDetections);
        }
    }

    void ProcessDetections(List<DetectionReceiver.Detection> detections)
    {
        foreach (var detection in detections)
        {
            if (detection.confidence < confidenceThreshold) continue;

            string key = detection.label;

            if (!activeMarkers.ContainsKey(key))
            {
                SpawnMarker(detection);
            }
            else
            {
                UpdateMarkerLabel(activeMarkers[key], detection);
            }
        }
    }

    void SpawnMarker(DetectionReceiver.Detection detection)
    {
        // Create marker GameObject
        GameObject marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        marker.name = "Marker_" + detection.label;
        marker.transform.localScale = Vector3.one * 0.3f;

        // Position at drone position plus height offset
        Vector3 dronePos = GetDronePosition();
        marker.transform.position = new Vector3(
            dronePos.x,
            markerHeight,
            dronePos.z);

        // Set colour based on confidence
        Renderer rend = marker.GetComponent<Renderer>();
        rend.material.color = GetConfidenceColour(detection.confidence);

        // Add label above marker
        GameObject labelObj = new GameObject("Label");
        labelObj.transform.SetParent(marker.transform);
        labelObj.transform.localPosition = Vector3.up * 0.8f;

        TextMeshPro tmp = labelObj.AddComponent<TextMeshPro>();
        tmp.text = detection.label + "\n" +
                   (detection.confidence * 100f).ToString("F0") + "%";
        tmp.fontSize = 3f;
        tmp.alignment = TextAlignmentOptions.Center;
        tmp.color = Color.white;

        activeMarkers[detection.label] = marker;
        Debug.Log("Spawned marker for: " + detection.label +
                  " confidence: " + detection.confidence);
    }

    void UpdateMarkerLabel(GameObject marker, DetectionReceiver.Detection detection)
    {
        TextMeshPro tmp = marker.GetComponentInChildren<TextMeshPro>();
        if (tmp != null)
        {
            tmp.text = detection.label + "\n" +
                       (detection.confidence * 100f).ToString("F0") + "%";
        }
        Renderer rend = marker.GetComponent<Renderer>();
        rend.material.color = GetConfidenceColour(detection.confidence);
    }

    Vector3 GetDronePosition()
    {
        GameObject drone = GameObject.Find("Drone");
        if (drone != null)
            return drone.transform.position;
        return Vector3.zero;
    }

    Color GetConfidenceColour(float confidence)
    {
        if (confidence >= 0.8f) return Color.green;
        if (confidence >= 0.6f) return Color.yellow;
        return Color.red;
    }

    public void ClearAllMarkers()
    {
        foreach (var marker in activeMarkers.Values)
            Destroy(marker);
        activeMarkers.Clear();
    }
}