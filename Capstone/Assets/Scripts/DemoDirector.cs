using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DemoDirector : MonoBehaviour
{
    [Header("Scene references")]
    public Transform drone;
    public DroneStatusUI hud;
    public MinimapController minimap;

    [Header("Person of interest")]
    public Transform personOfInterest;
    public GameObject worldMarkerPrefab;
    public float markerHeight = 0.4f;          // above the person's position
    public string detectionLabel = "person 94%";

    [Header("Playback")]
    public bool autoStart = true;
    public float startDelay = 3f;
    public bool loop = false;
    public float turnSpeed = 4f;
    public float hoverBob = 0.02f;
    public float floorY = 0f;                  // world Y of the floor (0 with Floor tracking)
    public Vector3 markerOffset = new Vector3(0f, 0.4f, 0f);

    [System.Serializable]
    public class Step
    {
        public string label;
        public Transform target;               // waypoint to fly to (optional)
        public float speed = 0.6f;
        public Transform faceTarget;           // turn to face this before pausing (optional)
        public bool placeMarker;               // tag the person on this step
        public float pauseAfter = 1f;
        public DroneStatusUI.DroneStatus status = DroneStatusUI.DroneStatus.Searching;
        
    }

    public List<Step> steps = new List<Step>();

    void Start()
    {
        if (autoStart) StartCoroutine(Run());
    }

    [ContextMenu("Play Demo")]
    public void Play() { StopAllCoroutines(); StartCoroutine(Run()); }

    IEnumerator Run()
    {
        if (hud != null)
        {
            hud.demoMode = true;
            hud.autoSpeed = false;
            hud.autoHeight = false;
            hud.status = DroneStatusUI.DroneStatus.Idle;
        }
        if (minimap != null)
        {
            minimap.demoWander = false;
            minimap.droneTransform = drone;
            minimap.facingSource = drone;
        }

        yield return new WaitForSeconds(startDelay);

        do
        {
            foreach (var s in steps)
            {
                if (hud != null) hud.status = s.status;

                if (s.target != null)
                    yield return FlyTo(s.target.position, s.speed);

                if (s.faceTarget != null)
                    yield return FaceTo(s.faceTarget.position, 1f);

                if (s.placeMarker)
                    TagPerson();

                yield return Hover(s.pauseAfter);
            }
        } while (loop);
    }

    IEnumerator FlyTo(Vector3 dest, float speed)
    {
        while ((drone.position - dest).sqrMagnitude > 0.0004f)
        {
            Vector3 dir = dest - drone.position;
            dir.y = 0f;
            if (dir.sqrMagnitude > 0.001f)
                drone.rotation = Quaternion.Slerp(drone.rotation,
                    Quaternion.LookRotation(dir), turnSpeed * Time.deltaTime);

            drone.position = Vector3.MoveTowards(drone.position, dest, speed * Time.deltaTime);
            UpdateHud(speed);
            yield return null;
        }
    }

    IEnumerator FaceTo(Vector3 point, float duration)
    {
        Vector3 dir = point - drone.position;
        dir.y = 0f;
        Quaternion goal = Quaternion.LookRotation(dir);
        Quaternion start = drone.rotation;
        float t = 0f;
        while (t < duration)
        {
            t += Time.deltaTime;
            drone.rotation = Quaternion.Slerp(start, goal, t / duration);
            UpdateHud(0.03f);
            yield return null;
        }
    }

    IEnumerator Hover(float seconds)
    {
        Vector3 basePos = drone.position;
        float e = 0f;
        while (e < seconds)
        {
            e += Time.deltaTime;
            drone.position = basePos + Vector3.up * Mathf.Sin(Time.time * 2f) * hoverBob;
            UpdateHud(0.03f);
            yield return null;
        }
        drone.position = basePos;
    }
    public Vector3 markerRotation = new Vector3(-90f, 0f, 0f);
    void TagPerson()
    {
        if (personOfInterest == null) return;

        if (worldMarkerPrefab != null)
        {
            var m = Instantiate(worldMarkerPrefab,
                personOfInterest.position + markerOffset,
                Quaternion.Euler(markerRotation));
            m.SetActive(true);
        }

        if (minimap != null)
        {
            Vector3 d = personOfInterest.position - drone.position;
            d.y = 0f;
            minimap.markerDistance = d.magnitude;
            minimap.PlaceMarker();
        }

        if (hud != null && hud.detectionsText != null)
            hud.detectionsText.text = detectionLabel;
    }

    void UpdateHud(float speed)
    {
        if (hud == null) return;
        hud.speed = speed;
        hud.height = Mathf.Clamp(drone.position.y - floorY, 0f, hud.maxHeight);
    }
}