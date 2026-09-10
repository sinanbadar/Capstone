using UnityEngine;
using TMPro;
using UnityEngine.UI;

public class ControlModeUI : MonoBehaviour
{
    public TextMeshProUGUI modeLabel;
    public TextMeshProUGUI modeSubLabel;
    public Image panelBackground;
    public DroneStatusUI droneStatusUI;

    public enum ControlMode { Manual, Autonomous, Gaze }
    public ControlMode currentMode = ControlMode.Autonomous;

    private Color manualColour = new Color(0.5f, 0.1f, 0.05f, 0.85f);
    private Color autonomousColour = new Color(0.05f, 0.1f, 0.4f, 0.85f);
    private Color gazeColour = new Color(0.15f, 0.05f, 0.35f, 0.85f);

    void Start()
    {
        SetMode(currentMode);
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Alpha1)) SetMode(ControlMode.Manual);
        if (Input.GetKeyDown(KeyCode.Alpha2)) SetMode(ControlMode.Autonomous);
        if (Input.GetKeyDown(KeyCode.Alpha3)) SetMode(ControlMode.Gaze);
    }

    public void SetMode(ControlMode mode)
    {
        currentMode = mode;

        switch (mode)
        {
            case ControlMode.Manual:
                if (modeLabel) modeLabel.text = "MANUAL";
                if (modeSubLabel) modeSubLabel.text = "Direct control active";
                if (panelBackground) panelBackground.color = manualColour;
                if (droneStatusUI) droneStatusUI.SetStatus(DroneStatusUI.DroneStatus.Flying);
                break;

            case ControlMode.Autonomous:
                if (modeLabel) modeLabel.text = "AUTONOMOUS";
                if (modeSubLabel) modeSubLabel.text = "Searching room";
                if (panelBackground) panelBackground.color = autonomousColour;
                if (droneStatusUI) droneStatusUI.SetStatus(DroneStatusUI.DroneStatus.Searching);
                break;

            case ControlMode.Gaze:
                if (modeLabel) modeLabel.text = "GAZE DIRECTED";
                if (modeSubLabel) modeSubLabel.text = "Look to direct drone";
                if (panelBackground) panelBackground.color = gazeColour;
                if (droneStatusUI) droneStatusUI.SetStatus(DroneStatusUI.DroneStatus.Flying);
                break;
        }
    }

    public void OnManualButton() => SetMode(ControlMode.Manual);
    public void OnAutonomousButton() => SetMode(ControlMode.Autonomous);
    public void OnGazeButton() => SetMode(ControlMode.Gaze);
}