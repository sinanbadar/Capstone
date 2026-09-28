import time
import math
import threading

# ── SEARCH PARAMETERS ────────────────────────────────────
STEP_SIZE_CM = 50
CRUISE_ALTITUDE_CM = 50
DETECTION_CONFIDENCE_THRESHOLD = 0.65
FLIGHT_SPEED = 40
MAX_ROWS = 20
POINT_CLOUD_THRESHOLD_DISTANCE = 0.8  # metres
POINT_CLOUD_MIN_POINTS = 5
# ─────────────────────────────────────────────────────────

class AutonomousSearch:
    def __init__(self, send_command_func, get_detections_func, get_position_func, get_point_cloud_func=None):
        self.send_command = send_command_func
        self.get_detections = get_detections_func
        self.get_position = get_position_func
        self.get_point_cloud = get_point_cloud_func
        self.running = False
        self.thread = None
        self.visited = []

    def start(self):
        if self.running:
            print("Already running")
            return
        self.running = True
        self.visited = []
        self.thread = threading.Thread(target=self._execute, daemon=True)
        self.thread.start()
        print("Autonomous search started")

    def stop(self):
        self.running = False
        self.send_command("rc 0 0 0 0")
        print("Autonomous search stopped")

    def obstacle_ahead(self):
        # Primary: SLAM point cloud
        if self.get_point_cloud:
            point_cloud = self.get_point_cloud()
            drone_pos = self.get_position()
            drone_x = drone_pos.get("x", drone_pos.get("pos_x", 0))
            drone_z = drone_pos.get("z", drone_pos.get("pos_z", 0))

            forward_points = []
            for point in point_cloud:
                dx = point["x"] - drone_x
                dz = point["z"] - drone_z
                distance = math.sqrt(dx**2 + dz**2)
                if distance < POINT_CLOUD_THRESHOLD_DISTANCE:
                    forward_points.append(point)

            if len(forward_points) >= POINT_CLOUD_MIN_POINTS:
                print(f"Point cloud: {len(forward_points)} points within {POINT_CLOUD_THRESHOLD_DISTANCE}m")
                return True

        # Fallback: YOLO proximity
        detections = self.get_detections()
        for detection in detections:
            bbox = detection["bbox"]
            box_width = bbox["x2"] - bbox["x1"]
            box_height = bbox["y2"] - bbox["y1"]
            fill_ratio = (box_width * box_height) / (640 * 480)
            if fill_ratio > 0.4:
                print(f"YOLO proximity: {detection['label']} at {fill_ratio:.0%}")
                return True

        return False

    def fly_until_blocked(self, lr, fb):
        self.send_command(f"rc {lr} {fb} 0 0")
        time.sleep(1.0)

        while self.running:
            time.sleep(0.2)
            if self.obstacle_ahead():
                self.send_command("rc 0 0 0 0")
                time.sleep(0.3)
                print("Obstacle detected, turning")
                return
            self._check_and_handle_detections()

    def _rc_move(self, lr, fb, ud, yaw, duration):
        self.send_command(f"rc {lr} {fb} {ud} {yaw}")
        end_time = time.time() + duration
        while time.time() < end_time and self.running:
            self._check_and_handle_detections()
            time.sleep(0.1)
        self.send_command("rc 0 0 0 0")
        time.sleep(0.3)

    def _execute(self):
        print("Taking off")
        self.send_command("takeoff")
        time.sleep(3)

        self._rc_move(0, 0, 50, 0, 1.5)

        row = 0
        print("Starting SLAM point cloud guided lawnmower")

        while self.running and row < MAX_ROWS:
            print(f"Row {row + 1}")

            if row % 2 == 0:
                self.fly_until_blocked(0, FLIGHT_SPEED)
            else:
                self.fly_until_blocked(0, -FLIGHT_SPEED)

            if not self.running:
                break

            self._rc_move(FLIGHT_SPEED, 0, 0, 0, STEP_SIZE_CM / FLIGHT_SPEED)
            row += 1

        if self.running:
            print("Search complete, landing")
            self.send_command("land")

        self.running = False

    def _check_and_handle_detections(self):
        detections = self.get_detections()
        if not detections:
            return

        for detection in detections:
            if detection["confidence"] < DETECTION_CONFIDENCE_THRESHOLD:
                continue
            if self._already_visited(detection):
                continue

            print(f"Detection: {detection['label']} {detection['confidence']:.2f}")
            self._inspect(detection)

    def _inspect(self, detection):
        print(f"Inspecting {detection['label']}")
        pos = self.get_position()
        self.visited.append({
            "label": detection["label"],
            "confidence": detection["confidence"],
            "pos_x": pos.get("x", pos.get("pos_x", 0)),
            "pos_z": pos.get("z", pos.get("pos_z", 0))
        })

        self.send_command("rc 0 0 0 0")
        time.sleep(0.5)

        self._rc_move(FLIGHT_SPEED, 0, 0, 0, 0.8)
        self._rc_move(-FLIGHT_SPEED, 0, 0, 0, 1.6)
        self._rc_move(FLIGHT_SPEED, 0, 0, 0, 0.8)

        print("Inspection complete, resuming")

    def _already_visited(self, detection):
        pos = self.get_position()
        for v in self.visited:
            if v["label"] == detection["label"]:
                dist = math.sqrt(
                    (pos.get("x", pos.get("pos_x", 0)) - v["pos_x"])**2 +
                    (pos.get("z", pos.get("pos_z", 0)) - v["pos_z"])**2
                )
                if dist < 80:
                    return True
        return False