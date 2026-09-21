import time
import math
import threading

# ── ROOM PARAMETERS ──────────────────────────────────────
ROOM_WIDTH_CM = 150
ROOM_DEPTH_CM = 150
STEP_SIZE_CM = 50
CRUISE_ALTITUDE_CM = 50
DETECTION_CONFIDENCE_THRESHOLD = 0.65
FLIGHT_SPEED = 70  # RC speed value 0-100
# ─────────────────────────────────────────────────────────

class AutonomousSearch:
    def __init__(self, send_command_func, get_detections_func, get_position_func):
        self.send_command = send_command_func
        self.get_detections = get_detections_func
        self.get_position = get_position_func
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

        # Rise to cruise altitude
        self._rc_move(0, 0, 50, 0, 1.5)

        rows = math.ceil(ROOM_DEPTH_CM / STEP_SIZE_CM)
        print(f"Search pattern: {rows} rows")

        # Time to fly room width at given speed
        # speed 40 roughly = 40cm/s in simulation
        fly_duration = ROOM_WIDTH_CM / FLIGHT_SPEED
        step_duration = STEP_SIZE_CM / FLIGHT_SPEED

        for row in range(rows):
            if not self.running:
                break

            print(f"Row {row + 1}/{rows}")

            # Fly across room width
            if row % 2 == 0:
                self._rc_move(0, FLIGHT_SPEED, 0, 0, fly_duration)
            else:
                self._rc_move(0, -FLIGHT_SPEED, 0, 0, fly_duration)

            if not self.running:
                break

            # Step to next row
            if row < rows - 1:
                self._rc_move(FLIGHT_SPEED, 0, 0, 0, step_duration)

        if self.running:
            print("Coverage complete, landing")
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
            "pos_x": pos.get("pos_x", 0),
            "pos_z": pos.get("pos_z", 0)
        })

        # Pause movement
        self.send_command("rc 0 0 0 0")
        time.sleep(0.5)

        # Lateral inspection
        self._rc_move(FLIGHT_SPEED, 0, 0, 0, 0.8)
        self._rc_move(-FLIGHT_SPEED, 0, 0, 0, 1.6)
        self._rc_move(FLIGHT_SPEED, 0, 0, 0, 0.8)

        print("Inspection complete, resuming")

    def _already_visited(self, detection):
        pos = self.get_position()
        for v in self.visited:
            if v["label"] == detection["label"]:
                dist = math.sqrt(
                    (pos.get("pos_x", 0) - v["pos_x"])**2 +
                    (pos.get("pos_z", 0) - v["pos_z"])**2
                )
                if dist < 80:
                    return True
        return False