import time
import math
import threading

# ── SEARCH PARAMETERS ────────────────────────────────────
FORWARD_SPEED = 25
YAW_SPEED = 30
LOOK_DEGREES = 45
LOOK_INTERVAL = 4.0
STEP_SIZE_M = 1.0
OBSTACLE_THRESHOLD_M = 1.0
OBSTACLE_MIN_POINTS = 15
# ─────────────────────────────────────────────────────────

class AutonomousSearch:
    def __init__(self, send_command_func, get_detections_func,
                 get_position_func, get_point_cloud_func=None,
                 get_optical_flow_func=None):
        self.send_command = send_command_func
        self.get_detections = get_detections_func
        self.get_position = get_position_func
        self.get_point_cloud = get_point_cloud_func
        self.get_optical_flow = get_optical_flow_func
        self.running = False
        self.thread = None
        self.pass_count = 0
        self.step_direction = 1
        self.is_yawing = False

    def start(self):
        if self.running:
            print("Already running")
            return
        self.running = True
        self.pass_count = 0
        self.is_yawing = False
        self.thread = threading.Thread(target=self._execute, daemon=True)
        self.thread.start()
        print("AUTO: search started")

    def stop(self):
        self.running = False
        self.is_yawing = False
        self.send_command("rc 0 0 0 0")
        print("AUTO: search stopped")

    # ── OBSTACLE DETECTION ────────────────────────────────
    def _is_forward_point(self, point, pos):
        px = point["x"] - pos.get("x", 0)
        py = point.get("y", 0)
        pz = point["z"] - pos.get("z", 0)
        dist = math.sqrt(px**2 + pz**2)

        if dist > OBSTACLE_THRESHOLD_M:
            return False
        if py < 0.2:
            return False
        dot = pz / dist if dist > 0 else 0
        return dot > 0.85

    def obstacle_ahead(self):
        if self.is_yawing:
            return False

        flow_triggered = False
        slam_triggered = False

        if self.get_optical_flow and self.get_optical_flow():
            flow_triggered = True

        if self.get_point_cloud:
            pos = self.get_position()
            if pos.get("tracking"):
                cloud = self.get_point_cloud()
                if cloud:
                    forward_points = [
                        p for p in cloud
                        if self._is_forward_point(p, pos)
                    ]
                    if len(forward_points) >= OBSTACLE_MIN_POINTS:
                        slam_triggered = True

        if flow_triggered and slam_triggered:
            print("AUTO: obstacle confirmed by both sensors")
            return True

        if flow_triggered:
            print("AUTO: optical flow obstacle")
            return True

        if slam_triggered:
            print("AUTO: SLAM obstacle")
            return True

        detections = self.get_detections()
        for detection in detections:
            bbox = detection["bbox"]
            w = bbox["x2"] - bbox["x1"]
            h = bbox["y2"] - bbox["y1"]
            fill = (w * h) / (640 * 480)
            cx = (bbox["x1"] + bbox["x2"]) / 2
            if fill > 0.35 and 160 < cx < 480:
                print(f"AUTO: YOLO obstacle {detection['label']}")
                return True

        return False

    # ── BRIEF LOOK AROUND ─────────────────────────────────
    def look_around(self):
        self.is_yawing = True

        # Look left
        self.send_command(f"rc 0 0 0 -{YAW_SPEED}")
        time.sleep(LOOK_DEGREES / YAW_SPEED)
        self.send_command("rc 0 0 0 0")
        time.sleep(0.5)

        # Look right past centre
        self.send_command(f"rc 0 0 0 {YAW_SPEED}")
        time.sleep((LOOK_DEGREES * 2) / YAW_SPEED)
        self.send_command("rc 0 0 0 0")
        time.sleep(0.5)

        # Return to centre
        self.send_command(f"rc 0 0 0 -{YAW_SPEED}")
        time.sleep(LOOK_DEGREES / YAW_SPEED)
        self.send_command("rc 0 0 0 0")
        time.sleep(0.5)

        self.is_yawing = False

    # ── STEP SIDEWAYS ─────────────────────────────────────
    def step_sideways(self):
        lr = FORWARD_SPEED * self.step_direction
        step_duration = STEP_SIZE_M / (FORWARD_SPEED / 100)
        self.send_command(f"rc {lr} 0 0 0")
        time.sleep(step_duration)
        self.send_command("rc 0 0 0 0")
        time.sleep(0.5)
        print(f"AUTO: stepped sideways, pass {self.pass_count + 1} next")

    # ── FORWARD PASS ──────────────────────────────────────
    def fly_pass(self, forward_speed):
        self.send_command(f"rc 0 {forward_speed} 0 0")
        start_time = time.time()
        last_look_time = time.time()

        while self.running:
            now = time.time()

            # Skip obstacle check for first 2 seconds
            if now - start_time < 2.0:
                time.sleep(0.05)
                continue

            # Check obstacle
            if self.obstacle_ahead():
                self.send_command("rc 0 0 0 0")
                time.sleep(0.3)
                return

            # Periodic look around
            if now - last_look_time > LOOK_INTERVAL:
                self.send_command("rc 0 0 0 0")
                time.sleep(0.3)
                self.look_around()
                self.send_command(f"rc 0 {forward_speed} 0 0")
                last_look_time = time.time()

            time.sleep(0.05)

    # ── MAIN EXECUTE ──────────────────────────────────────
    def _execute(self):
        print("AUTO: flying forward")
        forward = True

        while self.running:
            speed = FORWARD_SPEED if forward else -FORWARD_SPEED
            self.fly_pass(speed)

            if not self.running:
                break

            self.pass_count += 1
            print(f"AUTO: pass {self.pass_count} done, stepping sideways")

            self.step_sideways()
            forward = not forward

        if self.running:
            self.send_command("rc 0 0 0 0")
            print("AUTO: hovering")

        self.running = False

    def get_status(self):
        return {
            "pass_count": self.pass_count,
            "running": self.running
        }