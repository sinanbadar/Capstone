import time
import math
import threading

# ── SEARCH PARAMETERS ────────────────────────────────────
FORWARD_SPEED = 25
YAW_SPEED = 30
STEP_SIZE_M = 1.0
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

    def start(self):
        if self.running:
            print("Already running")
            return
        self.running = True
        self.pass_count = 0
        self.thread = threading.Thread(target=self._execute, daemon=True)
        self.thread.start()
        print("Autonomous search started")

    def stop(self):
        self.running = False
        self.send_command("rc 0 0 0 0")
        print("Autonomous search stopped")

    # ── OBSTACLE DETECTION ────────────────────────────────
    def obstacle_ahead(self):
        # Primary: optical flow
        if self.get_optical_flow and self.get_optical_flow():
            return True

        # Fallback: YOLO large object in centre
        detections = self.get_detections()
        for detection in detections:
            bbox = detection["bbox"]
            w = bbox["x2"] - bbox["x1"]
            h = bbox["y2"] - bbox["y1"]
            fill = (w * h) / (640 * 480)
            cx = (bbox["x1"] + bbox["x2"]) / 2
            # Only trigger if object is in centre of frame
            if fill > 0.35 and 160 < cx < 480:
                print(f"YOLO obstacle: {detection['label']} {fill:.0%}")
                return True

        return False

    # ── 360 SCAN ─────────────────────────────────────────
    def scan_360(self):
        print("360 scan starting")
        self.send_command(f"rc 0 0 0 {YAW_SPEED}")
        time.sleep(360 / YAW_SPEED)
        self.send_command("rc 0 0 0 0")
        time.sleep(1.0)
        print("360 scan complete")

    # ── STEP SIDEWAYS ─────────────────────────────────────
    def step_sideways(self):
        print("Stepping sideways")
        lr = FORWARD_SPEED * self.step_direction
        step_duration = STEP_SIZE_M / (FORWARD_SPEED / 100)
        self.send_command(f"rc {lr} 0 0 0")
        time.sleep(step_duration)
        self.send_command("rc 0 0 0 0")
        time.sleep(0.5)

    # ── FORWARD PASS ──────────────────────────────────────
    def fly_pass(self, forward_speed):
        print(f"Pass {self.pass_count + 1} starting")
        self.send_command(f"rc 0 {forward_speed} 0 0")
        start_time = time.time()

        while self.running:
            now = time.time()

            # Wait 2 seconds before checking
            if now - start_time < 2.0:
                time.sleep(0.1)
                continue

            if self.obstacle_ahead():
                self.send_command("rc 0 0 0 0")
                time.sleep(0.3)
                print("Obstacle detected, ending pass")
                return

            time.sleep(0.1)

    # ── MAIN EXECUTE ──────────────────────────────────────
    def _execute(self):
        print("Autonomous search starting")

        print("Waiting for SLAM tracking...")
        for _ in range(20):
            if self.get_position().get("tracking"):
                print("SLAM tracking confirmed")
                break
            time.sleep(0.5)

        if not self.get_position().get("tracking"):
            print("SLAM not tracking, using optical flow and YOLO only")

        forward = True

        while self.running:
            # Scan 360 before each pass
            self.scan_360()

            if not self.running:
                break

            # Fly pass
            speed = FORWARD_SPEED if forward else -FORWARD_SPEED
            self.fly_pass(speed)

            if not self.running:
                break

            self.pass_count += 1
            print(f"Pass {self.pass_count} complete")

            # Step sideways
            self.step_sideways()

            # Reverse
            forward = not forward

        if self.running:
            self.send_command("rc 0 0 0 0")
            print("Search complete, hovering")

        self.running = False

    def get_status(self):
        return {
            "pass_count": self.pass_count,
            "running": self.running
        }