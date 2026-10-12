import time
import math
import threading

# ── SEARCH PARAMETERS ────────────────────────────────────
FORWARD_SPEED = 5
YAW_SPEED = 5
STEP_SIZE_M = 1.0
OBSTACLE_THRESHOLD_M = 0.5
OBSTACLE_MIN_POINTS = 25
KEEPALIVE_INTERVAL = 5.0
FLAT_WALL_Z_STD = 0.15
FLAT_WALL_MIN_POINTS = 20 
FLAT_WALL_RADIUS = 2.0
POST_TURN_SLAM_DELAY = 4.0
# ─────────────────────────────────────────────────────────

class AutonomousSearch:
    def __init__(self, send_command_func, get_detections_func,
                 get_position_func, get_point_cloud_func=None,
                 get_optical_flow_func=None, pause_flow_func=None,
                 get_depth_func=None):
        self.send_command = send_command_func
        self.get_detections = get_detections_func
        self.get_position = get_position_func
        self.get_point_cloud = get_point_cloud_func
        self.get_optical_flow = get_optical_flow_func
        self.pause_flow = pause_flow_func
        self.get_depth = get_depth_func
        self.running = False
        self.thread = None
        self.pass_count = 0
        self.step_direction = 1
        self.is_yawing = False
        self.last_turn_time = 0

    def start(self):
        if self.running:
            print("Already running")
            return
        self.running = True
        self.pass_count = 0
        self.last_turn_time = 0
        self.thread = threading.Thread(target=self._execute, daemon=True)
        self.thread.start()
        print("AUTO: started")

    def stop(self):
        self.running = False
        if self.pause_flow:
            self.pause_flow(False)
        self.send_command("rc 0 0 0 0")
        print("AUTO: stopped")

    # ── FORWARD CONE CHECK ────────────────────────────────
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
        return dot > 0.93

    # ── FLAT WALL DETECTION ───────────────────────────────
    def flat_wall_ahead(self, cloud, pos):
        if not cloud:
            return False

        drone_x = pos.get("x", 0)
        drone_z = pos.get("z", 0)

        nearby = [
            p for p in cloud
            if math.sqrt(
                (p["x"] - drone_x)**2 +
                (p["z"] - drone_z)**2
            ) < FLAT_WALL_RADIUS
            and p.get("y", 0) > 0.2
        ]

        if len(nearby) < FLAT_WALL_MIN_POINTS:
            return False

        zs = [p["z"] for p in nearby]
        z_range = max(zs) - min(zs)

        if z_range < FLAT_WALL_Z_STD:
            print(f"AUTO: flat wall {len(nearby)} coplanar points")
            return True

        return False

    # ── OBSTACLE DETECTION ────────────────────────────────
    def obstacle_ahead(self):
        just_turned = time.time() - self.last_turn_time < POST_TURN_SLAM_DELAY

        flow_triggered = False
        slam_triggered = False
        flat_wall = False

        # Depth check first, primary sensor
        if self.get_depth and self.get_depth():
            print("AUTO: depth obstacle")
            return True

        if self.get_optical_flow and self.get_optical_flow():
            flow_triggered = True

        if not just_turned and self.get_point_cloud:
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

                    if not slam_triggered:
                        flat_wall = self.flat_wall_ahead(cloud, pos)

        if flow_triggered and slam_triggered:
            print("AUTO: obstacle confirmed by both sensors")
            return True
        if flow_triggered:
            print("AUTO: optical flow obstacle")
            return True
        if slam_triggered:
            print("AUTO: SLAM obstacle")
            return True
        if flat_wall:
            print("AUTO: flat wall detected")
            return True

        if not just_turned:
            detections = self.get_detections()
            for detection in detections:
                bbox = detection["bbox"]
                w = bbox["x2"] - bbox["x1"]
                h = bbox["y2"] - bbox["y1"]
                fill = (w * h) / (640 * 480)
                cx = (bbox["x1"] + bbox["x2"]) / 2
                if fill > 0.5 and 200 < cx < 440:
                    print(f"AUTO: YOLO obstacle {detection['label']}")
                    return True

        return False

    # ── TURN 180 ──────────────────────────────────────────
    def turn_180(self):
        print("AUTO: turning 180")
        if self.pause_flow:
            self.pause_flow(True)
        self.is_yawing = True
        self.send_command(f"rc 0 0 0 {YAW_SPEED}")
        time.sleep(180 / YAW_SPEED)
        self.send_command("rc 0 0 0 0")
        time.sleep(0.3)
        self.send_command("rc 0 0 0 0")
        time.sleep(0.3)
        self.send_command("rc 0 0 0 0")
        time.sleep(0.3)
        self.is_yawing = False
        self.last_turn_time = time.time()
        if self.pause_flow:
            self.pause_flow(False)
        print("AUTO: turn complete, SLAM paused 4s")

    # ── STEP SIDEWAYS ─────────────────────────────────────
    def step_sideways(self):
        if self.pause_flow:
            self.pause_flow(True)

        turn_yaw = YAW_SPEED if self.step_direction > 0 else -YAW_SPEED
        self.send_command(f"rc 0 0 0 {turn_yaw}")
        time.sleep(90 / YAW_SPEED)
        self.send_command("rc 0 0 0 0")
        time.sleep(0.5)

        if self.pause_flow:
            self.pause_flow(False)

        self.last_turn_time = time.time()
        time.sleep(0.5)

        if self.obstacle_ahead():
            print("AUTO: sideways blocked, reversing step direction")
            self.step_direction *= -1
            turn_yaw = YAW_SPEED if self.step_direction > 0 else -YAW_SPEED
            if self.pause_flow:
                self.pause_flow(True)
            self.send_command(f"rc 0 0 0 {turn_yaw}")
            time.sleep(180 / YAW_SPEED)
            self.send_command("rc 0 0 0 0")
            time.sleep(0.5)
            if self.pause_flow:
                self.pause_flow(False)
            self.last_turn_time = time.time()
            time.sleep(0.5)

        step_duration = STEP_SIZE_M / (FORWARD_SPEED / 100)
        self.send_command(f"rc 0 {FORWARD_SPEED} 0 0")
        elapsed = 0
        while elapsed < step_duration and self.running:
            time.sleep(1.0)
            elapsed += 1.0
            if elapsed < step_duration:
                self.send_command(f"rc 0 {FORWARD_SPEED} 0 0")
        self.send_command("rc 0 0 0 0")
        time.sleep(0.3)

        if self.pause_flow:
            self.pause_flow(True)
        turn_back = -turn_yaw
        self.send_command(f"rc 0 0 0 {turn_back}")
        time.sleep(90 / YAW_SPEED)
        self.send_command("rc 0 0 0 0")
        time.sleep(0.5)
        if self.pause_flow:
            self.pause_flow(False)

        print("AUTO: stepped sideways")

    # ── FORWARD PASS ──────────────────────────────────────
    def fly_pass(self):
        print(f"AUTO: pass {self.pass_count + 1}")
        self.send_command(f"rc 0 {FORWARD_SPEED} 0 0")
        start_time = time.time()
        last_keepalive = time.time()

        while self.running:
            now = time.time()

            if now - start_time < 2.0:
                time.sleep(0.05)
                continue

            if self.obstacle_ahead():
                self.send_command("rc 0 0 0 0")
                time.sleep(0.3)
                print("AUTO: wall detected")
                return

            if now - last_keepalive > KEEPALIVE_INTERVAL:
                self.send_command(f"rc 0 {FORWARD_SPEED} 0 0")
                last_keepalive = now

            time.sleep(0.05)

    # ── MAIN EXECUTE ──────────────────────────────────────
    def _execute(self):
        print("AUTO: _execute thread running")
        print("AUTO: flying forward")

        while self.running:
            self.fly_pass()

            if not self.running:
                break

            self.pass_count += 1
            print(f"AUTO: pass {self.pass_count} complete")
            self.step_sideways()
            self.turn_180()

        if self.running:
            self.send_command("rc 0 0 0 0")
            print("AUTO: hovering")

        self.running = False

    def get_status(self):
        return {
            "pass_count": self.pass_count,
            "running": self.running
        }