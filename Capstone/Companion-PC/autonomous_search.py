import time
import math
import threading

# SEARCH PARAMETERS
FORWARD_SPEED = 30
YAW_SPEED = 50
YAW_DEG_PER_SEC = 30.0       # measured: speed 50 turned 180 deg in about 6s. Recalibrate if it drifts
SCAN_STEP_DEG = 30           # yaw increment while looking for an open direction
SETTLE_TIME = 0.7            # wait after each yaw step so the depth map is fresh
OPEN_DEPTH_THRESHOLD = 4.2   # sampled depth at or BELOW this counts as open (set from 2m reading)
VISITED_RADIUS = 0.5
LOOKAHEAD_M = 2.0
KEEPALIVE_INTERVAL = 5.0
POST_TURN_SLAM_DELAY = 4.0
OBSTACLE_THRESHOLD_M = 0.5
OBSTACLE_MIN_POINTS = 25
FLAT_WALL_Z_STD = 0.15
FLAT_WALL_MIN_POINTS = 20
FLAT_WALL_RADIUS = 2.0
STOP_DEPTH_THRESHOLD = 4.8   # keep equal to DepthEstimator(threshold=...) in tello_main.py

# Sensor switches, handy for debugging one sensor at a time
USE_DEPTH = True
USE_FLOW = True
USE_SLAM_OBSTACLE = True
USE_YOLO_OBSTACLE = True


class AutonomousSearch:
    def __init__(self, send_command_func, get_detections_func,
                get_position_func, get_point_cloud_func=None,
                get_optical_flow_func=None, pause_flow_func=None,
                get_depth_func=None, get_depth_map_func=None,
                get_centre_depth_func=None, get_fresh_depth_func=None,
                reset_depth_func=None):
        self.send_command = send_command_func
        self.get_detections = get_detections_func
        self.get_position = get_position_func
        self.get_point_cloud = get_point_cloud_func
        self.get_optical_flow = get_optical_flow_func
        self.pause_flow = pause_flow_func
        self.get_depth = get_depth_func
        self.get_depth_map = get_depth_map_func
        self.get_centre_depth = get_centre_depth_func
        self.get_fresh_depth = get_fresh_depth_func
        self.reset_depth = reset_depth_func
        self.running = False
        self.thread = None
        self.pass_count = 0
        self.is_yawing = False
        self.last_turn_time = 0
        self.visited = []
        self.current_heading = 0.0

    def start(self):
        if self.running:
            print("Already running")
            return
        self.running = True
        self.pass_count = 0
        self.last_turn_time = 0
        self.visited = []
        self.current_heading = 0.0
        self.thread = threading.Thread(target=self._execute, daemon=True)
        self.thread.start()
        print("AUTO: started")

    def stop(self):
        self.running = False
        self.is_yawing = False
        if self.pause_flow:
            self.pause_flow(False)
        self.send_command("rc 0 0 0 0")
        print("AUTO: stopped")

    # VISITED TRACKING (needs SLAM, harmless without it)
    def mark_visited(self):
        pos = self.get_position()
        if pos.get("tracking"):
            self.visited.append({"x": pos.get("x", 0), "z": pos.get("z", 0)})

    def is_visited(self, x, z):
        for v in self.visited:
            if math.hypot(v["x"] - x, v["z"] - z) < VISITED_RADIUS:
                return True
        return False

    # SLAM HELPERS
    def _is_forward_point(self, point, pos):
        px = point["x"] - pos.get("x", 0)
        py = point.get("y", 0)
        pz = point["z"] - pos.get("z", 0)
        dist = math.sqrt(px**2 + pz**2)
        if dist > OBSTACLE_THRESHOLD_M or py < 0.2:
            return False
        dot = pz / dist if dist > 0 else 0
        return dot > 0.93

    def flat_wall_ahead(self, cloud, pos):
        if not cloud:
            return False
        dx, dz = pos.get("x", 0), pos.get("z", 0)
        nearby = [p for p in cloud
                  if math.hypot(p["x"] - dx, p["z"] - dz) < FLAT_WALL_RADIUS
                  and p.get("y", 0) > 0.2]
        if len(nearby) < FLAT_WALL_MIN_POINTS:
            return False
        zs = [p["z"] for p in nearby]
        return (max(zs) - min(zs)) < FLAT_WALL_Z_STD

    # OBSTACLE DETECTION (returns a reason string or None)
    def _check_obstacle(self):
        just_turned = time.time() - self.last_turn_time < POST_TURN_SLAM_DELAY

        if USE_DEPTH and self.get_depth and self.get_depth():
            return "depth"

        if USE_FLOW and self.get_optical_flow and self.get_optical_flow():
            return "flow"

        if USE_SLAM_OBSTACLE and not just_turned and self.get_point_cloud:
            pos = self.get_position()
            if pos.get("tracking"):
                cloud = self.get_point_cloud()
                if cloud:
                    fwd = [p for p in cloud if self._is_forward_point(p, pos)]
                    if len(fwd) >= OBSTACLE_MIN_POINTS:
                        return "slam"
                    if self.flat_wall_ahead(cloud, pos):
                        return "flat_wall"

        if USE_YOLO_OBSTACLE and not just_turned:
            for d in self.get_detections():
                b = d["bbox"]
                fill = ((b["x2"] - b["x1"]) * (b["y2"] - b["y1"])) / (640 * 480)
                cx = (b["x1"] + b["x2"]) / 2
                if fill > 0.5 and 200 < cx < 440:
                    return f"yolo:{d['label']}"
        return None

    def obstacle_ahead(self):
        reason = self._check_obstacle()
        if reason:
            depth = self.get_centre_depth() if self.get_centre_depth else None
            tracking = self.get_position().get("tracking")
            d = f"{depth:.2f}" if depth is not None else "n/a"
            print(f"STOP reason={reason} depth={d} slam={tracking} pass={self.pass_count + 1}")
            return True
        return False

    # DEPTH SAMPLE (raw reading, does not touch the trigger counter)
    def sample_depth_ahead(self):
        if self.get_centre_depth:
            return self.get_centre_depth()
        return 99.0  # unknown, treat as blocked

    # YAW HELPER
    def _yaw(self, degrees):
        """Positive = clockwise. Timed, so approximate."""
        if abs(degrees) < 1:
            return
        if self.pause_flow:
            self.pause_flow(True)
        self.is_yawing = True
        speed = YAW_SPEED if degrees > 0 else -YAW_SPEED
        self.send_command(f"rc 0 0 0 {speed}")
        time.sleep(abs(degrees) / YAW_DEG_PER_SEC)
        self.send_command("rc 0 0 0 0")
        time.sleep(0.3)
        self.is_yawing = False
        self.last_turn_time = time.time()
        if self.pause_flow:
            self.pause_flow(False)
        self.current_heading = (self.current_heading + degrees) % 360
        if self.reset_depth:
            self.reset_depth()

    def path_is_clear(self):
        depth = self.sample_depth_ahead()
        print(f"AUTO: preflight depth {depth:.2f} (stop at {STOP_DEPTH_THRESHOLD})")
        return depth < STOP_DEPTH_THRESHOLD

    # FIND NEXT OPEN, UNVISITED DIRECTION
    def find_next_direction(self):
        print("AUTO: looking for an open direction")
        offsets = [30, -30, 60, -60, 90, -90, 120, -120, 150, -150, 180]
        rel = 0
        best_depth, best_off = 999.0, 0

        for off in offsets:
            if not self.running:
                return False
            self._yaw(off - rel)
            rel = off
            self._yaw(off - rel)
            rel = off
            if self.get_fresh_depth:
                depth = self.get_fresh_depth(time.time() + 0.4)
            else:
                depth = self.sample_depth_ahead()
            print(f"AUTO: offset {off} depth {depth:.2f}")
            print(f"AUTO: offset {off} depth {depth:.2f}")
            if depth < best_depth:
                best_depth, best_off = depth, off

            if depth <= OPEN_DEPTH_THRESHOLD:
                pos = self.get_position()
                rad = math.radians(self.current_heading)
                tx = pos.get("x", 0) + math.sin(rad) * LOOKAHEAD_M
                tz = pos.get("z", 0) + math.cos(rad) * LOOKAHEAD_M
                if not self.is_visited(tx, tz):
                    print(f"AUTO: open direction at offset {off}")
                    return True

        # Nothing met the open threshold: face the most open direction seen
        if best_depth < OPEN_DEPTH_THRESHOLD + 0.8:
            self._yaw(best_off - rel)
            print(f"AUTO: taking best direction, offset {best_off}")
            return True

        print("AUTO: boxed in, no usable direction")
        return False

    # FORWARD PASS
    def fly_pass(self):
        print(f"AUTO: pass {self.pass_count + 1} starting")
        self.send_command(f"rc 0 {FORWARD_SPEED} 0 0")
        start = time.time()
        last_keepalive = time.time()
        last_mark = 0

        while self.running:
            now = time.time()
            if now - start < 1.0:
                time.sleep(0.05)
                continue

            if now - last_mark > 1.0:
                self.mark_visited()
                last_mark = now

            if self.obstacle_ahead():
                self.send_command("rc 0 0 0 0")
                time.sleep(0.3)
                return

            if now - last_keepalive > KEEPALIVE_INTERVAL:
                self.send_command(f"rc 0 {FORWARD_SPEED} 0 0")
                last_keepalive = now

            time.sleep(0.05)

    # MAIN LOOP
    def _execute(self):
        print("AUTO: reactive DFS exploration starting")

        while self.running:
            self.fly_pass()
            if not self.running:
                break
            self.pass_count += 1
            if not self.find_next_direction():
                print("AUTO: search complete, no open directions")
                break

        self.send_command("rc 0 0 0 0")
        self.running = False

    def get_status(self):
        return {
            "pass_count": self.pass_count,
            "running": self.running,
            "visited_count": len(self.visited),
        }