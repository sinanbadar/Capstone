import time
import math
import threading

# SEARCH PARAMETERS
FORWARD_SPEED = 20
YAW_SPEED = 60
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
STOP_DEPTH_THRESHOLD = 4.0   # keep equal to DepthEstimator(threshold=...) in tello_main.py
ALLOW_REVISIT = True     # if only visited headings are open, take the most open one before giving up

# Sensor switches, handy for debugging one sensor at a time
USE_DEPTH = True
USE_FLOW = True
USE_SLAM_OBSTACLE = True
USE_YOLO_OBSTACLE = True
EST_SPEED_MPS = 0.45


class AutonomousSearch:
    def __init__(self, send_command_func, get_detections_func,
                get_position_func, get_point_cloud_func=None,
                get_optical_flow_func=None, pause_flow_func=None,
                get_depth_func=None, get_depth_map_func=None,
                get_centre_depth_func=None, get_fresh_depth_func=None,
                reset_depth_func=None, alert_func=None, map_update_func=None, room_map=None):
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
        self.send_alert = alert_func
        self.map_update = map_update_func
        self.room_map = room_map
        self.rel = 0    
        self.est_x = 0.0
        self.est_z = 0.0
        self.stop_log = []
        self.on_stop = None 
        

    def start(self):
        if self.running:
            print("Already running")
            return
        self.rel = 0
        self.est_x = 0.0
        self.est_z = 0.0
        self.stop_log = []
        self.running = True
        self.pass_count = 0
        self.last_turn_time = 0
        self.visited = []
        self.current_heading = 0.0
        self.thread = threading.Thread(target=self._execute, daemon=True)
        self.thread.start()
        print("AUTO: started")
        if self.send_alert:
            self.send_alert(False, "")

    def stop(self):
        self.running = False
        self.is_yawing = False
        if self.pause_flow:
            self.pause_flow(False)
        self.send_command("rc 0 0 0 0")
        if self.send_alert:
            self.send_alert(False, "")
        print("AUTO: stopped")

    # VISITED TRACKING (needs SLAM, harmless without it)
    def _pos(self):
        p = self.get_position()
        if p.get("tracking"):
            return p.get("x", 0.0), p.get("z", 0.0)
        return self.est_x, self.est_z

    def mark_visited(self):
        x, z = self._pos()
        self.visited.append({"x": x, "z": z})

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
            self.stop_log.append({"reason": reason, "depth": depth})
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

    # FIND NEXT OPEN, UNVISITED DIRECTION
    def _turn_to(self, target):
        """Turn to an angle relative to the blocked heading, by the shortest way."""
        diff = (target - self.rel + 180) % 360 - 180
        self._yaw(diff)
        self.rel = target

    def _scan_arc(self, angles):
        """Visit each angle, sample fresh depth, return the open ones as (angle, depth, visited)."""
        found = []
        for a in angles:
            if not self.running:
                return None
            self._turn_to(a)
            if self.get_fresh_depth:
                depth = self.get_fresh_depth(time.time() + 0.4)
            else:
                depth = self.sample_depth_ahead()
            x, z = self._pos()
            rad = math.radians(self.current_heading)
            if self.map_update:
                self.map_update(self.est_x, self.est_z, self.current_heading)
            visited = self.is_visited(x + math.sin(rad) * LOOKAHEAD_M,
                                    z + math.cos(rad) * LOOKAHEAD_M)
            print(f"AUTO: angle {a:4d}  depth {depth:.2f}  visited={visited}")
            if depth <= OPEN_DEPTH_THRESHOLD:
                found.append((a, depth, visited))
        return found
    
    def _try_frontier(self):
        if not self.room_map:
            return None
        target = self.room_map.nearest_frontier(self.est_x, self.est_z)
        if target is None:
            return None
        bearing, dist = target
        diff = (bearing - self.current_heading + 180) % 360 - 180
        self._yaw(diff)
        self.rel = 0
        if self.get_fresh_depth and self.get_fresh_depth(time.time() + 0.4) <= OPEN_DEPTH_THRESHOLD:
            print(f"AUTO: heading to frontier {dist:.1f}m away")
            return True
        return None
    
    def find_next_direction(self):
        print("AUTO: blocked, scanning")
        if self._try_frontier():
            return True
        self.rel = 0
        phases = [
            ("left",  [-30, -60, -90]),
            ("right", [30, 60, 90]),
            ("rear",  [120, 150, 180, 210, 240]),
        ]
        revisit = []

        for name, angles in phases:
            if name == "right":
                self._turn_to(0)        # transit back through the blocked heading, no sample
            found = self._scan_arc(angles)
            if found is None:           # operator interrupted
                return False
            fresh = [c for c in found if not c[2]]
            revisit += [c for c in found if c[2]]
            if fresh:
                best = min(fresh, key=lambda c: c[1])
                self._turn_to(best[0])
                print(f"AUTO: {name} arc, going {best[0]} degrees (depth {best[1]:.2f})")
                return True
            print(f"AUTO: nothing open on the {name} arc")

        if ALLOW_REVISIT and revisit:
            best = min(revisit, key=lambda c: c[1])
            self._turn_to(best[0])
            print(f"AUTO: only visited headings open, revisiting {best[0]} degrees")
            return True

        return False

    def _hover_and_alert(self, message):
        print(f"AUTO: {message}. Hovering, manual control needed")
        while self.running:
            self.send_command("rc 0 0 0 0")        # keeps the Tello from timing out
            if self.send_alert:
                self.send_alert(True, message)
            time.sleep(1.0)

    def verify_heading(self, settle=0.5):
        """Pause briefly and confirm the chosen heading is open. Returns True if clear."""
        self.send_command("rc 0 0 0 0")
        time.sleep(settle)
        if self.get_fresh_depth:
            depth = self.get_fresh_depth(time.time() + 0.3)
        else:
            depth = self.sample_depth_ahead()
        ok = depth <= OPEN_DEPTH_THRESHOLD
        print(f"AUTO: verify heading depth {depth:.2f} -> {'clear' if ok else 'BLOCKED'}")
        return ok

    # FORWARD PASS
    def fly_pass(self):
        print(f"AUTO: pass {self.pass_count + 1} starting")
        self.send_command(f"rc 0 {FORWARD_SPEED} 0 0")
        start = time.time()
        last_keepalive = time.time()
        last_mark = 0
        last_map = 0
        last_t = time.time()

        while self.running:
            now = time.time()
            dt = now - last_t
            last_t = now
            rad = math.radians(self.current_heading)
            self.est_x += math.sin(rad) * EST_SPEED_MPS * dt
            self.est_z += math.cos(rad) * EST_SPEED_MPS * dt
            if self.map_update and now - last_map > 0.3 and not self.is_yawing:
                self.map_update(self.est_x, self.est_z, self.current_heading)
                last_map = now

            if now - start < 1.0:
                time.sleep(0.05)
                continue

            if now - last_mark > 1.0:
                self.mark_visited()
                print(f"EST pos {self.est_x:.2f}, {self.est_z:.2f} heading {self.current_heading:.0f}")
                last_mark = now

            if self.obstacle_ahead():
                self.send_command("rc 0 0 0 0")
                time.sleep(0.3)
                if self.on_stop:
                    self.on_stop(self.stop_log[-1] if self.stop_log else {})
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

            chosen = self.find_next_direction()
            tries = 0
            while chosen and self.running and not self.verify_heading() and tries < 2:
                tries += 1
                chosen = self.find_next_direction()      # rescan from the new heading
            if chosen and not self.verify_heading():
                chosen = False                            # still blocked after retries

            if not chosen:
                if not self.running:
                    break
                if self.room_map and not self.room_map.frontiers():
                    print("AUTO: room fully explored")
                self._hover_and_alert("No open path found, manual control required")
                break

        self.send_command("rc 0 0 0 0")
        self.running = False

    def get_status(self):
        return {
            "pass_count": self.pass_count,
            "running": self.running,
            "visited_count": len(self.visited),
        }