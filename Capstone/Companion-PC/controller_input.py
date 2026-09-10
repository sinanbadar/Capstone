import pygame
import time
import socket
import json
from enum import Enum

class ControlMode(Enum):
    MANUAL = "MANUAL"
    AUTONOMOUS = "AUTONOMOUS"
    GAZE = "GAZE"

class ControllerInput:
    def __init__(self, unity_ip="127.0.0.1", mode_port=9997):
        pygame.init()
        pygame.joystick.init()

        self.mode = ControlMode.MANUAL
        self.running = True
        self.deadzone = 0.12
        self.last_a_press = 0
        self.double_tap_threshold = 0.4
        self.button_cooldowns = {}

        self.unity_ip = unity_ip
        self.mode_port = mode_port
        self.mode_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        if pygame.joystick.get_count() == 0:
            print("No controller found")
            self.joystick = None
        else:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print(f"Controller connected: {self.joystick.get_name()}")

    def apply_deadzone(self, value):
        if abs(value) < self.deadzone:
            return 0.0
        sign = 1 if value > 0 else -1
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)

    def get_rc_values(self):
        if self.joystick is None:
            return 0, 0, 0, 0

        pygame.event.pump()

        left_x = self.apply_deadzone(self.joystick.get_axis(0))
        left_y = self.apply_deadzone(self.joystick.get_axis(1))
        right_x = self.apply_deadzone(self.joystick.get_axis(2))
        right_y = self.apply_deadzone(self.joystick.get_axis(3))

        left_right = int(left_x * 100)
        forward_back = int(-left_y * 100)
        up_down = int(-right_y * 100)
        yaw = int(right_x * 100)

        return left_right, forward_back, up_down, yaw

    def button_pressed(self, button_id, cooldown=0.3):
        now = time.time()
        last = self.button_cooldowns.get(button_id, 0)
        if self.joystick and self.joystick.get_button(button_id) and now - last > cooldown:
            self.button_cooldowns[button_id] = now
            return True
        return False

    def check_buttons(self):
        if self.joystick is None:
            return None

        pygame.event.pump()
        action = None

        # A button double tap: autonomous
        if self.button_pressed(0, cooldown=0.1):
            self.set_mode(ControlMode.AUTONOMOUS)
            print("Double tap A: AUTONOMOUS")

        # B button: manual
        if self.button_pressed(1):
            self.set_mode(ControlMode.MANUAL)
            print("B: MANUAL")

        # X button: gaze
        if self.button_pressed(2):
            self.set_mode(ControlMode.GAZE)
            print("X: GAZE")

        # RB: emergency stop
        if self.button_pressed(5, cooldown=1.0):
            action = "emergency"
            print("RB: EMERGENCY STOP")

        # Start: takeoff
        if self.button_pressed(7, cooldown=1.0):
            action = "takeoff"
            print("Start: TAKEOFF")

        # Back: land
        if self.button_pressed(6, cooldown=1.0):
            action = "land"
            print("Back: LAND")

        return action

    def set_mode(self, mode):
        self.mode = mode
        self.notify_unity_mode()

    def notify_unity_mode(self):
        try:
            message = json.dumps({"mode": self.mode.value}).encode()
            self.mode_sock.sendto(message, (self.unity_ip, self.mode_port))
            print(f"Unity notified: {self.mode.value}")
        except Exception as e:
            print(f"Could not notify Unity: {e}")

    def get_mode(self):
        return self.mode

    def stop(self):
        self.running = False
        self.mode_sock.close()
        pygame.quit()