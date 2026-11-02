import pygame
import time
import socket
import json
from enum import Enum


class ControlMode(Enum):
    MANUAL = "MANUAL"
    AUTONOMOUS = "AUTONOMOUS"


class ControllerInput:
    BTN_A = 0
    BTN_B = 1
    BTN_RB = 5
    BTN_START = 6     # takeoff, confirmed on this controller
    BTN_BACK = 4      # land, confirmed on this controller

    def __init__(self, unity_ip="127.0.0.1", mode_port=9997):
        pygame.init()
        pygame.joystick.init()

        self.mode = ControlMode.MANUAL
        self.running = True
        self.deadzone = 0.12
        self.prev_state = {}
        self.cooldowns = {}

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

        return (int(left_x * 100), int(-left_y * 100),
                int(-right_y * 100), int(right_x * 100))

    def just_pressed(self, button_id, cooldown=0.0):
        """True once per physical press (rising edge), with an optional cooldown."""
        if self.joystick is None:
            return False
        down = bool(self.joystick.get_button(button_id))
        was_down = self.prev_state.get(button_id, False)
        self.prev_state[button_id] = down
        if down and not was_down:
            now = time.time()
            if now - self.cooldowns.get(button_id, 0.0) >= cooldown:
                self.cooldowns[button_id] = now
                return True
        return False

    def check_buttons(self):
        if self.joystick is None:
            return None

        pygame.event.pump()
        action = None

        # A: double tap for autonomous
        if self.just_pressed(self.BTN_A):
            self.set_mode(ControlMode.AUTONOMOUS)
            print("A: AUTONOMOUS")

        # B: manual
        if self.just_pressed(self.BTN_B):
            self.set_mode(ControlMode.MANUAL)
            print("B: MANUAL")

        # RB: emergency stop
        if self.just_pressed(self.BTN_RB):
            action = "emergency"
            print("RB: EMERGENCY STOP")

        # Start: takeoff
        if self.just_pressed(self.BTN_START, cooldown=1.0):
            action = "takeoff"
            print("Start: TAKEOFF")

        # Back: land
        if self.just_pressed(self.BTN_BACK, cooldown=1.0):
            action = "land"
            print("Back: LAND")

        return action

    def set_mode(self, mode):
        if mode == self.mode:
            return
        self.mode = mode
        self.notify_unity_mode()

    def notify_unity_mode(self):
        try:
            message = json.dumps({"mode": self.mode.value}).encode()
            self.mode_sock.sendto(message, (self.unity_ip, self.mode_port))
        except Exception as e:
            print(f"Could not notify Unity: {e}")

    def get_mode(self):
        return self.mode

    def stop(self):
        self.running = False
        self.mode_sock.close()
        pygame.quit()