#!/usr/bin/env python3
from time import sleep
from datetime import datetime
from utils.HapticLimitator import HapticLimitator
from utils.HapticGlove import HapticGlove
from utils.GripperTwin import GripperTwin
from bhaptics import better_haptic_player as player
from bhaptics.better_haptic_player import BhapticsPosition

class HapticClient():

    def __init__(self, args):
        self.limitator = HapticLimitator(args)
        self.glove = HapticGlove(args)
        self.gripper = GripperTwin(args)
        self.time_received = datetime.now()
        self.current_time = datetime.now()
        
        player.initialize()
        print("register CenterX")
        player.register("CenterX", "CenterX.tact")
        print("register Circle")
        player.register("Circle", "Circle.tact")
        self.sample_initial_sensation()

    def sample_initial_sensation(self):
        print("SAMPLE SENSATION")
        self.activate_fingers(self.glove.precision_duration)

    def update_current_time(self):
        self.current_time = self.time_received

    def configure_fingers(self, fingers_recv):
        self.glove.current_fingers = self.limitator.configure_fingers(fingers_recv)

    def get_return_message(self):
        return [self.gripper.grip_width_var, self.gripper.grip_force, self.glove.current_fingers]
    
    def is_haptic_grip_detected(self):
        grip_detected = self.gripper.is_grip_detected()
        fingers_detected = self.glove.are_fingers_detected()
        closed_grip_received = self.glove.is_hand_closed()
        return (grip_detected and fingers_detected) or closed_grip_received

    def activate_fingers(self, duration_msec):
        glove_actuators = self.glove.configure_actuators()
        print("DUR: ", duration_msec)
        print("GLOVE: ", glove_actuators)
        if self.glove.right_hand_enabled:
            player.submit_dot("gloveRFrame", BhapticsPosition.GloveR.value, glove_actuators, int(duration_msec))
        else:
            player.submit_dot("gloveLFrame", BhapticsPosition.GloveL.value, glove_actuators, int(duration_msec))
        sleep(duration_msec*0.001) # Duration in seconds needed
    
    def configure_current_intensity(self):
        self.glove.configure_current_intensity(self.gripper.grip_width_var, self.gripper.grip_force, self.limitator)

    def activate_glove(self):
        time_delta = self.time_received - self.current_time
        time_diff_msec = round(float(time_delta.microseconds)*0.001, 3) 
        # MAPEAR FORCE Y WIDTH VAR TO HAPTIC INTENSITY - DONE
        self.configure_current_intensity()
        # CONSIDERAR TODOS LOS DELAYS, Y LIMITACIONES INTENSITY - PENDING
        if self.glove.discrete_sensations:
            # LOOPING THROUGH DISCRETIZED PRECISION_DURATION TIMES
            time_msec = 0.0
            while time_msec <= time_diff_msec:
                self.activate_fingers(self.glove.precision_duration)
                time_msec = time_msec + self.glove.precision_duration
        else:
            # USING CURRENT_TIME - TIME_RECEIVED CONTINOUS TIME
            self.activate_fingers(time_diff_msec)
        self.update_current_time()

    def update_prev_width(self):
        print("ACTUALIZO WIDTH")
        self.gripper.update_prev_width()

    def is_close_detected(self):
        return self.gripper.is_close_detected()
    
    def on_close(self):
        self.turn_gloves_off()
        player.destroy()

    def turn_gloves_off(self):
        self.glove.on_close()
        self.activate_fingers(self.glove.precision_duration)