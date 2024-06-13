#!/usr/bin/env python3
import rospy
from control_msgs.msg import GripperCommandActionGoal
from utils.Gripper import Gripper
from utils.EventManager import EventManager


class GripClient():

    def __init__(self, args):
        self.grip_topic = rospy.Publisher('/gripper_controller/cmd/goal', GripperCommandActionGoal, queue_size=1)
        self.gripper = Gripper(args)
        self.event_manager = EventManager()
        self.on_open_or_close()
    
    def on_open_or_close(self):
        self.gripper.on_open_or_close()
        self.send_gripper_action()
    
    def send_gripper_action(self):
        grip_action = GripperCommandActionGoal()
        grip_action.goal.command.position = self.gripper.objective_width
        grip_action.goal.command.max_effort = self.gripper.objective_force
        #print("COMMAND: ", grip_action)
        self.grip_topic.publish(grip_action)

    def get_width_variation(self):
        return self.gripper.get_width_variation()
    
    def get_grip_force(self):
        return self.gripper.objective_force

    def is_movement_detected(self):
        return self.gripper.is_movement_detected()
    
    def is_width_correct(self, rec_width):
        return self.gripper.is_width_correct(rec_width)

    def update_time_received(self, msg_received):
        self.event_manager.update_time_received(msg_received)

    def update_width(self, rec_width):
        self.gripper.update_width(rec_width)

    def on_objective_change(self):
        if self.gripper.same_objective_position():
            self.event_manager.not_objective_changed()
        else:
            self.event_manager.yes_objective_changed()
        self.gripper.update_objective_position()

    def process_grip_level(self, grip_level):
        grip_level = self.gripper.limit_grip_level(grip_level)
        self.event_manager.detect_grip(grip_level)
        self.gripper.configure_width(grip_level)
        self.on_objective_change()

    def configure_force(self, static_force):
        time_delta = self.event_manager.get_time_difference()
        self.gripper.configure_force(static_force, time_delta)
        self.event_manager.update_current_time()

    def update_detection(self):
        self.event_manager.update_detection(self.gripper)

    def process_gripper_state(self):
        force_feedback = 0.0

        if self.event_manager.grip_detected: # tool precision
            print("GRIP DETECTED")
            if not self.gripper.grip_within_limits():
                self.gripper.limit_movement()
            else:
                self.update_detection()
                
            width_var = self.gripper.get_width_variation()
            if self.event_manager.object_detected: #self.object_detected:
                if width_var <= 20:
                    print("START LOW VIBRATION")
                elif width_var <= 50:
                    print("START MEDIUM VIBRATION")
                elif width_var <= 80:
                    print("START STRONG VIBRATION")
            else:
                print("VIBRATION ONGOING")
            #force_feedback = self.configure_feedback(width_var) # HAPTIC FEEDBACK

        # IDEA: Determinar Force Feedback por Tiempo, o por Var Width o por Val Discretos PENDING

    def gripper_operation(self):
        self.send_gripper_action()
        #time.sleep(1) # TAL VEZ INNECESARIO
        if not self.event_manager.objective_changed:
            self.process_gripper_state()
        else:
            self.gripper.configure_delay()

