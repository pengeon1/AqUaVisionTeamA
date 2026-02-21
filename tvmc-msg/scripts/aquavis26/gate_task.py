#!/usr/bin/env python3
import time
import threading

import rospy
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Vector3
from statemachine import StateMachine, State
from tvmc import MotionController, DoF, ControlMode
import collections

# globals
CURRENT_YAW = 0
START_YAW = 0
REVERSE_YAW = -1
DATA_SOURCE = "sensors"

HEAVE_KP = -40 
HEAVE_KI = 0.09 
HEAVE_KD = 4.5 
HEAVE_TARGET = 0.3 
HEAVE_ACCEPTABLE_ERROR = 0.01
HEAVE_OFFSET = -0.11 

PITCH_KP = -0.12 
PITCH_KI = 0.0015
PITCH_KD = 0.2 
PITCH_TARGET = 0
PITCH_ACCEPTABLE_ERROR = 0.7 
PITCH_OFFSET = -0.5 

ROLL_KP = 0.1 
ROLL_KI = 0
ROLL_KD = 0.4
ROLL_TARGET = 0
ROLL_ACCEPTABLE_ERROR = 1.5

YAW_KP = 0
YAW_KI = 0
YAW_KD = 0
YAW_TARGET = 0
YAW_ACCEPTABLE_ERROR = 0


class QualificationTask(StateMachine):
    wait_to_start = State(initial=True)
    initializing_sensors = State()
    fixing_yaw = State()
    enabling_heave_pid = State()
    tracking_gate = State()
    finished = State(final=True)

    start_initializing_sensors = wait_to_start.to(initializing_sensors)
    fix_yaw = initializing_sensors.to(fixing_yaw)
    heave_down = fixing_yaw.to(enabling_heave_pid)
    start_gate_tracking = enabling_heave_pid.to(tracking_gate)
    finish = tracking_gate.to(finished)

    def __init__(self):
        self.m = MotionController()
        self.yaw_lock = None
        self.orientation_sub = None
        self.depth_sub = None
        self.vision_sub = None
        self.current_yaw = None
        self.current_depth = None
        
        # Variable for dynamic depth adjusting based on vision
        self.current_heave_target = HEAVE_TARGET

        super(QualificationTask, self).__init__()

    def on_enter_wait_to_start(self):
        print("Waiting to start.")
        self.m.start()
        self.start_initializing_sensors()

    def on_depth(self, depth: Float32):
        self.current_depth = depth.data
        self.m.set_current_point(DoF.HEAVE, depth.data)

    def set_yaw(self, angle):
        self.yaw_lock = angle
        self.m.set_target_point(DoF.YAW, angle)

    def on_orientation(self, vec: Vector3):
        self.current_yaw = vec.vector.z
        self.m.set_current_point(DoF.YAW, vec.vector.z)

    def on_enter_initializing_sensors(self):
        print("Initializing Sensors.")
        self.m.set_pid_constants(DoF.YAW, YAW_KP, YAW_KI, YAW_KD, YAW_ACCEPTABLE_ERROR)
        self.orientation_sub = rospy.Subscriber(
            "/euler", Vector3Stamped, self.on_orientation
        )
        self.depth_sub = rospy.Subscriber(f"/{DATA_SOURCE}/depth", Float32, self.on_depth)

        time.sleep(5)

        # Yaw lock logic
        samples = [173]
        samples = [(360 - x if x >= 180 else x) for x in samples]
        print(samples)
        self.yaw_lock = (sum(samples) / len(samples))
        self.yaw_lock = self.yaw_lock + 360 if self.yaw_lock < 0 else self.yaw_lock
        print("Yaw locked at: ", self.yaw_lock)

        self.fix_yaw()

    def on_enter_fixing_yaw(self):
        print("Attempting to fix yaw.")
        self.heave_down()

    def on_enter_enabling_heave_pid(self):
        print("Enabling heave PID.")
        self.m.set_pid_constants(
            DoF.HEAVE,
            HEAVE_KP,
            HEAVE_KI,
            HEAVE_KD,
            HEAVE_ACCEPTABLE_ERROR,
            HEAVE_OFFSET,
        )
        self.m.set_pid_limits(DoF.HEAVE, -10, 10, -25, 25)
        self.m.set_control_mode(DoF.HEAVE, ControlMode.CLOSED_LOOP)
        self.m.set_target_point(DoF.HEAVE, self.current_heave_target)

        while (
            self.current_depth is None
            or abs(self.current_depth - self.current_heave_target) > HEAVE_ACCEPTABLE_ERROR * 3
        ):
            time.sleep(0.1)
        
        # Move directly into vision tracking once depth is hit
        self.start_gate_tracking()

    def enable_pitch_pid(self):
        self.m.set_pid_constants(
            DoF.PITCH, PITCH_KP, PITCH_KI, PITCH_KD, PITCH_ACCEPTABLE_ERROR
        )
        self.m.set_target_point(DoF.PITCH, PITCH_TARGET)
        self.m.set_control_mode(DoF.PITCH, ControlMode.CLOSED_LOOP)
    
    def disable_pitch_pid(self):
        self.m.set_control_mode(DoF.PITCH, ControlMode.OPEN_LOOP)

    def enable_yaw_pid(self):
        self.m.set_pid_constants(
            DoF.YAW, YAW_KP, YAW_KI, YAW_KD, YAW_ACCEPTABLE_ERROR
        )
        self.m.set_control_mode(DoF.YAW, ControlMode.CLOSED_LOOP)

    def enable_heave_pid(self):
        self.m.set_pid_constants(
            DoF.HEAVE, HEAVE_KP, HEAVE_KI, HEAVE_KD, HEAVE_ACCEPTABLE_ERROR, HEAVE_OFFSET
        )
        self.m.set_pid_limits(DoF.HEAVE, -10, 10, -25, 25)
        self.m.set_control_mode(DoF.HEAVE, ControlMode.CLOSED_LOOP)

    # ==========================================================
    # VISION TRACKING LOGIC
    # ==========================================================

    def on_enter_tracking_gate(self):
        print("Gate Tracking Started. Listening to Vision Node...")
        self.enable_pitch_pid() 
        
        # Subscribe to our new vision command topic
        self.vision_sub = rospy.Subscriber("/vision/gate_cmd", String, self.on_vision_cmd)

    def on_vision_cmd(self, msg):
        # We only want vision controlling the bot in the tracking_gate state
        if not self.current_state.id == "tracking_gate":
            return

        cmd = msg.data
        
        # 1. ALWAYS ZERO OUT OPEN-LOOP THRUSTS FIRST
        self.m.set_thrust(DoF.SURGE, 0)
        self.m.set_thrust(DoF.SWAY, 0)

        # 2. APPLY ONLY THE COMMANDED DoF
        if cmd == "HEAVE_UP":
            self.current_heave_target -= 0.05 
            self.m.set_target_point(DoF.HEAVE, self.current_heave_target)
            
        elif cmd == "HEAVE_DOWN":
            self.current_heave_target += 0.05 
            self.m.set_target_point(DoF.HEAVE, self.current_heave_target)
            
        elif cmd == "SWAY_LEFT":
            self.m.set_control_mode(DoF.SWAY, ControlMode.OPEN_LOOP)
            self.m.set_thrust(DoF.SWAY, -40) 
            
        elif cmd == "SWAY_RIGHT":
            self.m.set_control_mode(DoF.SWAY, ControlMode.OPEN_LOOP)
            self.m.set_thrust(DoF.SWAY, 40) 
            
        elif cmd == "SURGE_FORWARD":
            self.m.set_control_mode(DoF.SURGE, ControlMode.OPEN_LOOP)
            self.m.set_thrust(DoF.SURGE, 50) 
            
        elif cmd == "SEARCH":
            # Oscillating sway pattern to search for the gate
            # 6.0 seconds per full sweep (3s right, 3s left)
            cycle_time = 6.0 
            current_time = time.time()
            self.m.set_control_mode(DoF.SWAY, ControlMode.OPEN_LOOP)
            
            if (current_time % cycle_time) < (cycle_time / 2):
                self.m.set_thrust(DoF.SWAY, 30)  # Gentle sway right
            else:
                self.m.set_thrust(DoF.SWAY, -30) # Gentle sway left
                
        elif cmd == "PASS_THROUGH":
            print("Executing Final Pass-Through!")
            self.m.set_control_mode(DoF.SURGE, ControlMode.OPEN_LOOP)
            self.m.set_thrust(DoF.SURGE, 70) 
            time.sleep(4) # Drive forward for 4 seconds to clear the gate
            self.m.set_thrust(DoF.SURGE, 0)
            self.finish() # End task

    def on_enter_finished(self):
        print("Task Finished. Maintaining Depth.")
        self.m.set_thrust(DoF.SURGE, 0)
        self.m.set_thrust(DoF.SWAY, 0)
        self.m.set_control_mode(DoF.HEAVE, ControlMode.CLOSED_LOOP)


if __name__ == "__main__":
    rospy.init_node('auv_control_node') 
    task = QualificationTask()
    rospy.spin()
