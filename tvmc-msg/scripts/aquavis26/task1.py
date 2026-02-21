#!/usr/bin/env python3
import time
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from statemachine import StateMachine, State
from tvmc import MotionController, DoF, ControlMode

# ==========================================
# CONSTANTS & PID TUNING
# ==========================================
DATA_SOURCE = "sensors"

# Path Tracking Specifics
SURGE_THRUST = 50        # Constant forward thrust
TRACKING_DURATION = 6.0  # Seconds to travel ~3 meters (Tune this empirically)

# Heave (Depth)
HEAVE_KP = -40 
HEAVE_KI = 0.09 
HEAVE_KD = 4.5 
HEAVE_TARGET = 0.4       # Depth to maintain during the run
HEAVE_ACCEPTABLE_ERROR = 0.05
HEAVE_OFFSET = -0.11 

# Yaw (Heading)
# NOTE: These need to be well-tuned to keep the sub strictly on a straight line
YAW_KP = 1.5 
YAW_KI = 0.01
YAW_KD = 0.5
YAW_ACCEPTABLE_ERROR = 2.0 # Degrees of acceptable heading error

class PathTrackingTask(StateMachine):
    wait_to_start = State(initial=True)
    initializing_sensors = State()
    stabilizing = State()
    tracking_path = State()
    finished = State(final=True)

    # Transitions
    start_initializing = wait_to_start.to(initializing_sensors)
    sensors_ready = initializing_sensors.to(stabilizing)
    stabilized = stabilizing.to(tracking_path)
    finish = tracking_path.to(finished)

    def __init__(self):
        self.m = MotionController()
        self.orientation_sub = None
        self.depth_sub = None
        
        self.current_yaw = None
        self.current_depth = None
        self.yaw_lock = None

        super(PathTrackingTask, self).__init__()

    # ==========================================
    # SENSOR CALLBACKS
    # ==========================================
    def on_depth(self, depth: Float32):
        self.current_depth = depth.data
        self.m.set_current_point(DoF.HEAVE, depth.data)

    def on_orientation(self, vec: Vector3):
        self.current_yaw = vec.z
        self.m.set_current_point(DoF.YAW, vec.z)

    # ==========================================
    # STATE HOOKS
    # ==========================================
    def on_enter_wait_to_start(self):
        print("Waiting to start Path Tracking.")
        self.m.start()
        self.start_initializing()

    def on_enter_initializing_sensors(self):
        print("Initializing Sensors and determining initial heading.")
        
        # Subscribe to IMU and Depth
        self.orientation_sub = rospy.Subscriber(
            f"/{DATA_SOURCE}/orientation", Vector3, self.on_orientation
        )
        self.depth_sub = rospy.Subscriber(
            f"/{DATA_SOURCE}/depth", Float32, self.on_depth
        )

        # Allow sensors to warm up and publish initial data
        time.sleep(3)

        # Lock onto the current yaw to maintain a straight line
        if self.current_yaw is not None:
            self.yaw_lock = self.current_yaw
        else:
            self.yaw_lock = 0.0 # Fallback if IMU fails to publish
            
        print(f"Yaw locked at: {self.yaw_lock} degrees.")
        self.sensors_ready()

    def on_enter_stabilizing(self):
        print("Stabilizing Depth and Heading.")
        
        # 1. Enable Heave PID
        self.m.set_pid_constants(
            DoF.HEAVE, HEAVE_KP, HEAVE_KI, HEAVE_KD, HEAVE_ACCEPTABLE_ERROR, HEAVE_OFFSET
        )
        self.m.set_pid_limits(DoF.HEAVE, -10, 10, -25, 25)
        self.m.set_target_point(DoF.HEAVE, HEAVE_TARGET)
        self.m.set_control_mode(DoF.HEAVE, ControlMode.CLOSED_LOOP)

        # 2. Enable Yaw PID
        self.m.set_pid_constants(
            DoF.YAW, YAW_KP, YAW_KI, YAW_KD, YAW_ACCEPTABLE_ERROR
        )
        self.m.set_target_point(DoF.YAW, self.yaw_lock)
        self.m.set_control_mode(DoF.YAW, ControlMode.CLOSED_LOOP)

        # Wait until depth and heading are within acceptable margins before surging
        while not rospy.is_shutdown():
            if self.current_depth is None or self.current_yaw is None:
                continue
                
            depth_error = abs(self.current_depth - HEAVE_TARGET)
            
            # Handle yaw wrap-around (0-360 degrees)
            yaw_error = abs((self.current_yaw - self.yaw_lock + 180) % 360 - 180)

            if depth_error <= HEAVE_ACCEPTABLE_ERROR * 3 and yaw_error <= YAW_ACCEPTABLE_ERROR * 2:
                break
                
            time.sleep(0.1)

        print("AUV Stabilized. Proceeding to path tracking.")
        self.stabilized()

    def on_enter_tracking_path(self):
        print(f"Tracking Path: Surging forward at thrust {SURGE_THRUST} for {TRACKING_DURATION}s.")
        
        # Maintain closed-loop Yaw and Heave, but open-loop Surge
        self.m.set_control_mode(DoF.SURGE, ControlMode.OPEN_LOOP)
        self.m.set_thrust(DoF.SURGE, SURGE_THRUST)

        # Wait for the calculated duration to cross 3 meters
        time.sleep(TRACKING_DURATION)
        
        self.finish()

    def on_enter_finished(self):
        print("Path Tracking Complete. Halting forward motion.")
        # Kill forward momentum
        self.m.set_thrust(DoF.SURGE, 0)
        self.m.set_control_mode(DoF.SURGE, ControlMode.CLOSED_LOOP) # Optional: set to closed loop at 0 to brake
        
        # Maintain final depth and heading
        print("Maintaining position.")

if __name__ == "__main__":
    rospy.init_node('path_tracking_node')
    task = PathTrackingTask()
    rospy.spin()
