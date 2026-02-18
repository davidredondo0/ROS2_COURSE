#!/usr/bin/python3

import sys, os, time
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np

# TF2 Imports
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# PATH setup
PATH = os.path.join(get_package_share_directory("ros2srrc_execution"), 'python')
PATH_ROB = PATH + "/robot"
PATH_EEGz = PATH + "/endeffector_gz"

sys.path.append(PATH_ROB)
from robot import RBT

sys.path.append(PATH_EEGz)
from parallelGripper import parallelGR

from ros2srrc_data.msg import Robpose
from ros2srrc_data.msg import Action
from ros2srrc_data.msg import Joints
from ros2srrc_data.msg import Joint
from ros2srrc_data.msg import Xyz
from ros2srrc_data.msg import Xyzypr
from ros2srrc_data.msg import Ypr

class ShapeDetector(Node):
    def __init__(self):
        super().__init__('shape_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.latest_image = None
        self.detected_shape = None

    def listener_callback(self, msg):
        self.latest_image = msg

    def analyze_shape(self):
        if self.latest_image is None:
            self.get_logger().warn("No image received yet.")
            return None

        cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # 1. Segment Object from Background
        # Red
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        mask_red = cv2.inRange(hsv_image, lower_red1, upper_red1) + cv2.inRange(hsv_image, lower_red2, upper_red2)
        
        # Green
        lower_green = np.array([40, 100, 100])
        upper_green = np.array([80, 255, 255])
        mask_green = cv2.inRange(hsv_image, lower_green, upper_green)
        
        # Blue
        lower_blue = np.array([100, 100, 100])
        upper_blue = np.array([140, 255, 255])
        mask_blue = cv2.inRange(hsv_image, lower_blue, upper_blue)

        # Combined Mask (ROI)
        mask_total = mask_red + mask_green + mask_blue
        
        # Clean noise
        kernel = np.ones((5,5), np.uint8)
        mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_OPEN, kernel)
        mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_CLOSE, kernel)

        # 2. Find Contours
        contours, _ = cv2.findContours(mask_total, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            print("No object detected.")
            return None

        # Assume largest contour is the object
        c = max(contours, key=cv2.contourArea)
        print(f"Contour Area: {cv2.contourArea(c)}")
        
        if cv2.contourArea(c) < 100:
            print("Object too small/noise.")
            return None

        # 3. Shape Analysis (Vertex Counting)
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)
        vertices = len(approx)
        print(f"Detected Vertices: {vertices}")
        
        # Show image with contours
        debug_img = cv_image.copy()
        cv2.drawContours(debug_img, [c], -1, (0, 255, 0), 2) # Original contour in Green
        cv2.drawContours(debug_img, [approx], -1, (0, 0, 255), 3) # Approx polygon in Red
        
        # Label vertices
        for p in approx:
            cv2.circle(debug_img, (p[0][0], p[0][1]), 5, (255, 255, 0), -1)
            
        cv2.putText(debug_img, f"Vertices: {vertices}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        cv2.imshow("Shape Debug", debug_img)
        cv2.waitKey(3000)
        
        shape = None
        if vertices == 3:
            shape = "prism"    # Triangle
            print(f"Vertices: {vertices} -> Detected: Triangle")
        elif vertices == 4:
            shape = "box"      # Square/Rect
            print(f"Vertices: {vertices} -> Detected: BOX")
        else:
            shape = "cylinder" # Circle/Polygon
            print(f"Vertices: {vertices} -> Detected: CYLINDER")
            
        return shape

def main():    
    
    step_done = False
    rclpy.init()
    
    print("Initializing Robot, Gripper and Vision...")

    robot = RBT()
    gripper = parallelGR(None, "ur3", "EE_robotiq_hande")
    detector = ShapeDetector()

    # Initialize TF Listener
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, detector)

    def get_and_print_pose():
        """
        Reads the current TF transform from 'base_link' to 'tool0' (Gripper TCP).
        Prints it to terminal and returns the transform object.
        """
        # Spin to update TF buffer
        for _ in range(10):
            rclpy.spin_once(detector, timeout_sec=0.05)
            
        try:
            # Look up transform
            # 'base_link' is usually the robot base. 'tool0' is the flange/tool.
            t = tf_buffer.lookup_transform(
                'base_link',
                'tool0',
                rclpy.time.Time()) # Get latest available
            
            p = t.transform.translation
            r = t.transform.rotation
            
            print(f"\n[POSE] Current TCP Pose (base_link -> tool0):")
            print(f"   Position (XYZ): [{p.x:.4f}, {p.y:.4f}, {p.z:.4f}]")
            print(f"   Rotation (xyzw): [{r.x:.4f}, {r.y:.4f}, {r.z:.4f}, {r.w:.4f}]")
            print("-" * 40)
            
            return t
        except TransformException as ex:
            print(f"[POSE] Could not transform base_link to tool0: {ex}")
            return None

    # ========================== MOVEMENT HELPER FUNCTIONS ========================== #
    # Difference between robot.Move_EXECUTE and robot.RobMove_EXECUTE:
    # 1. robot.Move_EXECUTE(action): 
    #    Executes predefined movement primitives (Joint, Linear, Relative, Rotation).
    #    Does not support custom path planners (like "circular"). 
    #    Ideally used for simple, standard movements.
    #
    # 2. robot.RobMove_EXECUTE(type, speed, pose):
    #    Executes a Cartesian movement to a specific absolute Target Pose.
    #    The 'type' string selects the planner/strategy (e.g. "cartesian", "circular", "PTP").
    #    Used when specific trajectory behavior is needed (e.g. circular arcs).

    def move_joints(j1, j2, j3, j4, j5, j6, speed=1.0):
        """
        MoveJ: Point-to-Point movement in Joint Space.
        Moves all joints designated values simultaneously.
        Best for: Fast movements in open space (e.g. Home to Approach).
        """
        print(f"Moving to Joints (deg): [{j1}, {j2}, {j3}, {j4}, {j5}, {j6}]")
        action = Action()
        action.action = "MoveJ"
        action.speed = speed
        
        INPUT = Joints()
        INPUT.joint1 = float(j1)
        INPUT.joint2 = float(j2)
        INPUT.joint3 = float(j3)
        INPUT.joint4 = float(j4)
        INPUT.joint5 = float(j5)
        INPUT.joint6 = float(j6)
        action.movej = INPUT
        
        result = robot.Move_EXECUTE(action)
        return result["Success"]

    def move_linear(x, y, z, speed=0.5):
        """
        MoveL: Linear movement in Cartesian Space.
        Computes absolute target by adding relative offset to current pose.
        Maintains current orientation, moves Tool Center Point (TCP) in straight line.
        Best for: Insertions, approaches, vertical moves.
        """
        
        # Get current pose to compute absolute target
        current_pose = get_and_print_pose()
        if current_pose is None:
            print("ERROR: Could not get current pose for linear movement")
            return False
        
        # Compute absolute target position
        tgt_x = current_pose.transform.translation.x + x
        tgt_y = current_pose.transform.translation.y + y
        tgt_z = current_pose.transform.translation.z + z
        
        # Keep current orientation
        tgt_qx = current_pose.transform.rotation.x
        tgt_qy = current_pose.transform.rotation.y
        tgt_qz = current_pose.transform.rotation.z
        tgt_qw = current_pose.transform.rotation.w
        
        print(f"Target Absolute Pos=[{tgt_x:.4f}, {tgt_y:.4f}, {tgt_z:.4f}]")
    
        pose = Robpose()
        pose.x = float(tgt_x)
        pose.y = float(tgt_y)
        pose.z = float(tgt_z)
        pose.qx = float(tgt_qx)
        pose.qy = float(tgt_qy)
        pose.qz = float(tgt_qz)
        pose.qw = float(tgt_qw)
        
        result = robot.RobMove_EXECUTE("LIN", speed, pose)
        return result["Success"]

    def move_relative(x, y, z, yaw, pitch, roll, speed=0.5):
        """
        MoveRP: Relative Position movement.
        Moves the robot relative to its current position/orientation (Base Frame).
        x,y,z in meters; yaw,pitch,roll in degrees.
        """
        print(f"Moving Relative: dPos=[{x}, {y}, {z}], dRot(deg)=[{yaw}, {pitch}, {roll}]")
        action = Action()
        action.action = "MoveRP"
        action.speed = speed
        
        INPUT = Xyzypr()
        INPUT.x = float(x)
        INPUT.y = float(y)
        INPUT.z = float(z)
        INPUT.yaw = float(yaw)
        INPUT.pitch = float(pitch)
        INPUT.roll = float(roll)
        action.moverp = INPUT
        
        result = robot.Move_EXECUTE(action)
        return result["Success"]

    def move_single_joint(joint_name, value, speed=0.5):
        """
        MoveR: Moves a specific single joint to a value (degrees).
        """
        print(f"Moving Single Joint: {joint_name} -> {value}")
        action = Action()
        action.action = "MoveR"
        action.speed = speed
        
        INPUT = Joint()
        INPUT.joint = joint_name
        INPUT.value = float(value)
        action.mover = INPUT
        
        result = robot.Move_EXECUTE(action)
        return result["Success"]

    def move_rotation(yaw, pitch, roll, speed=0.5):
        """
        MoveROT: Reorient the end-effector without changing position (X,Y,Z).
        Inputs in degrees.
        """
        print(f"Moving Rotation: Yaw={yaw}, Pitch={pitch}, Roll={roll}")
        action = Action()
        action.action = "MoveROT"
        action.speed = speed
        
        INPUT = Ypr()
        INPUT.yaw = float(yaw)
        INPUT.pitch = float(pitch)
        INPUT.roll = float(roll)
        action.moverot = INPUT
        
        result = robot.Move_EXECUTE(action)
        return result["Success"]

    def move_gripper_action(value, speed=0.5):
        """
        MoveG: Move external gripper axis (if configured as planning group).
        """
        print(f"Moving Gripper Axis: {value}")
        action = Action()
        action.action = "MoveG"
        action.speed = speed
        action.moveg = float(value)
        
        result = robot.Move_EXECUTE(action)
        return result["Success"]

    def move_cartesian(x, y, z, qx, qy, qz, qw, speed=0.5):
        """
        RobMove (Cartesian): Absolute Movement to target Pose (Pos+Quat).
        Uses 'cartesian' planner (Linear path).
        """
        print(f"Moving Cartesian: Pos=[{x}, {y}, {z}], Rot=[{qx}, {qy}, {qz}, {qw}]")
        pose = Robpose()
        pose.x = float(x)
        pose.y = float(y)
        pose.z = float(z)
        pose.qx = float(qx)
        pose.qy = float(qy)
        pose.qz = float(qz)
        pose.qw = float(qw)
        
        result = robot.RobMove_EXECUTE("cartesian", speed, pose)
        return result["Success"]

    def move_circular(x, y, z, qx, qy, qz, qw, speed=0.5):
        """
        RobMove (Circular): Absolute Movement to target Pose.
        Uses 'circular' planner (Arc path).
        """
        print(f"Moving Circular: Pos=[{x}, {y}, {z}], Rot=[{qx}, {qy}, {qz}, {qw}]")
        pose = Robpose()
        pose.x = float(x)
        pose.y = float(y)
        pose.z = float(z)
        pose.qx = float(qx)
        pose.qy = float(qy)
        pose.qz = float(qz)
        pose.qw = float(qw)
        
        result = robot.RobMove_EXECUTE("circular", speed, pose)
        return result["Success"]
    # ========================== END MOVEMENT HELPERS ========================== #

    
    while True:

        # Sequence Execution
        gripper.OPEN()
        
        print("1. Moving to Home...")
        move_joints(90.0, -90.0, 90.0, -90.0, -90.0, 0.0)
        
        print("2. Moving to Initial Pose...")
        move_joints(90.0, -90.0, 90.0, -90.0, -90.0, 0.0)

        print("3. Moving to Analice Approach ...")
        move_joints(90.0, -90.0, 90.0, -90.0, -90.0, 0.0)
        time.sleep(2.0)
        
        # Spin detector to get image
        print("4. Analyzing Shape...")
        shape = None
        while shape is None:
            # Spin a bit to process callbacks
            for _ in range(5): 
                rclpy.spin_once(detector, timeout_sec=0.1)
            
            # Try to detect
            shape = detector.analyze_shape()
            
            if shape is None:
                print("No shape detected, retrying...")
                time.sleep(2.0)
            
        print(f"Shape Identified: {shape}")

        print("5. Moving to Pick Position ...")
        # MoveL is a RELATIVE movement 
        print(f"   Moving Down 12cm...")
        move_joints(90.0, -90.0, 90.0, -90.0, -90.0, 0.0, speed=0.25)
        time.sleep(3.0)
        gripper.CLOSE(20.0)
        
        print("6. Moving to loading Position ...")
        move_joints(90.0, -90.0, 90.0, -90.0, -90.0, 0.0)

        print("7. Moving to Place Approach...")
        if shape == "box": # GREEN
            print("Target: GREEN Zone")
            move_joints(90.0, -90.0, 90.0, -90.0, -90.0, 0.0)
        elif shape == "cylinder": # RED
            print("Target: RED Zone")
            move_joints(90.0, -90.0, 90.0, -90.0, -90.0, 0.0)
        elif shape == "prism": # BLUE
            print("Target: BLUE Zone")
            move_joints(90.0, -90.0, 90.0, -90.0, -90.0, 0.0)

        gripper.OPEN()
        print("Sequence Completed.")
        
        time.sleep(6.0)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

