#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import threading

import math

from as2_python_api.drone_interface import DroneInterface
from as2_python_api.drone_interface_teleop import DroneInterfaceTeleop
from as2_python_api.mission_interpreter.mission import Mission
from as2_python_api.mission_interpreter.mission_interpreter import MissionInterpreter
from as2_msgs.msg import PlatformStatus
import sys

class FuzzyController:
    def __init__(self):
        pass
        
    def compute(self, error, d_error):
        # Constants
        # Error limits (pixels)
        E_NM = -300
        E_NS = -150
        E_PS = 150
        E_PM = 300
        
        # Derivative limits
        D_NM = -10
        D_NS = -5
        D_PS = 5
        D_PM = 10
        
        # Output weights (Singleton) - Yaw Speed (rad/s)
        O_NB = -0.2
        O_NM = -0.05
        O_NS = -0.03
        O_ZE = 0.0
        O_PS = 0.03
        O_PM = 0.05
        O_PB = 0.2
        
        # Fuzzification
        # Error
        mu_e_nb = 1.0 if error < E_NM else (0.0 if error > E_NS else (E_NS - error)/(E_NS - E_NM))
        mu_e_ns = 0.0 if error < E_NM or error > 0 else ((error - E_NM)/(E_NS - E_NM) if error < E_NS else (0 - error)/(0 - E_NS))
        mu_e_ze = 0.0 if error < E_NS or error > E_PS else ((error - E_NS)/(0 - E_NS) if error < 0 else (E_PS - error)/(E_PS - 0))
        mu_e_ps = 0.0 if error < 0 or error > E_PM else ((error - 0)/(E_PS - 0) if error < E_PS else (E_PM - error)/(E_PM - E_PS))
        mu_e_pb = 1.0 if error > E_PM else (0.0 if error < E_PS else (error - E_PS)/(E_PM - E_PS))
        
        # Derivative
        # Simplified to 3 sets for derivative: N, Z, P
        mu_d_n = 1.0 if d_error < D_NM else (0.0 if d_error > 0 else (0 - d_error)/(0 - D_NM))
        mu_d_z = 0.0 if d_error < D_NM or d_error > D_PM else ((d_error - D_NM)/(0 - D_NM) if d_error < 0 else (D_PM - d_error)/(D_PM - 0))
        mu_d_p = 1.0 if d_error > D_PM else (0.0 if d_error < 0 else (d_error - 0)/(D_PM - 0))
        
        # Rule Base & Inference
        act_nb = 0.0
        act_nm = 0.0
        act_ns = 0.0
        act_ze = 0.0
        act_ps = 0.0
        act_pm = 0.0
        act_pb = 0.0
        
        # Rule 1: If E is NB
        act_nb = max(act_nb, min(mu_e_nb, mu_d_n)) # D is N -> NB
        act_nb = max(act_nb, min(mu_e_nb, mu_d_z)) # D is Z -> NB
        act_nm = max(act_nm, min(mu_e_nb, mu_d_p)) # D is P -> NM

        # Rule 2: If E is NS
        act_nb = max(act_nb, min(mu_e_ns, mu_d_n)) # D is N -> NB
        act_ns = max(act_ns, min(mu_e_ns, mu_d_z)) # D is Z -> NS
        act_ze = max(act_ze, min(mu_e_ns, mu_d_p)) # D is P -> ZE

        # Rule 3: If E is ZE
        act_ns = max(act_ns, min(mu_e_ze, mu_d_n)) # D is N -> NS
        act_ze = max(act_ze, min(mu_e_ze, mu_d_z)) # D is Z -> ZE
        act_ps = max(act_ps, min(mu_e_ze, mu_d_p)) # D is P -> PS

        # Rule 4: If E is PS
        act_ze = max(act_ze, min(mu_e_ps, mu_d_n)) # D is N -> ZE
        act_ps = max(act_ps, min(mu_e_ps, mu_d_z)) # D is Z -> PS
        act_pb = max(act_pb, min(mu_e_ps, mu_d_p)) # D is P -> PB

        # Rule 5: If E is PB
        act_pm = max(act_pm, min(mu_e_pb, mu_d_n)) # D is N -> PM
        act_pb = max(act_pb, min(mu_e_pb, mu_d_z)) # D is Z -> PB
        act_pb = max(act_pb, min(mu_e_pb, mu_d_p)) # D is P -> PB
        
        # Defuzzification
        numerator = (act_nb * O_NB + act_nm * O_NM + act_ns * O_NS + 
                     act_ze * O_ZE + 
                     act_ps * O_PS + act_pm * O_PM + act_pb * O_PB)
        denominator = (act_nb + act_nm + act_ns + act_ze + act_ps + act_pm + act_pb)
        
        if denominator == 0:
            return 0.0
            
        return numerator / denominator

class RoadFollower(Node):
    def __init__(self, drone_interface, controller_type='pid'):
        super().__init__('road_follower')
        self.drone_interface = drone_interface
        self.controller_type = controller_type
        self.bridge = CvBridge()
        
        if self.controller_type == 'fuzzy':
            self.fuzzy_controller = FuzzyController()
            self.get_logger().info("Using Fuzzy Controller")
        elif self.controller_type == 'pid-fuzzy':
            self.fuzzy_controller = FuzzyController()
            self.active_controller = 'pid'
            self.small_error_start_time = None
            self.get_logger().info("Using PID-Fuzzy Hybrid Controller")
            
            # Hybrid PID constants
            self.kp_hybrid = 0.0001
            self.ki_hybrid = 0.0
            self.kd_hybrid = 0.001
        else:
            self.get_logger().info("Using PID Controller")
        
        # PID constants
        self.kp = 0.002
        self.ki = 0.0
        self.kd = 0.0009
        
        self.last_time = None
        self.max_yaw_speed = 0.6  # rad/s
        self.prev_error = 0
        self.integral = 0
        
        self.target_height = 6.0
        self.forward_speed = 1.0  # m/s
        
        self.image_width = None
        self.image_center_x = None
        
        self.centroid_x = None
        self.road_detected = False
        
        self.centroid_history = []
        self.new_measurement = False
        
        # Subscribe to camera
        self.subscription = self.create_subscription(
            Image,
            '/drone0/sensor_measurements/down_camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data)


        self.publisher = self.create_publisher(Image, 'road_following/debug_image', 10)
        
        self.error_pub = self.create_publisher(Float32, 'road_following/error', 10)
        self.yaw_speed_pub = self.create_publisher(Float32, 'road_following/yaw_speed', 10)
        self.setpoint_pub = self.create_publisher(Float32, 'road_following/setpoint', 10)
        self.drone_pos_pub = self.create_publisher(Float32, 'road_following/drone_position', 10)
            
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.start_following = False

    def image_callback(self, msg):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return
            
        self.image_width = cv_image.shape[1]
        self.image_center_x = self.image_width / 2
        
        # Segmentation
        # Convert to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Define range for gray color (road)
        lower_gray = np.array([0, 0, 50])
        upper_gray = np.array([180, 50, 255])
        
        mask = cv2.inRange(hsv, lower_gray, upper_gray)
        
        # Morphological operations to remove noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Calculate centroid of the white pixels
        M = cv2.moments(mask)
        
        # Create debug image (convert mask to BGR to draw colored points)
        debug_image = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            self.centroid_x = cx
            self.road_detected = True
            
            # Update history
            now = self.get_clock().now().nanoseconds / 1e9
            self.centroid_history.append((now, cx))
            if len(self.centroid_history) > 5:
                self.centroid_history.pop(0)
            self.new_measurement = True
            
            # Draw green point for centroid
            cv2.circle(debug_image, (cx, cy), 10, (0, 255, 0), -1)
        else:
            self.road_detected = False
            self.centroid_history = []

        # Draw red point for image center (drone position)
        height, width = mask.shape
        center_x = width // 2
        center_y = height // 2
        cv2.circle(debug_image, (center_x, center_y), 10, (0, 0, 255), -1)

        # Publish debug image
        debug_image_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
        self.publisher.publish(debug_image_msg)

    def control_loop(self):
        if not self.start_following:
            return
            
        target_x = None
        
        if self.road_detected and self.image_center_x is not None:
            if self.new_measurement:
                target_x = self.centroid_x
                self.new_measurement = False
            elif len(self.centroid_history) >= 2:
                # Predict
                times = np.array([p[0] for p in self.centroid_history])
                xs = np.array([p[1] for p in self.centroid_history])
                
                t0 = times[0]
                times_rel = times - t0
                
                # Linear regression
                A = np.vstack([times_rel, np.ones(len(times_rel))]).T
                m, c = np.linalg.lstsq(A, xs, rcond=None)[0]
                
                now = self.get_clock().now().nanoseconds / 1e9
                t_pred = now - t0
                target_x = m * t_pred + c
            else:
                target_x = self.centroid_x

        if target_x is not None:
            # PID Control
            error = self.image_center_x - target_x
            
            yaw_speed = 0.0

            # TO DO 
            #
            #
            #
            #
            #
            #
            # Limit yaw speed to avoid excessive rotation
            yaw_speed = max(min(yaw_speed, self.max_yaw_speed), -self.max_yaw_speed)  # Limit yaw speed
            
            self.prev_error = error

            # Publish debug data
            self.error_pub.publish(Float32(data=float(error)))
            self.yaw_speed_pub.publish(Float32(data=float(yaw_speed)))
            self.setpoint_pub.publish(Float32(data=float(target_x)))
            self.drone_pos_pub.publish(Float32(data=float(self.image_center_x)))
            
            # Height control (simple P controller)
            try:
                current_z = self.drone_interface.position[2]
                vz = 0.5 * (self.target_height - current_z)
            except:
                vz = 0.0

            print(f"Error: {error:.2f}, Yaw Speed: {yaw_speed:.2f}")
            print(f"Centroid X: {target_x:.2f}, Image Center X: {self.image_center_x:.2f}")
            
            # Convert body frame velocity to earth frame
            yaw = self.drone_interface.orientation[2]
            vx_earth = self.forward_speed * math.cos(yaw)
            vy_earth = self.forward_speed * math.sin(yaw)

            # Send command
            # vx = 0.5 m/s (forward)
            self.drone_interface.motion_ref_handler.speed.send_speed_command_with_yaw_speed(
                [vx_earth, vy_earth, vz], twist_frame_id='earth', yaw_speed=yaw_speed
            )
            # self.get_logger().info(f"Following road: Error={error:.2f}, Yaw={yaw_speed:.2f}")
        else:
            # Stop or hover if road lost
            self.drone_interface.motion_ref_handler.speed.send_speed_command_with_yaw_speed(
                [0.0, 0.0, 0.0], twist_frame_id='earth', yaw_speed=0.0
            )
            # self.get_logger().info("Road not detected, hovering")

def main():
    rclpy.init()
    
    # Parse arguments
    controller_type = 'pid'
    if len(sys.argv) > 1:
        # Simple check for arguments
        for arg in sys.argv:
            if arg == 'fuzzy':
                controller_type = 'fuzzy'
            elif arg == 'pid':
                controller_type = 'pid'
            elif arg == 'pid-fuzzy':
                controller_type = 'pid-fuzzy'
    
    print(f"Selected controller: {controller_type}")
    
    drone_interface = DroneInterfaceTeleop("drone0", verbose=True)
    
    road_follower = RoadFollower(drone_interface, controller_type=controller_type)
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(road_follower)
    
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    try:
        # Wait for info to be available
        print("Waiting for drone info...")
        time.sleep(2.0)

        if drone_interface.info['state'] == PlatformStatus.FLYING:
            print("Drone is already flying")
        else:
            print("Arming")
            drone_interface.arm()
            
            print("Offboard")
            drone_interface.offboard()
            
            print("Takeoff to 6m")
            drone_interface.takeoff(height=6.0, speed=1.0)
        
        print("Waiting 5 seconds")
        time.sleep(5)
        
        print("Starting Road Following")
        road_follower.start_following = True
        
        while rclpy.ok():
            time.sleep(1)
            
    except KeyboardInterrupt:
        pass
    finally:
        drone_interface.shutdown()
        road_follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
