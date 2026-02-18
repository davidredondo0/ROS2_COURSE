#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
from linkattacher_msgs.srv import Attach, Detach
from geometry_msgs.msg import Pose
import math
import tf2_ros
from tf2_ros import TransformException

class GraspingManager(Node):
    def __init__(self):
        super().__init__('grasping_manager')
        
        self.attach_client = self.create_client(Attach, '/attach')
        self.detach_client = self.create_client(Detach, '/detach')
        
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.model_sub = self.create_subscription(ModelStates, '/model_states', self.model_callback, 10)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.gripper_joint_name = 'robotiq_hande_left_finger_joint'
        # Hand-E: 0 (open) to 0.025 (closed).
        # "Halfway" is 0.0125.
        self.close_threshold = 0.0125
        self.open_threshold = 0.010
        
        self.object_name = 'conveyor_object'
        self.robot_model_name = 'ur3' 
        self.gripper_link_name = 'robotiq_hande_base_link' 
        self.object_link_name = 'link'
        
        self.current_gripper_pos = 0.0
        self.object_pose = None
        
        self.attached = False
        self.grasp_distance_threshold = 0.15 # 15cm
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Grasping Manager Started')

    def joint_callback(self, msg):
        if self.gripper_joint_name in msg.name:
            idx = msg.name.index(self.gripper_joint_name)
            self.current_gripper_pos = msg.position[idx]

    def model_callback(self, msg):
        if self.object_name in msg.name:
            idx = msg.name.index(self.object_name)
            self.object_pose = msg.pose[idx]
        else:
            self.object_pose = None

    def control_loop(self):
        if self.object_pose is None:
            return

        # Get Gripper Pose
        try:
            # Get transform from world to gripper
            t = self.tf_buffer.lookup_transform('world', self.gripper_link_name, rclpy.time.Time())
            gripper_pos = t.transform.translation
        except TransformException as ex:
            return

        # Calculate distance
        dx = gripper_pos.x - self.object_pose.position.x
        dy = gripper_pos.y - self.object_pose.position.y
        dz = gripper_pos.z - self.object_pose.position.z
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        # Logic
        if not self.attached:
            # If gripper is closed enough AND object is close
            if self.current_gripper_pos > self.close_threshold and dist < self.grasp_distance_threshold:
                self.attach_object()
        else:
            # If gripper opens
            if self.current_gripper_pos < self.open_threshold:
                self.detach_object()

    def attach_object(self):
        self.get_logger().info(f'Attaching object! Dist: {self.current_gripper_pos}')
        req = Attach.Request()
        req.model_name_1 = self.robot_model_name
        req.link_name_1 = self.gripper_link_name
        req.model_name_2 = self.object_name
        req.link_name_2 = self.object_link_name
        
        future = self.attach_client.call_async(req)
        self.attached = True

    def detach_object(self):
        self.get_logger().info('Detaching object!')
        req = Detach.Request()
        req.model_name_1 = self.robot_model_name
        req.link_name_1 = self.gripper_link_name
        req.model_name_2 = self.object_name
        req.link_name_2 = self.object_link_name
        
        future = self.detach_client.call_async(req)
        self.attached = False

def main(args=None):
    rclpy.init(args=args)
    manager = GraspingManager()
    rclpy.spin(manager)
    manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
