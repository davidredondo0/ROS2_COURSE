#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity, SetEntityState, GetEntityState
from linkattacher_msgs.srv import AttachLink, DetachLink
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion
import time
import random
import math

class ConveyorManager(Node):
    def __init__(self):
        super().__init__('conveyor_manager')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self.set_state_client = self.create_client(SetEntityState, '/set_entity_state')
        self.get_state_client = self.create_client(GetEntityState, '/get_entity_state')

        self.conveyor_start_x = 1.3
        self.conveyor_end_x = 0.5
        self.conveyor_y = 0.0
        self.conveyor_z = 0.5 
        
        self.zones = {
            'zone_1': {'x': 0.0, 'y': 0.6, 'shape': 'cylinder'},
            'zone_2': {'x': -0.6, 'y': 0.0, 'shape': 'box'},
            'zone_3': {'x': 0.0, 'y': -0.6, 'shape': 'prism'}
        }
        
        self.object_counter = 0
        self.current_object_name = f"conveyor_object_{self.object_counter}"
        self.current_shape = None
        
        self.get_logger().info('Waiting for services...')
        if not self.spawn_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/spawn_entity service not available')
        if not self.delete_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/delete_entity service not available')
        if not self.set_state_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/set_entity_state service not available')
        if not self.get_state_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/get_entity_state service not available')
        self.get_logger().info('Services check complete')
        
        # Initialize ParallelGripper
        self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_client = self.create_client(DetachLink, '/DETACHLINK')
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        
        self.gripper_joint_name = 'robotiq_hande_left_finger_joint'
        self.gripper_closed_threshold = 0.001 # Adjust based on gripper
        self.current_gripper_pos = 0.0
        self.attached_object = None
        
    def joint_callback(self, msg):
        if self.gripper_joint_name in msg.name:
            idx = msg.name.index(self.gripper_joint_name)
            self.current_gripper_pos = msg.position[idx]
            self.check_grasping()

    def check_grasping(self):
        # If gripper is closed and it is not attached, try to attach
        if self.current_gripper_pos > self.gripper_closed_threshold and self.attached_object is None:
            # Check distance to object
            req = GetEntityState.Request()
            req.name = self.current_object_name
            future = self.get_state_client.call_async(req)
            pass

    def run(self):
        while rclpy.ok():
            if not self.spawn_object():
                time.sleep(1.0)
                continue

            self.move_object()
            
            # Wait loop with grasping check
            while True:
                # Check object position
                req = GetEntityState.Request()
                req.name = self.current_object_name
                future = self.get_state_client.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                res = future.result()
                
                if not res.success:
                    self.get_logger().warn("Object lost!")
                    break
                
                if res.success:
                    pos = res.state.pose.position
                    
                    # Check for pickup (z height or distance from conveyor end)
                    dist_from_end = math.sqrt((pos.x - self.conveyor_end_x)**2 + (pos.y - self.conveyor_y)**2)
                    
                    # Grasping Logic
                    if self.current_gripper_pos > self.gripper_closed_threshold and self.attached_object is None:
                        if dist_from_end < 0.05: # Close to pickup zone
                             if self.attach_object():
                                 break # Break to wait for placement
                             else:
                                 # If attach fails, just continue loop
                                 self.get_logger().warn("Attach failed, retrying...")
                                 time.sleep(1.0)
                    
                    # If already attached, check for placement
                    print("object attached:", self.attached_object)
                    print("gripper pos:", self.current_gripper_pos)
                    if self.attached_object is not None:
                        if self.current_gripper_pos < self.gripper_closed_threshold:
                            self.detach_object()
                            break # Break to delete object (or wait for placement confirmation)
                        
                
                time.sleep(0.1)
            
            # Wait for placement 
            while self.attached_object is not None:
                 # Update gripper pos
                 rclpy.spin_once(self, timeout_sec=0.1)
                 
                 if self.current_gripper_pos < self.gripper_closed_threshold:
                     self.detach_object()
                     break
                 time.sleep(0.1)
            
            # Check if placed in zone
            req = GetEntityState.Request()
            req.name = self.current_object_name
            future = self.get_state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            
            should_delete = False
            if future.result().success:
                pos = future.result().state.pose.position
                for zone_name, zone_data in self.zones.items():
                    dist = math.sqrt((pos.x - zone_data['x'])**2 + (pos.y - zone_data['y'])**2)
                    if dist < 0.08: # Threshold for zone
                        self.get_logger().info(f"Object placed in {zone_name}")
                        should_delete = True
                        break
            
            if should_delete:
                self.get_logger().info("Object placed in zone. Waiting 2s for fall...")
                time.sleep(2.0)
                self.delete_object()
            else:
                self.get_logger().info("Object dropped outside zone, letting gravity take over.")
            
            # Prepare for next object
            self.object_counter += 1
            self.current_object_name = f"conveyor_object_{self.object_counter}"
            time.sleep(1.0)

    def attach_object(self):
        self.get_logger().info(f'Attaching {self.current_object_name}')
        req = AttachLink.Request()
        req.model1_name = 'ur3' # Robot name
        req.link1_name = 'robotiq_hande_base_link' # Gripper base link
        req.model2_name = self.current_object_name
        req.link2_name = 'link'
        
        # Try different link names if the first one fails
        link_names = ['robotiq_hande_base_link', 'robotiq_coupler', 'tool0', 'wrist_3_link']
        
        for link in link_names:
            req.link1_name = link
            future = self.attach_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result().success:
                 self.attached_object = self.current_object_name
                 self.get_logger().info(f'Attached to {link}!')
                 return True
        
        self.get_logger().warn('Attach failed!')
        return False

    def detach_object(self):
        self.get_logger().info(f'Detaching {self.attached_object}')
        req = DetachLink.Request()
        req.model1_name = 'ur3'
        req.link1_name = 'robotiq_hande_base_link'
        req.model2_name = self.attached_object
        req.link2_name = 'link'
        
        # Try different link names if the first one fails
        link_names = ['robotiq_hande_base_link', 'robotiq_coupler', 'tool0', 'wrist_3_link']
        
        for link in link_names:
            req.link1_name = link
            future = self.detach_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result().success:
                 self.attached_object = None
                 self.get_logger().info(f'Detached from {link}!')
                 return

        self.get_logger().warn('Detach failed!')

    def spawn_object(self):
        # Ensure previous object is gone
        self.delete_object_silent()
        
        shapes = ['box', 'cylinder', 'prism']
        self.current_shape = random.choice(shapes)
        
        sdf = self.get_sdf(self.current_shape)
        
        req = SpawnEntity.Request()
        req.name = self.current_object_name
        req.xml = sdf
        req.initial_pose.position.x = self.conveyor_start_x
        req.initial_pose.position.y = self.conveyor_y
        req.initial_pose.position.z = self.conveyor_z
        
        future = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f'Spawned {self.current_shape}')
            time.sleep(1.0)
            return True
        else:
            self.get_logger().error(f'Failed to spawn: {future.result().status_message}')
            time.sleep(1.0)
            return False

    def delete_object_silent(self):
        req = DeleteEntity.Request()
        req.name = self.current_object_name
        future = self.delete_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def get_sdf(self, shape):
        # Added kinematic=true to avoid physics issues during transport
        kinematic = "<kinematic>0</kinematic>"
        if shape == 'box':
            return f"""
            <sdf version='1.6'>
                <model name='{self.current_object_name}'>
                    <link name='link'>
                        {kinematic}
                        <inertial>
                            <mass>0.1</mass>
                            <inertia>
                                <ixx>0.0001</ixx>
                                <ixy>0</ixy>
                                <ixz>0</ixz>
                                <iyy>0.0001</iyy>
                                <iyz>0</iyz>
                                <izz>0.0001</izz>
                            </inertia>
                        </inertial>
                        <collision name='collision'>
                            <geometry>
                                <box>
                                    <size>0.05 0.02 0.03</size>
                                </box>
                            </geometry>
                        </collision>
                        <visual name='visual'>
                            <geometry>
                                <box>
                                    <size>0.05 0.02 0.03</size>
                                </box>
                            </geometry>
                            <material>
                                <script>
                                    <name>Gazebo/Green</name>
                                    <uri>file://media/materials/scripts/gazebo.material</uri>
                                </script>
                            </material>
                        </visual>
                    </link>
                </model>
            </sdf>
            """
        elif shape == 'cylinder':
            return f"""
            <sdf version='1.6'>
                <model name='{self.current_object_name}'>
                    <link name='link'>
                        {kinematic}
                        <inertial>
                            <mass>0.1</mass>
                            <inertia>
                                <ixx>0.0001</ixx>
                                <ixy>0</ixy>
                                <ixz>0</ixz>
                                <iyy>0.0001</iyy>
                                <iyz>0</iyz>
                                <izz>0.0001</izz>
                            </inertia>
                        </inertial>
                        <collision name='collision'>
                            <geometry>
                                <cylinder>
                                    <radius>0.01</radius>
                                    <length>0.03</length>
                                </cylinder>
                            </geometry>
                        </collision>
                        <visual name='visual'>
                            <geometry>
                                <cylinder>
                                    <radius>0.01</radius>
                                    <length>0.03</length>
                                </cylinder>
                            </geometry>
                            <material>
                                <script>
                                    <name>Gazebo/Red</name>
                                    <uri>file://media/materials/scripts/gazebo.material</uri>
                                </script>
                            </material>
                        </visual>
                    </link>
                </model>
            </sdf>
            """
        elif shape == 'prism':
            return f"""
            <sdf version='1.6'>
                <model name='{self.current_object_name}'>
                    <link name='link'>
                        {kinematic}
                        <inertial>
                            <mass>0.1</mass>
                            <inertia>
                                <ixx>0.0001</ixx>
                                <ixy>0</ixy>
                                <ixz>0</ixz>
                                <iyy>0.0001</iyy>
                                <iyz>0</iyz>
                                <izz>0.0001</izz>
                            </inertia>
                        </inertial>
                        <collision name='collision'>
                            <geometry>
                                <polyline>
                                    <point>-0.015 -0.01</point>
                                    <point>0.015 -0.01</point>
                                    <point>0 0.015</point>
                                    <height>0.03</height>
                                </polyline>
                            </geometry>
                        </collision>
                        <visual name='visual'>
                            <geometry>
                                <polyline>
                                    <point>-0.015 -0.01</point>
                                    <point>0.015 -0.01</point>
                                    <point>0 0.015</point>
                                    <height>0.03</height>
                                </polyline>
                            </geometry>
                            <material>
                                <script>
                                    <name>Gazebo/Blue</name>
                                    <uri>file://media/materials/scripts/gazebo.material</uri>
                                </script>
                            </material>
                        </visual>
                    </link>
                </model>
            </sdf>
            """

    def move_object(self):
        current_x = self.conveyor_start_x
        step = 0.005
        self.get_logger().info(f'Starting conveyor movement from {current_x} to {self.conveyor_end_x}')
        while current_x > self.conveyor_end_x:
            current_x -= step
            req = SetEntityState.Request()
            req.state.name = self.current_object_name
            req.state.pose.position.x = current_x
            req.state.pose.position.y = self.conveyor_y
            req.state.pose.position.z = self.conveyor_z
            req.state.reference_frame = 'world'
            
            future = self.set_state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if not future.result().success:
                self.get_logger().warn(f'SetEntityState failed for {self.current_object_name}')
            
            time.sleep(0.05) # Control speed
        self.get_logger().info('Object reached end of conveyor')

    def wait_for_pickup(self):
        pass # Replaced by logic in run()

    def wait_for_placement(self):
        pass # Replaced by logic in run()

    def delete_object(self):
        req = DeleteEntity.Request()
        req.name = self.current_object_name
        future = self.delete_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Object deleted')

def main(args=None):
    print("Starting Conveyor Manager...")
    rclpy.init(args=args)
    manager = ConveyorManager()
    manager.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
