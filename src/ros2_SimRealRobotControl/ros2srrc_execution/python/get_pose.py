#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
import time

class PoseListener(Node):
    def __init__(self):
        super().__init__('pose_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_pose(self):
        try:
            # Check for transform from base_link to EE_robotiq_hande (TCP)
            # If EE_robotiq_hande doesn't exist in TF tree, try tool0 or robotiq_hande_base_link
            target_frame = 'EE_robotiq_hande' 
            source_frame = 'base_link'
            
            if self.tf_buffer.can_transform(source_frame, target_frame, rclpy.time.Time()):
                t = self.tf_buffer.lookup_transform(source_frame, target_frame, rclpy.time.Time())
                print(f"\n=== Current Pose ({source_frame} -> {target_frame}) ===")
                print(f"x: {t.transform.translation.x:.4f}")
                print(f"y: {t.transform.translation.y:.4f}")
                print(f"z: {t.transform.translation.z:.4f}")
                print(f"qx: {t.transform.rotation.x:.4f}")
                print(f"qy: {t.transform.rotation.y:.4f}")
                print(f"qz: {t.transform.rotation.z:.4f}")
                print(f"qw: {t.transform.rotation.w:.4f}")
                print("==============================================")
                return True
            else:
                # self.get_logger().info(f'Waiting for transform {source_frame} -> {target_frame}...')
                return False
        except Exception as e:
            # self.get_logger().error(f'Error: {e}')
            return False

def main():
    rclpy.init()
    node = PoseListener()
    print("Listening for TF data...")
    try:
        start_time = time.time()
        while rclpy.ok():
            if node.get_pose():
                break
            if time.time() - start_time > 10.0:
                print("Timeout waiting for TF.")
                break
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
