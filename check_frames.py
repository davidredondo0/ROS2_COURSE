import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

class FrameListener(Node):
    def __init__(self):
        super().__init__('frame_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        frames = self.tf_buffer.all_frames_as_yaml()
        if frames:
            print(frames)
            rclpy.shutdown()

def main():
    rclpy.init()
    node = FrameListener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
