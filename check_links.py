import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState

def main():
    rclpy.init()
    node = Node('check_link')
    client = node.create_client(GetEntityState, '/get_entity_state')
    while not client.wait_for_service(timeout_sec=1.0):
        print("Waiting for service...")
    
    req = GetEntityState.Request()
    req.name = 'ur3'
    req.reference_frame = 'world'
    
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    print(f"UR3 Model State: {future.result().success}")

    # Check link
    req.name = 'ur3::robotiq_hande_base_link'
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    print(f"Link 'ur3::robotiq_hande_base_link': {future.result().success}")
    
    req.name = 'ur3::base_link'
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    print(f"Link 'ur3::base_link': {future.result().success}")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
