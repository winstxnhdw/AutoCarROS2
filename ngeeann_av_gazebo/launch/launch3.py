import sys
import rclpy
from gazebo_msgs.srv import SpawnEntity

def request_spawn(xml: str):
    rclpy.init()
    node = rclpy.create_node('spawn_entity')
    client = node.create_client(SpawnEntity, 'spawn_entity')
    if not client.service_is_ready():
        client.wait_for_service()
    request = SpawnEntity.Request()
    request.xml = xml
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError('exception while calling service: %r' % future.exception())
    node.destroy_node()
    rclpy.shutdown()


if len(sys.argv) < 2:
    print('usage: ros2 run my_package my_node.py -- example.urdf')
    sys.exit(1)

f = open(sys.argv[1], 'r')
request_spawn(f.read())