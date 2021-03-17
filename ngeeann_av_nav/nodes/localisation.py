import rclpy
import tf
import numpy as np

from rclpy.node import Node
from gazebo_msgs.srv import GetModelState  
from nav_msgs.msg import Odometry
from ngeeann_av_msgs.msg import State2D
from tf.transformations import quaternion_from_euler

class Localisation(Node):

    def __init__(self):

        super().__init__('localisation')

        # Initialise publishers
        self.localisation_pub = node.create_publisher(State2D, '/ngeeann_av/state2D')
        self.odom_pub = node.create_publisher(Odometry, '/ngeeann_av/odom')

        # Publishes artificial map frame
        self.map_broadcaster = tf.TransformBroadcaster()

        # Load parameters
        try:
            self.declare_parameter(
                namespace='',
                parameters=[
                    ('model_name'),
                ]
            )

            self.model = self.get_parameter("model_name")

        except:
            raise Exception("Missing ROS parameters. Check the configuration file.")

        # Class constants
        self.state = None

    # Gets vehicle position from Gazebo and publishes data
    def update_state(self):

        # Define vehicle pose x,y, theta
        state2d = State2D()
        state2d.pose.x = self.state.pose.position.x
        state2d.pose.y = self.state.pose.position.y
        state2d.pose.theta = 2.0 * np.arctan2(self.state.pose.orientation.z, self.state.pose.orientation.w)
        
        # Aligning heading to y-axis, accounts for double rotation error
        if state2d.pose.theta < 0.0:
            state2d.pose.theta += 2.0 * np.pi
        
        # Define linear velocity x,y and angular velocity w
        state2d.twist.x = self.state.twist.linear.x
        state2d.twist.y = self.state.twist.linear.y
        state2d.twist.w = -self.state.twist.angular.z

        # Publish odometry message
        odom = Odometry()
        odom.pose.pose.position.x = self.state.pose.position.x
        odom.pose.pose.position.y = self.state.pose.position.y
        odom.pose.pose.position.z = 0.0
        odom.twist.twist.linear.x = self.state.twist.linear.x
        odom.twist.twist.linear.y = self.state.twist.linear.y
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.z = self.state.twist.angular.z
        odom.pose.pose.orientation.x = self.state.pose.orientation.x
        odom.pose.pose.orientation.y = self.state.pose.orientation.y
        odom.pose.pose.orientation.z = self.state.pose.orientation.z
        odom.pose.pose.orientation.w = self.state.pose.orientation.w

        odom.header.stamp = node.get_clock().now().to_msg()
        odom.header.frame_id = "/map"
        odom.pose.pose.orientation = self.state.pose.orientation

        self.localisation_pub.publish(state2d)
        self.odom_pub.publish(odom)

        # Print state
        print("Position (x,y): ({},{})".format(round(state2d.pose.x, 5), round(state2d.pose.y, 5)))
        print("Heading: {}".format(round(state2d.pose.theta, 5)))
        print("Velocity (x,y): ({},{})".format(round(state2d.twist.x, 5), round(state2d.twist.y, 5)))

    # Publishes map frame transform
    def update_odom(self):

        # Publish transform
        x = self.state.pose.position.x
        y = self.state.pose.position.y
        z = self.state.pose.position.z
        odom_quat = [self.state.pose.orientation.x, self.state.pose.orientation.y, self.state.pose.orientation.z, self.state.pose.orientation.w]
        self.map_broadcaster.sendTransform((x, y, z), odom_quat, node.get_clock().now().to_msg(), "base_link", "map")

def main():

    # Initialise the class
    localisation = Localisation()

    # Initialise the node
    rclpy.init(args=args)
    node = rclpy.create_node('localisation')
    
    while not rclpy.ok():
        try:
            localisation.state = localisation.get_model_srv(localisation.model, '')
            localisation.update_state()
            localisation.update_odom()

            rclpy.spin(localisation)

        except KeyboardInterrupt:
            print("Shutting down ROS node...")

if __name__=="__main__":
    main()