#!/usr/bin/env python3

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from autocar_msgs.msg import State2D


class Localisation(Node):

    def __init__(self):

        super().__init__('localisation')

        # Initialise publishers
        self.localisation_pub = self.create_publisher(State2D, '/autocar/state2D', 10)

        # Initialise subscribers
        self.odom_sub = self.create_subscription(Odometry, '/autocar/odom', self.vehicle_state_cb, 10)

        # Load parameters
        try:
            self.declare_parameters(
                namespace='',
                parameters=[
                    ('update_frequency', None)
                ]
            )

            self.frequency = float(self.get_parameter("update_frequency").value)

        except:
            raise Exception("Missing ROS parameters. Check the configuration file.")

        # Class constants
        self.state = None

    def vehicle_state_cb(self, msg):
        self.state = msg
        self.update_state()

    # Gets vehicle position from Gazebo and publishes data
    def update_state(self):

        # Define vehicle pose x,y, theta
        state2d = State2D()
        state2d.pose.x = self.state.pose.pose.position.x
        state2d.pose.y = self.state.pose.pose.position.y
        state2d.pose.theta = 2.0 * np.arctan2(self.state.pose.pose.orientation.z, self.state.pose.pose.orientation.w)
        
        # Aligning heading to y-axis, accounts for double rotation error
        if state2d.pose.theta < 0.0:
            state2d.pose.theta += 2.0 * np.pi
        
        # Define linear velocity x,y and angular velocity w
        state2d.twist.x = self.state.twist.twist.linear.x
        state2d.twist.y = self.state.twist.twist.linear.y
        state2d.twist.w = -self.state.twist.twist.angular.z

        self.localisation_pub.publish(state2d)

def main(args=None):

    # Initialise the node
    rclpy.init(args=args)

    try:
        # Initialise the class
        localisation = Localisation()

        # Stop the node from exiting
        rclpy.spin(localisation)

    finally:
        localisation.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()